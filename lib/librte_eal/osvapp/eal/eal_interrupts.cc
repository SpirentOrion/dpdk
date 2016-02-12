/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <atomic>
#include <chrono>
#include <vector>
#include <unordered_map>

#include <rte_common.h>
#include <rte_eal.h>
#include <rte_interrupts.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_log.h>

#include "eal_private.h"
#include "eal_interrupts_osv.hh"

#include <osv/msi.hh>
#include <osv/pci.hh>
#include <osv/sched.hh>

#define EAL_INTR_EPOLL_WAIT_FOREVER (-1)
#define NB_OTHER_INTR		    1

static RTE_DEFINE_PER_LCORE(int, _poller_id) = -1;

/* pollable --> interrupt source map */
static std::unordered_map<rte::pollable_vector *,
			  std::unique_ptr<rte::intr_source> > intr_src_map;

/* Lock for modifying interrupt source map */
static mutex intr_lock = MUTEX_INITIALIZER;

/* Pollable object for waking up the interrupt handling thread */
static std::unique_ptr<rte::pollable> intr_thread_pollable;

/* interrupt thread */
sched::thread *intr_thread;

/*
 * List of rte::poller objects used for fast interrupt handling.
 * The index in the list corresponds to the thread local _poller_id.
 */
static std::vector<std::unique_ptr<rte::poller> > intr_pollers;

/* Lock for modifying intr_pollers vector */
static mutex intr_pollers_lock = MUTEX_INITIALIZER;

int
rte_intr_callback_register(struct rte_intr_handle *intr_handle,
			rte_intr_callback_fn cb_fn, void *cb_arg)
{
	/* sanity check input */
	if (intr_handle == nullptr
		|| intr_handle->pollable == nullptr
		|| cb_fn == nullptr) {
		RTE_LOG(ERR, EAL,
			"Registering with invalid input parameter\n");
		return -EINVAL;
	}

	rte::pollable_vector *p = reinterpret_cast<rte::pollable_vector *>(intr_handle->pollable);
	rte::intr_source *src = nullptr;
	bool do_updates = false;

	/* Find the pollable's interrupt source object */
	WITH_LOCK(intr_lock) {
		auto it = intr_src_map.find(p);
		if (it == intr_src_map.end()) {
			/* Create an interrupt source for this pollable */
			auto result = intr_src_map.emplace(
				p,
				std::unique_ptr<rte::intr_source>(new rte::intr_source(intr_handle)));
			assert(result.second == true);
			it = result.first;

			/* Perform some extra updates without the lock */
			do_updates = true;
		}
		src = it->second.get();
	}

	assert(src != nullptr);

	/* Add the new callback to the interrupt source */
	int error = src->add(cb_fn, cb_arg);
	if (error) {
		return error;
	}

	if (do_updates) {
		/* Update the pollable's callback */
		p->set_callback([src] { src->run(); });

		/* And wake up the interrupt thread */
		intr_thread_pollable->wake();
	}

	return 0;
}

int
rte_intr_callback_unregister(struct rte_intr_handle *intr_handle,
			rte_intr_callback_fn cb_fn, void *cb_arg)
{
	/* sanity check input */
	if (intr_handle == nullptr
		|| intr_handle->pollable == nullptr
		|| cb_fn == nullptr) {
		RTE_LOG(ERR, EAL,
			"Unregistering with invalid input parameter\n");
		return -EINVAL;
	}

	rte::pollable_vector *p = reinterpret_cast<rte::pollable_vector *>(intr_handle->pollable);
	rte::intr_source *src = nullptr;

	/* Find the pollable's interrupt source object */
	WITH_LOCK(intr_lock) {
		auto it = intr_src_map.find(p);
		if (it == intr_src_map.end()) {
			return -ENOENT;
		}
		src = it->second.get();
	}

	assert(src != nullptr);

	return src->del(cb_fn, cb_arg);
}

int
rte_intr_enable(struct rte_intr_handle *intr_handle)
{
	if (intr_handle == nullptr
		|| intr_handle->device == nullptr) {
		return -EINVAL;
	}

	pci::device *device = static_cast<pci::device *>(intr_handle->device);

	u8 bus, dev, func;
	device->get_bdf(bus, dev, func);

	switch (intr_handle->type) {
	case RTE_INTR_HANDLE_MSI:
	{
		rte::pollable_vector *p = reinterpret_cast<rte::pollable_vector *>(
			intr_handle->pollable);
		p->enable();
		RTE_LOG(DEBUG, EAL, "Enabled MSI interrupt %u on " PCI_PRI_FMT "\n",
			intr_handle->msi ? p->get_vector() : 0,
			0, bus, dev, func);

		size_t i = 0;
		for (i = 0; i < intr_handle->nb_pollables; i++) {
			p = reinterpret_cast<rte::pollable_vector *>(intr_handle->pollables[i]);
			p->enable();
			RTE_LOG(DEBUG, EAL, "Enabled MSI interrupt %u on " PCI_PRI_FMT "\n",
				intr_handle->msi ? p->get_vector() : 0,
				0, bus, dev, func);
		}

		break;
	}
	default:
	{
		RTE_LOG(ERR, EAL,
			"Unknown handle type for " PCI_PRI_FMT "\n",
			0, bus, dev, func);
		return -1;
	}
	}
	return 0;
}

int
rte_intr_disable(struct rte_intr_handle *intr_handle)
{
	if (intr_handle == nullptr
		|| intr_handle->device == nullptr) {
		return -EINVAL;
	}

	pci::device *device = static_cast<pci::device *>(intr_handle->device);
	uint8_t bus, dev, func;
	device->get_bdf(bus, dev, func);

	switch (intr_handle->type) {
	case RTE_INTR_HANDLE_MSI:
	{
		rte::pollable_vector *p = reinterpret_cast<rte::pollable_vector *>(
			intr_handle->pollable);
		p->disable();
		RTE_LOG(DEBUG, EAL, "Disabled MSI interrupt %u on " PCI_PRI_FMT "\n",
			intr_handle->msi ? p->get_vector() : 0,
			0, bus, dev, func);

		size_t i = 0;
		for (i = 0; i < intr_handle->nb_pollables; i++) {
			p = reinterpret_cast<rte::pollable_vector *>(intr_handle->pollables[i]);
			p->disable();
			RTE_LOG(DEBUG, EAL, "Disabled MSI interrupt %u on " PCI_PRI_FMT "\n",
				intr_handle->msi ? p->get_vector() : 0,
				0, bus, dev, func);

		}

		break;
	}
	default:
	{
		RTE_LOG(ERR, EAL,
			"Unknown handle type for " PCI_PRI_FMT "\n",
			0, bus, dev, func);
		return -1;
	}
	}

	return 0;
}

/**
 * It builds/rebuilds up the epoll file descriptor with all the
 * file descriptors being waited on. Then handles the interrupts.
 *
 * @param arg
 *  pointer. (unused)
 *
 * @return
 *  never return;
 */
static __attribute__((noreturn)) void *
eal_intr_thread_main(__rte_unused void *arg)
{
	std::unique_ptr<osv::newpoll::poller> poller;

	for (;;) {
		poller.reset(new osv::newpoll::poller());
		poller->add(intr_thread_pollable.get());

		WITH_LOCK(intr_lock) {
			for (auto &item : intr_src_map) {
				poller->add(item.first);
			}
		}

		// Wait until we get interrupted.
		poller->wait();

		// Run any pending callbacks */
		poller->process();
	}
}

int
rte_eal_intr_init(void)
{
	/* Create a pollable object for notifing the interrupt thread */
	intr_thread_pollable.reset(new rte::pollable());

	/* Create a host thread to wait for/handle interrupts */
	intr_thread = new sched::thread(
		[] { eal_intr_thread_main(nullptr); },
		sched::thread::attr().name("eal-intr-thread"));
	intr_thread->start();

	return 0;
}

int
rte_intr_tls_epfd(void)
{
	int epfd = -1;
	if ((epfd = RTE_PER_LCORE(_poller_id)) == -1) {
		SCOPE_LOCK(intr_pollers_lock);
		epfd = intr_pollers.size();
		intr_pollers.push_back(
			std::unique_ptr<rte::poller>(new rte::poller()));
		RTE_PER_LCORE(_poller_id) = epfd;
	}

	return epfd;
}

static rte::poller *
eal_get_pollable(int epfd)
{
	if (epfd == RTE_EPOLL_PER_THREAD) {
		epfd = rte_intr_tls_epfd();
	}

	assert(epfd != RTE_EPOLL_PER_THREAD);

	rte::poller *p = nullptr;
	WITH_LOCK(intr_pollers_lock) {
		p = intr_pollers[epfd].get();
	}
	return p;
}

int
rte_epoll_wait(int epfd, struct rte_epoll_event *events,
	int maxevents, int timeout)
{
	auto ep = eal_get_pollable(epfd);
	assert(ep != nullptr);

	/* Get setup for our attack run... */
	ep->reset(maxevents);

	if (timeout > 0) {
		auto expire = (std::chrono::high_resolution_clock::now()
			+ std::chrono::milliseconds(timeout));
		ep->set_timer(expire);
	}

	if (timeout != 0) {
		/*
		 * >0 means we have an explicit timeout, which is set
		 * <0 means we wait until something happens
		 */
		ep->wait();
	}

	if (!ep->expired()) {
		/* Timer didn't expire, so something needs to be serviced */
		ep->process();
	}

	size_t nb_events = ep->get(events, maxevents);
	if (nb_events == maxevents) {
		/* XXX: need to fix this */
		RTE_LOG(WARNING, EAL, "Maximum number of epoll events returned.	 "
			"Some events were probably lost.\n");
	}

	return nb_events;
}

int
rte_intr_rx_ctl(struct rte_intr_handle *intr_handle,
		int epfd, int op, unsigned int vec, void *data)
{
	unsigned int pollable_idx = (vec >= RTE_INTR_VEC_RXTX_OFFSET
				? vec - RTE_INTR_VEC_RXTX_OFFSET
				: vec);
	int rc = 0;

	if (intr_handle == nullptr
		|| intr_handle->nb_pollables == 0
		|| pollable_idx > intr_handle->nb_pollables) {
		RTE_LOG(ERR, EAL, "Wrong intr vector number.\n");
		return -EPERM; /* ? */
	}

	auto ep = eal_get_pollable(epfd);
	if (ep == nullptr) {
		RTE_LOG(ERR, EAL, "Invalid epfd.\n");
		return -EINVAL;
	}

	rte::pollable_vector *pv = reinterpret_cast<rte::pollable_vector *>(
		intr_handle->pollables[pollable_idx]);

	switch (op) {
	case RTE_INTR_EVENT_ADD:
		pv->set_callback(
			[ep, data] { ep->add(data); });
		ep->add(pv);
		RTE_LOG(DEBUG, EAL,
			"Added vector %u to poller id %d\n",
			pv->get_vector(), RTE_PER_LCORE(_poller_id));
		break;
	case RTE_INTR_EVENT_DEL:
		pv->set_callback(nullptr);
		ep->del(pv);
		RTE_LOG(DEBUG, EAL,
			"Removed vector %u from poller id %d\n",
			pv->get_vector(), RTE_PER_LCORE(_poller_id));
		break;
	default:
		RTE_LOG(ERR, EAL, "Unknown event op type\n");
		rc = -EPERM;
	}

	return rc;
}

int
rte_intr_efd_enable(struct rte_intr_handle *intr_handle, uint32_t nb_efd)
{
	size_t n = RTE_MIN(nb_efd, (uint32_t)RTE_MAX_RXTX_INTR_VEC_ID);

	assert(nb_efd != 0);

	if (intr_handle->type == RTE_INTR_HANDLE_MSI) {
		pci::device *device = static_cast<pci::device *>(intr_handle->device);
		interrupt_manager *msi = static_cast<interrupt_manager *>(intr_handle->msi);
		auto vectors = msi->request_vectors(n);
		if (vectors.size() != n) {
			msi->free_vectors(vectors);
			return -1;
		}

		for (size_t i = 0; i < n; i++) {
			auto v = vectors[i];
			rte::pollable_vector *pv = new rte::pollable_vector(v);
			msi->assign_isr(v, [pv] { return pv->ack(); });
			if (!msi->setup_entry(NB_OTHER_INTR + i, v)) {
				uint8_t bus, dev, func;
				device->get_bdf(bus, dev, func);
				RTE_LOG(ERR, EAL, "cannot setup entry %zu for "
					"vector %u on " PCI_PRI_FMT "\n",
					NB_OTHER_INTR + i, v->get_vector(),
					0, bus, dev, func);
				for (size_t j = 0; j < n; j++) {
					if (j < i) {
						/* Delete previous pollables + vectors */
						delete reinterpret_cast<rte::pollable_vector *>(
							intr_handle->pollables[j]);
						intr_handle->pollables[j] = 0;
					} else if (j == i) {
						/* Delete current pollable + vector */
						delete pv;
					} else {
						/* Delete unused vectors */
						delete vectors[i];
					}
				}
				return -1;
			}

			intr_handle->pollables[i] = reinterpret_cast<struct rte_pollable *>(pv);

			RTE_LOG(DEBUG, EAL,
				"  Entry %zu --> MSI vector %u\n", NB_OTHER_INTR + i, pv->get_vector());
		}
		intr_handle->nb_pollables = n;
		intr_handle->max_intr = NB_OTHER_INTR + n;
	}

	return 0;
}

void
rte_intr_efd_disable(struct rte_intr_handle *intr_handle)
{
	size_t i = 0;
	if (intr_handle->type == RTE_INTR_HANDLE_MSI) {
		for (i = 0; i < intr_handle->nb_pollables; i++) {
			rte::pollable_vector *p = reinterpret_cast<rte::pollable_vector *>(
				intr_handle->pollables[i]);
			p->disable();
			delete p;
			intr_handle->pollables[i] = nullptr;
		}
		intr_handle->nb_pollables = 0;
		intr_handle->max_intr = 1;
	}
}

int
rte_intr_dp_is_en(struct rte_intr_handle *intr_handle)
{
	return (intr_handle->nb_pollables > 0 ? 1 : 0);
}

int
rte_intr_allow_others(struct rte_intr_handle *intr_handle)
{
        if (!rte_intr_dp_is_en(intr_handle)
                || intr_handle->max_intr > intr_handle->nb_pollables) {
                return 1;
        }

        return 0;
}

int
rte_intr_cap_multiple(struct rte_intr_handle *intr_handle)
{
	return (intr_handle->type == RTE_INTR_HANDLE_MSI ? 1 : 0);
}
