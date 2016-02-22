/*
 * Copyright (c) 2016 Spirent Communications, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "eal_interrupts_osv.hh"
#include "eal_interrupts_osv.h"

#include <sys/epoll.h>

#include <rte_common.h>
#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_pci.h>

#include <osv/mutex.h>
#include <osv/pci.hh>

int eal_interrupts_osv_setup(struct rte_intr_handle *handle,
			pci::device *device)
{
	if (handle == nullptr || device == nullptr) {
		return -EINVAL;
	}

	handle->device = device;

	if (device->is_msi() || device->is_msix()) {
		auto msi = new interrupt_manager(device);

		if (device->is_msix()) {
			device->msix_enable();
		} else {
			device->msi_enable();
		}

		auto vectors = msi->request_vectors(1);
		auto pv = new rte::pollable_vector(vectors[0]);

		msi->assign_isr(vectors[0], [pv] { return pv->ack(); });
		if (!msi->setup_entry(0, vectors[0])) {
			delete pv;
			delete msi;
			return -EFAULT;
		}

		handle->type = RTE_INTR_HANDLE_MSI;
		handle->msi = msi;
		handle->pollable = reinterpret_cast<struct rte_pollable *>(pv);
		handle->max_intr = 1;
	} else {
		handle->type = RTE_INTR_HANDLE_UNKNOWN;
		handle->max_intr = 0;
	}

	return (0);
}

void
rte::pollable::set_callback(std::function<void ()> callback)
{
        _callback = callback;
}

void
rte::pollable::on_wake()
{
        if (_callback) {
                _callback();
        }

        enable();
}

bool
rte::pollable_vector::ack()
{
	disable();
	wake();
	return true;
}

void
rte::pollable_vector::enable()
{
	_vector->msix_unmask_entries();
}

void
rte::pollable_vector::disable()
{
	_vector->msix_mask_entries();
}

void
rte::poller::reset(size_t hint)
{
	SCOPE_LOCK(_lock);
	_data.clear();
	_data.reserve(hint);
}

void
rte::poller::add(void *data)
{
	SCOPE_LOCK(_lock);
	_data.push_back(data);
}

size_t
rte::poller::get(struct rte_epoll_event *events, size_t max_events)
{
	size_t nb_events = RTE_MIN(_data.size(), max_events);
	for (size_t i = 0; i < nb_events; i++) {
		events[i].status       = RTE_EPOLL_VALID;
		events[i].epdata.event = EPOLLIN;
		events[i].epdata.data  = _data[i];
	}

	return nb_events;
}

bool
rte::intr_source::is_active()
{
	return _active.load(std::memory_order_relaxed);
}

int
rte::intr_source::add(rte_intr_callback_fn cb_fn, void *cb_arg)
{
	/*
	 * Seems kind of silly to allocate this memory from an rte
	 * routine given how many objects C++ objects we're allocating
	 * off the heap, but we can try to do what the linuxapp does. :)
	 */
	struct rte_intr_callback *cb = static_cast<struct rte_intr_callback *>(
		rte_zmalloc("interrupt callback list", sizeof(*cb), 0));
	if (cb == nullptr) {
		RTE_LOG(ERR, EAL, "Cannot allocate memory\n");
		return -ENOMEM;
	}

	cb->cb_fn = cb_fn;
	cb->cb_arg = cb_arg;

	if (is_active()) {
		rte_free(cb);
		return -EAGAIN;
	}

	SCOPE_LOCK(_lock);
	_callbacks.push_back(cb);

	return 0;
}

int
rte::intr_source::del(rte_intr_callback_fn cb_fn, void *cb_arg)
{
	if (is_active()) {
		return -EAGAIN;
	}

	SCOPE_LOCK(_lock);

	size_t old_size = _callbacks.size();
	_callbacks.erase(
		std::remove_if(
			_callbacks.begin(),
			_callbacks.end(),
			[cb_fn, cb_arg](struct rte_intr_callback *cb) ->bool {
				if (cb_fn == cb->cb_fn
					&& (cb_arg == reinterpret_cast<void *>(-1)
						|| cb_arg == cb->cb_arg)) {
					rte_free(cb);
					return true;
				}
				return false;
			}),
		_callbacks.end());
	size_t new_size = _callbacks.size();

	return (old_size - new_size);
}


void
rte::intr_source::run()
{
	WITH_LOCK(_lock) {
		_active.store(true, std::memory_order_release);
		for (auto cb : _callbacks) {
			cb->cb_fn(_handle, cb->cb_arg);
		}
		_active.store(false, std::memory_order_release);
	}
}

extern "C" {

	struct rte_pollable *
	rte_pollable_allocate(void)
	{
		rte::pollable *p = new rte::pollable();
		return reinterpret_cast<struct rte_pollable *>(p);
	}

	void rte_pollable_wake(struct rte_pollable *pollable)
	{
		rte::pollable *src = reinterpret_cast<rte::pollable *>(pollable);
		src->wake();
	}

	void rte_pollable_free(struct rte_pollable **pollablep)
	{
		struct rte_pollable *pollable = *pollablep;
		rte::pollable *p = reinterpret_cast<rte::pollable *>(pollable);
		delete p;
		*pollablep = nullptr;
	}
}
