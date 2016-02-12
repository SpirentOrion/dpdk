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
 * ARE DISCLAIMED.	IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef EAL_INTERRUPTS_OSV_HH_
#define EAL_INTERRUPTS_OSV_HH_

#include <rte_interrupts.h>

#include <osv/msi.hh>
#include <osv/newpoll.hh>

namespace pci {
	class device;
}

namespace rte {

	/**
	 * These pollable objects provide poll like semantics for hardware
	 * interrupts.  The DPDK API assumes interrupts are only ever relayed
	 * via file descriptors, so these objects allows us to provide the
	 * same functionality in osvapp as in linuxapp.
	 *
	 * Both the poller and pollable objects are based around OSv's
	 * newpoll implementation.
	 */

	/* Non-hardware dependant version for unit tests */
	class pollable : public osv::newpoll::pollable {
	public:
		pollable()
			: _callback(nullptr) {};
		virtual ~pollable() {};

		virtual void disable() {};
		virtual void enable() {};

		void on_wake();
		void set_callback(std::function<void ()> callback);
	private:
		std::function<void ()> _callback;
	};


	/*
	 * Used for MSI based interrupt vectors
	 * Note: This class takes ownership of the vector
	 */
	class pollable_vector : public pollable {
	public:
		pollable_vector(msix_vector *vector)
			: _vector(vector) {};
		virtual ~pollable_vector() { disable(); };

		void disable();  /* Disable hardware vector */
		void enable();   /* Enable hardware vector */

		bool ack();      /* disable() && wake() */
		unsigned get_vector() { return _vector->get_vector(); };
	private:
		std::unique_ptr<msix_vector> _vector;
	};

	/* Provides epoll like functionality for rte_epoll_wait */
	class poller : public osv::newpoll::poller {
	public:
		poller() {};
		virtual ~poller() {};

		void reset(size_t hint);
		void add(void *data);
		size_t get(struct rte_epoll_event *events, size_t max_events);
	private:
		std::vector<void *> _data;
		mutex _lock;
	};


	/*
	 * This container class holds client registered interrupt callbacks.
	 * The run method is used as the pollable callback when interrupts are
	 * trigger.
	 * This object is only used for the first (or only) MSI interrupt vector.
	 */
	class intr_source {
	public:
		intr_source(struct rte_intr_handle *handle)
			: _active(false), _handle(handle) {};
		virtual ~intr_source() {};

		bool is_active();
		int add(rte_intr_callback_fn cb_fn, void *cb_arg);
		int del(rte_intr_callback_fn cb_fn, void *cb_arg);

		void run();
	private:
		struct rte_intr_callback {
			rte_intr_callback_fn cb_fn;
			void *cb_arg;
		};

		std::atomic_bool _active;
		struct rte_intr_handle *_handle;
		std::vector<rte_intr_callback *> _callbacks;
		mutex _lock;
	};
}

int eal_interrupts_osv_setup(struct rte_intr_handle *handle,
			     pci::device *device);

#endif /* EAL_INTERRUPTS_OSV_HH_ */
