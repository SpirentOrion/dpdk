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

#ifndef _RTE_INTERRUPTS_H_
#error "don't include this file directly, please include generic <rte_interrupts.h>"
#endif

#include <stdint.h>

#ifndef _RTE_OSVAPP_INTERRUPTS_H_
#define _RTE_OSVAPP_INTERRUPTS_H_

#define RTE_MAX_RXTX_INTR_VEC_ID     32
#define RTE_INTR_VEC_ZERO_OFFSET      0
#define RTE_INTR_VEC_RXTX_OFFSET      1

enum rte_intr_handle_type {
        RTE_INTR_HANDLE_UNKNOWN = 0,
        RTE_INTR_HANDLE_MSI,
        RTE_INTR_HANDLE_MAX
};

#define RTE_INTR_EVENT_ADD            1UL
#define RTE_INTR_EVENT_DEL            2UL

typedef void (*rte_intr_event_cb_t)(int fd, void *arg);

struct rte_epoll_data {
	uint32_t event;               /**< event type */
	void *data;                   /**< User data */
};

enum {
	RTE_EPOLL_INVALID = 0,
	RTE_EPOLL_VALID,
	RTE_EPOLL_EXEC,
};

/** interrupt epoll event obj, taken by epoll_event.ptr */
struct rte_epoll_event {
	volatile uint32_t status;  /**< OUT: event status */
	struct rte_epoll_data epdata;
};

struct rte_intr_pollable;

/** Handle for interrupts. */
struct rte_intr_handle {
	struct rte_pollable *pollable;   /**< primary interrupt poll object */
	void *device;                    /**< pointer to OSv device object */
	void *msi;                       /**< pointer to OSv MSI interrupt manager */
	enum rte_intr_handle_type type;  /**< handle type */
	uint32_t max_intr;               /**< max interrupt requested */
	/*
	 * Annoyingly, drivers query nb_efd directly.  The union is just to point
	 * those encapsulation borked drivers to the right value.
	 */
	union {
		uint32_t nb_efd;         /**< number of avaiable event fds */
		uint32_t nb_pollables;   /**< number of available pollable objects */
	};
	uint8_t pad[4];
	struct rte_pollable *pollables[RTE_MAX_RXTX_INTR_VEC_ID];
	                                 /**< pollable objects for fastpath interrupts */
	int *intr_vec;			 /**< intr vector number array */

};

#define RTE_EPOLL_PER_THREAD        -1  /**< to hint using per thread epfd */

/**
 * It waits for events on the epoll instance.
 *
 * @param epfd
 *   Epoll instance fd on which the caller wait for events.
 * @param events
 *   Memory area contains the events that will be available for the caller.
 * @param maxevents
 *   Up to maxevents are returned, must greater than zero.
 * @param timeout
 *   Specifying a timeout of -1 causes a block indefinitely.
 *   Specifying a timeout equal to zero cause to return immediately.
 * @return
 *   - On success, returns the number of available event.
 *   - On failure, a negative value.
 */
int
rte_epoll_wait(int epfd, struct rte_epoll_event *events,
	       int maxevents, int timeout);

/**
 * The function returns the per thread epoll instance.
 *
 * @return
 *   epfd the epoll instance referred to.
 */
int
rte_intr_tls_epfd(void);

/**
 * @param intr_handle
 *   Pointer to the interrupt handle.
 * @param epfd
 *   Epoll instance fd which the intr vector associated to.
 * @param op
 *   The operation be performed for the vector.
 *   Operation type of {ADD, DEL}.
 * @param vec
 *   RX intr vector number added to the epoll instance wait list.
 * @param data
 *   User raw data.
 * @return
 *   - On success, zero.
 *   - On failure, a negative value.
 */
int
rte_intr_rx_ctl(struct rte_intr_handle *intr_handle,
		int epfd, int op, unsigned int vec, void *data);

/**
 * It enables the fastpath event fds if it's necessary.
 * It creates event fds when multi-vectors allowed,
 * otherwise it multiplexes the single event fds.
 *
 * @param intr_handle
 *   Pointer to the interrupt handle.
 * @param nb_efd
 *   Number of interrupt vector trying to enable.
 *   The value 0 is not allowed.
 * @return
 *   - On success, zero.
 *   - On failure, a negative value.
 */
int
rte_intr_efd_enable(struct rte_intr_handle *intr_handle, uint32_t nb_efd);

/**
 * It disable the fastpath event fds.
 * It deletes registered eventfds and closes the open fds.
 *
 * @param intr_handle
 *   Pointer to the interrupt handle.
 */
void
rte_intr_efd_disable(struct rte_intr_handle *intr_handle);

/**
 * The fastpath interrupt is enabled or not.
 *
 * @param intr_handle
 *   Pointer to the interrupt handle.
 */
int rte_intr_dp_is_en(struct rte_intr_handle *intr_handle);

/**
 * The interrupt handle instance allows other cause or not.
 * Other cause stands for none fastpath interrupt.
 *
 * @param intr_handle
 *   Pointer to the interrupt handle.
 */
int rte_intr_allow_others(struct rte_intr_handle *intr_handle);

/**
 * The multiple interrupt vector capability of interrupt handle instance.
 * It returns zero if no multiple interrupt vector support.
 *
 * @param intr_handle
 *   Pointer to the interrupt handle.
 */
int
rte_intr_cap_multiple(struct rte_intr_handle *intr_handle);

#endif /* _RTE_OSVAPP_INTERRUPTS_H_ */
