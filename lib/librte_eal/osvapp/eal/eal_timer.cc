/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
 *   Copyright(c) 2012-2013 6WIND S.A.
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <errno.h>

#include <rte_common.h>
#include <rte_log.h>
#include <rte_cycles.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_eal.h>
#include <rte_debug.h>

#include "eal_private.h"
#include "eal_internal_cfg.h"

#include <cpuid.hh>
#include <osv/clock.hh>
#include <osv/sched.hh>

enum timer_source eal_timer_source = EAL_TIMER_TSC;

using namespace osv::clock::literals;

uint64_t
rte_get_hpet_hz(void)
{
        if (internal_config.no_hpet)
                rte_panic("Error, HPET called, but no HPET present\n");

        return std::chrono::duration_cast<std::chrono::nanoseconds>(1_s).count();
}

uint64_t
rte_get_hpet_cycles(void)
{
        if (internal_config.no_hpet)
                rte_panic("Error, HPET called, but no HPET present\n");

        auto now = osv::clock::uptime::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

static void
check_tsc_flags(void)
{
        if (!processor::features().invariant_tsc) {
                RTE_LOG(WARNING, EAL,
                        "WARNING: cpu_flags "
                        "invariant_tsc=no "
                        "-> using unreliable clock cycles!\n");
        }
}

uint64_t
get_tsc_freq(void)
{
        auto clock_start = osv::clock::uptime::now();
        uint64_t tsc_start = rte_rdtsc();

        sched::thread::sleep(100_ms);

        std::chrono::duration<double> clock_diff = osv::clock::uptime::now() - clock_start;
        uint64_t tsc_diff = rte_rdtsc() - tsc_start;

        uint64_t tsc_hz = tsc_diff / clock_diff.count();
        return tsc_hz;
}

int
rte_eal_timer_init(void)
{
        if (internal_config.no_hpet) {
                eal_timer_source = EAL_TIMER_TSC;
                set_tsc_freq();
                check_tsc_flags();
        } else {
                eal_timer_source = EAL_TIMER_HPET;
        }

	return 0;
}

int
rte_eal_hpet_init(int make_default)
{
	if (internal_config.no_hpet) {
		RTE_LOG(NOTICE, EAL, "HPET is disabled\n");
		return -1;
	}

	if (make_default) {
		eal_timer_source = EAL_TIMER_HPET;
	}

	return 0;
}
