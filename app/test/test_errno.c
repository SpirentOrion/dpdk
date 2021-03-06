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

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <rte_per_lcore.h>
#include <rte_errno.h>
#include <rte_string_fns.h>

#include "test.h"

#if defined(RTE_EXEC_ENV_BSDAPP)
static
void get_unknown_error(char *buf, size_t len, int err)
{
	snprintf(buf, len, "Unknown error: %d", err);
}
#elif defined(RTE_EXEC_ENV_LINUXAPP)
static
void get_unknown_error(char *buf, size_t len, int err)
{
	snprintf(buf, len, "Unknown error %d", err);
}
#elif defined(RTE_EXEC_ENV_OSVAPP)
static
void get_unknown_error(char *buf, size_t len, int err)
{
	(void)err;
	snprintf(buf, len, "No error information");
}
#else
#error "No get_unknown_error function defined for this platform."
#endif

static int
test_errno(void)
{
	const char *rte_retval;
	const char *libc_retval;

	char expected_libc_retval[1024];

	/* use a small selection of standard errors for testing */
	int std_errs[] = {EAGAIN, EBADF, EACCES, EINTR, EINVAL};
	/* test ALL registered RTE error codes for overlap */
	int rte_errs[] = {E_RTE_SECONDARY, E_RTE_NO_CONFIG};
	unsigned i;

	rte_errno = 0;
	if (rte_errno != 0)
		return -1;
	/* check for standard errors we return the same as libc */
	for (i = 0; i < sizeof(std_errs)/sizeof(std_errs[0]); i++){
		rte_retval = rte_strerror(std_errs[i]);
		libc_retval = strerror(std_errs[i]);
		printf("rte_strerror: '%s', strerror: '%s'\n",
				rte_retval, libc_retval);
		if (strcmp(rte_retval, libc_retval) != 0)
			return -1;
	}
	/* for rte-specific errors ensure we return a different string
	 * and that the string for libc is for an unknown error
	 */
	for (i = 0; i < sizeof(rte_errs)/sizeof(rte_errs[0]); i++){
		rte_retval = rte_strerror(rte_errs[i]);
		libc_retval = strerror(rte_errs[i]);
		printf("rte_strerror: '%s', strerror: '%s'\n",
				rte_retval, libc_retval);
		if (strcmp(rte_retval, libc_retval) == 0)
			return -1;
		/* generate appropriate error string for unknown error number
		 * and then check that this is what we got back. If not, we have
		 * a duplicate error number that conflicts with errno.h */
		get_unknown_error(expected_libc_retval, sizeof(expected_libc_retval),
				rte_errs[i]);
		if ((strcmp(expected_libc_retval, libc_retval) != 0) &&
				(strcmp("", libc_retval) != 0)){
			printf("Error, duplicate error code %d\n", rte_errs[i]);
			return -1;
		}
	}

	/* ensure that beyond RTE_MAX_ERRNO, we always get an unknown code */
	rte_retval = rte_strerror(RTE_MAX_ERRNO + 1);
	libc_retval = strerror(RTE_MAX_ERRNO + 1);
	get_unknown_error(expected_libc_retval, sizeof(expected_libc_retval),
			RTE_MAX_ERRNO + 1);
	printf("rte_strerror: '%s', strerror: '%s'\n",
			rte_retval, libc_retval);
	if ((strcmp(rte_retval, libc_retval) != 0) ||
			(strcmp(expected_libc_retval, libc_retval) != 0)){
		if (strcmp("", libc_retval) != 0){
			printf("Failed test for RTE_MAX_ERRNO + 1 value\n");
			return -1;
		}
	}

	return 0;
}

REGISTER_TEST_COMMAND(errno_autotest, test_errno);
