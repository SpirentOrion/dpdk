#   BSD LICENSE
#
#   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

include $(RTE_SDK)/mk/rte.vars.mk

LIB = librte_eal.a

EXPORT_MAP := rte_eal_version.map

LIBABIVER := 3

VPATH += $(RTE_SDK)/lib/librte_eal/common
VPATH += $(RTE_SDK)/lib/librte_eal/common/arch/$(ARCH_DIR)

INC_FLAGS := -I$(SRCDIR)/include
INC_FLAGS += -I$(RTE_SDK)/lib/librte_eal/common
INC_FLAGS += -I$(RTE_SDK)/lib/librte_eal/common/include
INC_FLAGS += -I$(RTE_SDK)/lib/librte_malloc
INC_FLAGS += -I$(RTE_SDK)/lib/librte_ether
INC_FLAGS += -I$(RTE_SDK)/lib/librte_pmd_ring
INC_FLAGS += -I$(RTE_SDK)/lib/librte_pmd_pcap
INC_FLAGS += -I$(RTE_SDK)/lib/librte_pmd_af_packet
INC_FLAGS += -I$(RTE_SDK)/lib/librte_pmd_xenvirt
INC_FLAGS += -I$(OSV_SDK)
INC_FLAGS += -I$(OSV_SDK)/arch/x64
INC_FLAGS += -I$(OSV_SDK)/arch/common
INC_FLAGS += -I$(OSV_SDK)/include

CFLAGS += $(INC_FLAGS) $(WERROR_FLAGS) -O3
CXXFLAGS += $(INC_FLAGS) -std=gnu++11 -O3

# Need to accept the existence of OSv specific functions on faith
CC_SUPPORTS_Z = false

LDLIBS += -ldl
LDLIBS += -lpthread
LDLIBS += -lgcc_s

# specific to osvapp exec-env
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) := eal.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_hugepage_info.cc
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_memory.cc
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_thread.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_pci.cc
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_debug.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_lcore.cc
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_timer.cc
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_interrupts.cc
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_interrupts_osv.cc
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_alarm.cc

# from common dir
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_memzone.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_log.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_launch.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_vdev.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_pci.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_memory.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_tailqs.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_errno.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_cpuflags.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_string_fns.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_hexdump.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_devargs.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_bus.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_dev.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_options.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_thread.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += eal_common_timer.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += rte_malloc.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += malloc_elem.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += malloc_heap.c
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += rte_keepalive.c

# from arch dir
SRCS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += rte_cpuflags.c
SRCS-$(CONFIG_RTE_ARCH_X86) += rte_spinlock.c

CFLAGS_eal_common_cpuflags.o := $(CPUFLAGS_LIST)

CFLAGS_eal.o := -D_GNU_SOURCE
CFLAGS_eal_lcore.o := -D_GNU_SOURCE
CFLAGS_eal_thread.o := -D_GNU_SOURCE
CFLAGS_eal_log.o := -D_GNU_SOURCE
CFLAGS_eal_common_log.o := -D_GNU_SOURCE
CFLAGS_eal_hugepage_info.o := -D_GNU_SOURCE
CFLAGS_eal_pci.o := -D_GNU_SOURCE
CFLAGS_eal_common_whitelist.o := -D_GNU_SOURCE
CFLAGS_eal_common_options.o := -D_GNU_SOURCE
CFLAGS_eal_common_thread.o := -D_GNU_SOURCE

# workaround for a gcc bug with noreturn attribute
# http://gcc.gnu.org/bugzilla/show_bug.cgi?id=12603
ifeq ($(CONFIG_RTE_TOOLCHAIN_GCC),y)
CFLAGS_eal_thread.o += -Wno-return-type
endif

INC := rte_interrupts.h

SYMLINK-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP)-include/exec-env := \
	$(addprefix include/exec-env/,$(INC))

DEPDIRS-$(CONFIG_RTE_LIBRTE_EAL_OSVAPP) += lib/librte_eal/common

include $(RTE_SDK)/mk/rte.lib.mk
