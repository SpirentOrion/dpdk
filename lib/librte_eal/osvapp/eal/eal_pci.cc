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

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdarg.h>
#include <errno.h>
#include <limits.h>

#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_pci.h>
#include <rte_common.h>
#include <rte_launch.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_eal.h>
#include <rte_eal_memconfig.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_malloc.h>
#include <rte_string_fns.h>
#include <rte_debug.h>
#include <rte_devargs.h>

#include "eal_filesystem.h"
#include "eal_private.h"
#include "eal_interrupts_osv.hh"

#include <drivers/device.hh>
#include <drivers/pci-device.hh>
#include <osv/pci.hh>

/**
 * @file
 * PCI probing under OSv
 *
 * This code probes the PCI bus via native OSv methods.
 */

struct uio_map {
        void *addr;
        uint64_t offset;
        uint64_t size;
        uint64_t phaddr;
};

/*
 * For multi-process we need to reproduce all PCI mappings in secondary
 * processes, so save them in a tailq.
 */
struct uio_resource {
        TAILQ_ENTRY(uio_resource) next;

        struct rte_pci_addr pci_addr;
        char path[PATH_MAX];
        size_t nb_maps;
        struct uio_map maps[PCI_MAX_RESOURCE];
};

TAILQ_HEAD(uio_res_list, uio_resource);

static struct uio_res_list *uio_res_list = NULL;

static struct rte_tailq_elem rte_pci_tailq = {
        NULL,
        { NULL, NULL, },
        "PCI_RESOURCE_LIST",
};
EAL_REGISTER_TAILQ(rte_pci_tailq)

/* unbind kernel driver for this device */
int
pci_unbind_kernel_driver(struct rte_pci_device *dev __rte_unused)
{
        RTE_LOG(ERR, EAL, "RTE_PCI_DRV_FORCE_UNBIND flag is not implemented "
                "for OSv\n");
        return -ENOTSUP;
}

int
rte_eal_pci_map_device(struct rte_pci_device *dev)
{
        RTE_LOG(DEBUG, EAL,
                "	 Not managed by a supported kernel driver, skipped\n");
        return 0;
}

void
rte_eal_pci_unmap_device(struct rte_pci_device *dev)
{
        RTE_LOG(DEBUG, EAL,
                "	 Not managed by a supported kernel driver, skipped\n");
        return;
}

/* Scan one pci entry, and fill the devices list from it. */
static int
pci_scan_one(hw::hw_device* dev)
{
        u8 bus, device, func;
        auto pci_dev = static_cast<pci::device*>(dev);

        /* we only really care about network devices, I think... */
        if (pci_dev->get_base_class_code() != pci::function::PCI_CLASS_NETWORK)
                return 0;

        auto rte_dev = new rte_pci_device();

        /* get bus id, device id, function no */
        pci_dev->get_bdf(bus, device, func);
        rte_dev->addr.domain = 0;
        rte_dev->addr.bus = bus;
        rte_dev->addr.devid = device;
        rte_dev->addr.function = func;

        /* get vendor id */
        rte_dev->id.vendor_id = pci_dev->get_vendor_id();

        /* get device id */
        rte_dev->id.device_id = pci_dev->get_device_id();

        /* get subsystem_vendor id */
        rte_dev->id.subsystem_vendor_id = pci_dev->get_subsystem_vid();

        /* get subsystem_device id */
        rte_dev->id.subsystem_device_id = pci_dev->get_subsystem_id();

        /* get class id */
        rte_dev->id.class_id = ((pci_dev->get_base_class_code() << 16) |
                                (pci_dev->get_sub_class_code() << 8) |
                                (pci_dev->get_programming_interface()));

        /* TODO: get max_vfs */
        rte_dev->max_vfs = 0;

        /* OSv has no NUMA support (yet) */
        rte_dev->device.numa_node = 0;

        /* Configure interrupts */
        if (eal_interrupts_osv_setup(&rte_dev->intr_handle, pci_dev) != 0) {
                RTE_LOG(NOTICE, EAL, "   Interrupt setup failed\n");
        }

        /* Turn on bus mastering */
        pci_dev->set_bus_master(true);
        if (!pci_dev->get_bus_master()) {
                RTE_LOG(NOTICE, EAL, "   Bus master setup failed\n");
        }

        for (int i = 0; i < PCI_MAX_RESOURCE; i++) {
                auto bar = pci_dev->get_bar(i+1);
                if (bar == nullptr) {
                        continue;
                } else if (bar->is_mmio()) {
                        rte_dev->mem_resource[i].len = bar->get_size();
                        rte_dev->mem_resource[i].phys_addr = bar->get_addr64();
                        bar->map();
                        rte_dev->mem_resource[i].addr = const_cast<void *>(bar->get_mmio());
                } else {
                        rte_dev->mem_resource[i].len = bar->get_size();
                        rte_dev->mem_resource[i].phys_addr = 0;
                        rte_dev->mem_resource[i].addr = reinterpret_cast<void *>(bar->get_addr_lo());
                }
        }

        pci_dev->print();
        /* device is valid, add in list (sorted) */
        if (TAILQ_EMPTY(&pci_device_list)) {
                TAILQ_INSERT_TAIL(&pci_device_list, rte_dev, next);
        }
        else {
                struct rte_pci_device *dev2 = NULL;
                int ret;

                TAILQ_FOREACH(dev2, &pci_device_list, next) {
                        ret = rte_eal_compare_pci_addr(&rte_dev->addr, &dev2->addr);
                        if (ret > 0)
                                continue;
                        else if (ret < 0) {
                                TAILQ_INSERT_BEFORE(dev2, rte_dev, next);
                                return 1;
                        } else { /* already registered */
                                dev2->kdrv = rte_dev->kdrv;
                                dev2->max_vfs = rte_dev->max_vfs;
                                memmove(dev2->mem_resource,
                                        rte_dev->mem_resource,
                                        sizeof(rte_dev->mem_resource));
                                delete rte_dev;
                                return 0;
                        }
                }
                TAILQ_INSERT_TAIL(&pci_device_list, rte_dev, next);
        }

        return 1;
}

/*
 * Scan the content of the PCI bus, and add the devices in the devices
 * list. Call pci_scan_one() for each pci entry found.
 */
int
rte_eal_pci_scan(void)
{
        unsigned dev_count = 0;
        int err = 0;

        auto dm = hw::device_manager::instance();
        dm->for_each_device([&dev_count, &err] (hw::hw_device* dev) {
                                if (dev->is_attached())
                                        return;
                                int ret = pci_scan_one(dev);
                                if (ret < 0) {
                                        err++;
                                } else {
                                        dev_count += ret;
                                }
                        });

        if (err)
                return -1;

        RTE_LOG(ERR, EAL, "PCI scan found %u devices\n", dev_count);
        return 0;
}

int
pci_update_device(const struct rte_pci_addr *addr)
{
        (void)addr;
        return (0);
}

int
rte_eal_pci_read_config(const struct rte_pci_device *device,
                        void *buf, size_t len, off_t offset)
{
        uint8_t *data = nullptr;
        size_t last_read = 0, to_read = 0;
        size_t idx = 0;

        for (data = static_cast<uint8_t *>(buf), to_read = len;
             to_read > 0;
             to_read -= last_read, idx += last_read, data += last_read) {
                if (to_read >= sizeof(uint32_t)) {
                        *(reinterpret_cast<uint32_t *>(data)) =
                                pci::read_pci_config(device->addr.bus,
                                                device->addr.devid,
                                                device->addr.function,
                                                offset + idx);
                        last_read = sizeof(uint32_t);
                } else if (to_read >= sizeof(uint16_t)) {
                        *(reinterpret_cast<uint16_t *>(data)) =
                                pci::read_pci_config_word(device->addr.bus,
                                                        device->addr.devid,
                                                        device->addr.function,
                                                        offset + idx);
                        last_read = sizeof(uint16_t);
                } else {
                        *data = pci::read_pci_config_byte(device->addr.bus,
                                                        device->addr.devid,
                                                        device->addr.function,
                                                        offset + idx);
                        last_read = sizeof(uint8_t);
                }
        }

        return 0;
}

int
rte_eal_pci_write_config(const struct rte_pci_device *device,
                        const void *buf, size_t len, off_t offset)
{
        const uint8_t *data = nullptr;
        size_t last_write = 0, to_write = 0;
        size_t idx = 0;

        for (data = static_cast<const uint8_t *>(buf), to_write = len;
             to_write > 0;
             to_write -= last_write, idx += last_write, data += last_write) {
                if (to_write >= sizeof(uint32_t)) {
                        pci::write_pci_config(device->addr.bus,
                                        device->addr.devid,
                                        device->addr.function,
                                        offset + idx,
                                        *(reinterpret_cast<const uint32_t *>(data)));
                        last_write = sizeof(uint32_t);
                } else if (to_write >= sizeof(uint16_t)) {
                        pci::write_pci_config_word(device->addr.bus,
                                                device->addr.devid,
                                                device->addr.function,
                                                offset + idx,
                                                *(reinterpret_cast<const uint16_t *>(data)));
                        last_write = sizeof(uint16_t);
                } else {
                        pci::write_pci_config_byte(device->addr.bus,
                                                device->addr.devid,
                                                device->addr.function,
                                                offset + idx,
                                                *data);
                        last_write = sizeof(uint8_t);
                }
        }

        return 0;
}

int rte_eal_pci_ioport_map(struct rte_pci_device *dev, int bar,
                        struct rte_pci_ioport *p)
{
        int ret = 0;

        if (dev->mem_resource[bar].len != 0) {
                p->dev = dev;
                p->base = bar;  /* this is much more useful */
        } else {
                ret = -1;
        }

        return ret;
}

int rte_eal_pci_ioport_unmap(struct rte_pci_ioport *p __rte_unused)
{
        p->dev = nullptr;
        p->base = 0;
        return 0;
}

void rte_eal_pci_ioport_read(struct rte_pci_ioport *p,
                        void *data, size_t len, off_t offset)
{
        if (p->dev == nullptr) {
                return;
        }

        struct rte_intr_handle *intr_handle = &p->dev->intr_handle;
        pci::device *device = static_cast<pci::device *>(intr_handle->device);
        auto bar = device->get_bar(p->base + 1);
        assert(bar != nullptr);
        int size = 0;
        uint32_t reg = offset;

        for (uint8_t *d = static_cast<uint8_t *>(data);
             len > 0;
             d+= size, reg += size, len -= size) {
                if (len >= 8) {
                        size = 8;
                        *(uint64_t *)d = bar->readq(reg);
                } else if (len >= 4) {
                        size = 4;
                        *(uint32_t *)d = bar->readl(reg);
                } else if (len >= 2) {
                        size = 2;
                        *(uint16_t *)d = bar->readw(reg);
                } else {
                        size = 1;
                        *d = bar->readb(reg);
                }
        }
}

void rte_eal_pci_ioport_write(struct rte_pci_ioport *p,
                        const void *data, size_t len, off_t offset)
{
        if (p->dev == nullptr) {
                return;
        }

        struct rte_intr_handle *intr_handle = &p->dev->intr_handle;
        pci::device *device = static_cast<pci::device *>(intr_handle->device);
        auto bar = device->get_bar(p->base + 1);
        assert(bar != nullptr);
        int size = 0;
        uint32_t reg = offset;

        for (const uint8_t *s = static_cast<const uint8_t *>(data);
             len > 0;
             s+= size, reg += size, len -= size) {
                if (len >= 8) {
                        size = 8;
                        bar->writeq(reg, *(reinterpret_cast<const uint64_t *>(s)));
                } else if (len >= 4) {
                        size = 4;
                        bar->writel(reg, *(reinterpret_cast<const uint32_t *>(s)));
                } else if (len >= 2) {
                        size = 2;
                        bar->writew(reg, *(reinterpret_cast<const uint16_t *>(s)));
                } else {
                        size = 1;
                        bar->writeb(reg, *s);
                }
        }
}

/* Init the PCI EAL subsystem */
int
rte_eal_pci_init(void)
{
        /* for debug purposes, PCI can be disabled */
        if (internal_config.no_pci)
                return 0;

        if (rte_eal_pci_scan() < 0) {
                RTE_LOG(ERR, EAL, "%s(): Cannot scan PCI bus\n", __func__);
                return -1;
        }
        return 0;
}
