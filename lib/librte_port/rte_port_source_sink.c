/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
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
#include <string.h>

#include <rte_mbuf.h>
#include <rte_mempool.h>
#include <rte_malloc.h>

#ifdef RTE_NEXT_ABI

#include <rte_memcpy.h>

#ifdef RTE_PORT_PCAP
#include <rte_ether.h>
#include <pcap.h>
#endif

#else
#undef RTE_PORT_PCAP
#endif

#include "rte_port_source_sink.h"

/*
 * Port SOURCE
 */
#ifdef RTE_PORT_STATS_COLLECT

#define RTE_PORT_SOURCE_STATS_PKTS_IN_ADD(port, val) \
	port->stats.n_pkts_in += val
#define RTE_PORT_SOURCE_STATS_PKTS_DROP_ADD(port, val) \
	port->stats.n_pkts_drop += val

#else

#define RTE_PORT_SOURCE_STATS_PKTS_IN_ADD(port, val)
#define RTE_PORT_SOURCE_STATS_PKTS_DROP_ADD(port, val)

#endif

struct rte_port_source {
	struct rte_port_in_stats stats;

	struct rte_mempool *mempool;

#ifdef RTE_NEXT_ABI
	/* PCAP buffers and indexes */
	uint8_t **pkts;
	uint8_t *pkt_buff;
	uint32_t *pkt_len;
	uint32_t n_pkts;
	uint32_t pkt_index;
#endif
};

#ifdef RTE_NEXT_ABI

#ifdef RTE_PORT_PCAP

/**
 * Load PCAP file, allocate and copy packets in the file to memory
 *
 * @param p
 *   Parameters for source port
 * @param port
 *   Handle to source port
 * @param socket_id
 *   Socket id where the memory is created
 * @return
 *   0 on SUCCESS
 *   error code otherwise
 */
static int
pcap_source_load(struct rte_port_source_params *p,
		struct rte_port_source *port,
		int socket_id)
{
	uint32_t status = 0;
	uint32_t n_pkts = 0;
	uint32_t i;
	uint32_t *pkt_len_aligns = NULL;
	size_t total_buff_len = 0;
	pcap_t *pcap_handle;
	char pcap_errbuf[PCAP_ERRBUF_SIZE];
	uint32_t max_len;
	struct pcap_pkthdr pcap_hdr;
	const uint8_t *pkt;
	uint8_t *buff = NULL;
	uint32_t pktmbuf_maxlen = (uint32_t)
			(rte_pktmbuf_data_room_size(port->mempool) -
			RTE_PKTMBUF_HEADROOM);

	if (p->file_name == NULL)
		return 0;

	if (p->n_bytes_per_pkt == 0)
		max_len = pktmbuf_maxlen;
	else
		max_len = RTE_MIN(p->n_bytes_per_pkt, pktmbuf_maxlen);

	/* first time open, get packet number */
	pcap_handle = pcap_open_offline(p->file_name, pcap_errbuf);
	if (pcap_handle == NULL) {
		status = -ENOENT;
		goto error_exit;
	}

	while ((pkt = pcap_next(pcap_handle, &pcap_hdr)) != NULL)
		n_pkts++;

	pcap_close(pcap_handle);

	port->pkt_len = rte_zmalloc_socket("PCAP",
		(sizeof(*port->pkt_len) * n_pkts), 0, socket_id);
	if (port->pkt_len == NULL) {
		status = -ENOMEM;
		goto error_exit;
	}

	pkt_len_aligns = rte_malloc("PCAP",
		(sizeof(*pkt_len_aligns) * n_pkts), 0);
	if (pkt_len_aligns == NULL) {
		status = -ENOMEM;
		goto error_exit;
	}

	port->pkts = rte_zmalloc_socket("PCAP",
		(sizeof(*port->pkts) * n_pkts), 0, socket_id);
	if (port->pkts == NULL) {
		status = -ENOMEM;
		goto error_exit;
	}

	/* open 2nd time, get pkt_len */
	pcap_handle = pcap_open_offline(p->file_name, pcap_errbuf);
	if (pcap_handle == NULL) {
		status = -ENOENT;
		goto error_exit;
	}

	for (i = 0; i < n_pkts; i++) {
		pkt = pcap_next(pcap_handle, &pcap_hdr);
		port->pkt_len[i] = RTE_MIN(max_len, pcap_hdr.len);
		pkt_len_aligns[i] = RTE_CACHE_LINE_ROUNDUP(
			port->pkt_len[i]);
		total_buff_len += pkt_len_aligns[i];
	}

	pcap_close(pcap_handle);

	/* allocate a big trunk of data for pcap file load */
	buff = rte_zmalloc_socket("PCAP",
		total_buff_len, 0, socket_id);
	if (buff == NULL) {
		status = -ENOMEM;
		goto error_exit;
	}

	port->pkt_buff = buff;

	/* open file one last time to copy the pkt content */
	pcap_handle = pcap_open_offline(p->file_name, pcap_errbuf);
	if (pcap_handle == NULL) {
		status = -ENOENT;
		goto error_exit;
	}

	for (i = 0; i < n_pkts; i++) {
		pkt = pcap_next(pcap_handle, &pcap_hdr);
		rte_memcpy(buff, pkt, port->pkt_len[i]);
		port->pkts[i] = buff;
		buff += pkt_len_aligns[i];
	}

	pcap_close(pcap_handle);

	port->n_pkts = n_pkts;

	rte_free(pkt_len_aligns);

	return 0;

error_exit:
	if (pkt_len_aligns)
		rte_free(pkt_len_aligns);
	if (port->pkt_len)
		rte_free(port->pkt_len);
	if (port->pkts)
		rte_free(port->pkts);
	if (port->pkt_buff)
		rte_free(port->pkt_buff);

	return status;
}

#else
static int
pcap_source_load(__rte_unused struct rte_port_source_params *p,
		struct rte_port_source *port,
		__rte_unused int socket_id)
{
	port->pkt_buff = NULL;
	port->pkt_len = NULL;
	port->pkts = NULL;
	port->pkt_index = 0;

	return -ENOTSUP;
}
#endif /* RTE_PORT_PCAP */

#endif

static void *
rte_port_source_create(void *params, int socket_id)
{
	struct rte_port_source_params *p =
			(struct rte_port_source_params *) params;
	struct rte_port_source *port;

	/* Check input arguments*/
	if ((p == NULL) || (p->mempool == NULL)) {
		RTE_LOG(ERR, PORT, "%s: Invalid params\n", __func__);
		return NULL;
	}

	/* Memory allocation */
	port = rte_zmalloc_socket("PORT", sizeof(*port),
			RTE_CACHE_LINE_SIZE, socket_id);
	if (port == NULL) {
		RTE_LOG(ERR, PORT, "%s: Failed to allocate port\n", __func__);
		return NULL;
	}

	/* Initialization */
	port->mempool = (struct rte_mempool *) p->mempool;

#ifdef RTE_NEXT_ABI

	/* pcap file load and initialization */
	int status = pcap_source_load(p, port, socket_id);

	if (status == 0) {
		if (port->pkt_buff != NULL) {
			RTE_LOG(INFO, PORT, "Successfully load pcap file "
				"'%s' with %u pkts\n",
				p->file_name, port->n_pkts);
		}
	} else if (status != -ENOTSUP) {
		/* ENOTSUP is not treated as error */
		switch (status) {
		case -ENOENT:
			RTE_LOG(ERR, PORT, "%s: Failed to open pcap file "
				"'%s' for reading\n",
				__func__, p->file_name);
			break;
		case -ENOMEM:
			RTE_LOG(ERR, PORT, "%s: Not enough memory\n",
				__func__);
			break;
		default:
			RTE_LOG(ERR, PORT, "%s: Failed to enable PCAP "
				"support for unknown reason\n",
				__func__);
			break;
		}

		rte_free(port);
		port = NULL;
	}

#endif

	return port;
}

static int
rte_port_source_free(void *port)
{
	struct rte_port_source *p =
			(struct rte_port_source *)port;

	/* Check input parameters */
	if (p == NULL)
		return 0;

#ifdef RTE_NEXT_ABI

	if (p->pkt_len)
		rte_free(p->pkt_len);
	if (p->pkts)
		rte_free(p->pkts);
	if (p->pkt_buff)
		rte_free(p->pkt_buff);
#endif

	rte_free(p);

	return 0;
}

static int
rte_port_source_rx(void *port, struct rte_mbuf **pkts, uint32_t n_pkts)
{
	struct rte_port_source *p = (struct rte_port_source *) port;
	uint32_t i;

	if (rte_mempool_get_bulk(p->mempool, (void **) pkts, n_pkts) != 0)
		return 0;

	for (i = 0; i < n_pkts; i++) {
		rte_mbuf_refcnt_set(pkts[i], 1);
		rte_pktmbuf_reset(pkts[i]);
	}

#ifdef RTE_NEXT_ABI

	if (p->pkt_buff != NULL) {
		for (i = 0; i < n_pkts; i++) {
			uint8_t *pkt_data = rte_pktmbuf_mtod(pkts[i],
				uint8_t *);

			rte_memcpy(pkt_data, p->pkts[p->pkt_index],
					p->pkt_len[p->pkt_index]);
			pkts[i]->data_len = p->pkt_len[p->pkt_index];
			pkts[i]->pkt_len = pkts[i]->data_len;

			p->pkt_index++;
			if (p->pkt_index >= p->n_pkts)
				p->pkt_index = 0;
		}
	}

#endif

	RTE_PORT_SOURCE_STATS_PKTS_IN_ADD(p, n_pkts);

	return n_pkts;
}

static int
rte_port_source_stats_read(void *port,
		struct rte_port_in_stats *stats, int clear)
{
	struct rte_port_source *p =
		(struct rte_port_source *) port;

	if (stats != NULL)
		memcpy(stats, &p->stats, sizeof(p->stats));

	if (clear)
		memset(&p->stats, 0, sizeof(p->stats));

	return 0;
}

/*
 * Port SINK
 */
#ifdef RTE_PORT_STATS_COLLECT

#define RTE_PORT_SINK_STATS_PKTS_IN_ADD(port, val) \
	(port->stats.n_pkts_in += val)
#define RTE_PORT_SINK_STATS_PKTS_DROP_ADD(port, val) \
	(port->stats.n_pkts_drop += val)

#else

#define RTE_PORT_SINK_STATS_PKTS_IN_ADD(port, val)
#define RTE_PORT_SINK_STATS_PKTS_DROP_ADD(port, val)

#endif

struct rte_port_sink {
	struct rte_port_out_stats stats;

	/* PCAP dumper handle and pkts number */
	void *dumper;
	uint32_t max_pkts;
	uint32_t pkt_index;
	uint32_t dump_finish;
};

#ifdef RTE_PORT_PCAP

/**
 * Open PCAP file for dumping packets to the file later
 *
 * @param port
 *   Handle to sink port
 * @param p
 *   Sink port parameter
 * @return
 *   0 on SUCCESS
 *   error code otherwise
 */
static int
pcap_sink_open(struct rte_port_sink *port,
		__rte_unused struct rte_port_sink_params *p)
{
	pcap_t *tx_pcap;
	pcap_dumper_t *pcap_dumper;

	if (p->file_name == NULL) {
		port->dumper = NULL;
		port->max_pkts = 0;
		port->pkt_index = 0;
		port->dump_finish = 0;
		return 0;
	}

	/** Open a dead pcap handler for opening dumper file */
	tx_pcap = pcap_open_dead(DLT_EN10MB, 65535);
	if (tx_pcap == NULL)
		return -ENOENT;

	/* The dumper is created using the previous pcap_t reference */
	pcap_dumper = pcap_dump_open(tx_pcap, p->file_name);
	if (pcap_dumper == NULL)
		return -ENOENT;

	port->dumper = pcap_dumper;
	port->max_pkts = p->max_n_pkts;
	port->pkt_index = 0;
	port->dump_finish = 0;

	return 0;
}

uint8_t jumbo_pkt_buf[ETHER_MAX_JUMBO_FRAME_LEN];

/**
 * Dump a packet to PCAP dumper
 *
 * @param p
 *   Handle to sink port
 * @param mbuf
 *   Handle to mbuf structure holding the packet
 */
static void
pcap_sink_dump_pkt(struct rte_port_sink *port, struct rte_mbuf *mbuf)
{
	uint8_t *pcap_dumper = (uint8_t *)(port->dumper);
	struct pcap_pkthdr pcap_hdr;
	uint8_t *pkt;

	/* Maximum num packets already reached */
	if (port->dump_finish)
		return;

	pkt = rte_pktmbuf_mtod(mbuf, uint8_t *);

	pcap_hdr.len = mbuf->pkt_len;
	pcap_hdr.caplen = pcap_hdr.len;
	gettimeofday(&(pcap_hdr.ts), NULL);

	if (mbuf->nb_segs > 1) {
		struct rte_mbuf *jumbo_mbuf;
		uint32_t pkt_index = 0;

		/* if packet size longer than ETHER_MAX_JUMBO_FRAME_LEN,
		 * ignore it.
		 */
		if (mbuf->pkt_len > ETHER_MAX_JUMBO_FRAME_LEN)
			return;

		for (jumbo_mbuf = mbuf; jumbo_mbuf != NULL;
				jumbo_mbuf = jumbo_mbuf->next) {
			rte_memcpy(&jumbo_pkt_buf[pkt_index],
				rte_pktmbuf_mtod(jumbo_mbuf, uint8_t *),
				jumbo_mbuf->data_len);
			pkt_index += jumbo_mbuf->data_len;
		}

		jumbo_pkt_buf[pkt_index] = '\0';

		pkt = jumbo_pkt_buf;
	}

	pcap_dump(pcap_dumper, &pcap_hdr, pkt);

	port->pkt_index++;

	if ((port->max_pkts != 0) && (port->pkt_index >= port->max_pkts)) {
		port->dump_finish = 1;
		RTE_LOG(INFO, PORT, "Dumped %u packets to file\n",
				port->pkt_index);
	}

}

/**
 * Flush pcap dumper
 *
 * @param dumper
 *   Handle to pcap dumper
 */

static void
pcap_sink_flush_pkt(void *dumper)
{
	pcap_dumper_t *pcap_dumper = (pcap_dumper_t *)dumper;

	pcap_dump_flush(pcap_dumper);
}

/**
 * Close a PCAP dumper handle
 *
 * @param dumper
 *   Handle to pcap dumper
 */
static void
pcap_sink_close(void *dumper)
{
	pcap_dumper_t *pcap_dumper = (pcap_dumper_t *)dumper;

	pcap_dump_close(pcap_dumper);
}

#else

static int
pcap_sink_open(struct rte_port_sink *port,
		__rte_unused struct rte_port_sink_params *p)
{
	port->dumper = NULL;
	port->max_pkts = 0;
	port->pkt_index = 0;
	port->dump_finish = 0;

	return -ENOTSUP;
}

static void
pcap_sink_dump_pkt(__rte_unused struct rte_port_sink *port,
		__rte_unused struct rte_mbuf *mbuf) {}

static void
pcap_sink_flush_pkt(__rte_unused void *dumper) {}

static void
pcap_sink_close(__rte_unused void *dumper) {}

#endif

static void *
rte_port_sink_create(__rte_unused void *params, int socket_id)
{
	struct rte_port_sink *port;
	struct rte_port_sink_params *p = params;
	int status;

	/* Memory allocation */
	port = rte_zmalloc_socket("PORT", sizeof(*port),
			RTE_CACHE_LINE_SIZE, socket_id);
	if (port == NULL) {
		RTE_LOG(ERR, PORT, "%s: Failed to allocate port\n", __func__);
		return NULL;
	}

	/* Try to open PCAP file for dumping, if possible */
	status = pcap_sink_open(port, p);
	if (status == 0) {
		if (port->dumper != NULL)
			RTE_LOG(INFO, PORT, "Ready to dump packets "
				"to file %s\n", p->file_name);

	} else if (status != -ENOTSUP) {
		if (status == -ENOENT)
			RTE_LOG(ERR, PORT, "%s: Failed to open pcap file "
				"%s for writing\n", __func__,
				p->file_name);
		else
			RTE_LOG(ERR, PORT, "%s: Failed to enable pcap "
				"support for unknown reason\n", __func__);

		rte_free(port);
		port = NULL;
	}

	return port;
}

static int
rte_port_sink_tx(void *port, struct rte_mbuf *pkt)
{
	__rte_unused struct rte_port_sink *p = (struct rte_port_sink *) port;

	RTE_PORT_SINK_STATS_PKTS_IN_ADD(p, 1);
	if (p->dumper != NULL)
		pcap_sink_dump_pkt(p, pkt);
	rte_pktmbuf_free(pkt);
	RTE_PORT_SINK_STATS_PKTS_DROP_ADD(p, 1);

	return 0;
}

static int
rte_port_sink_tx_bulk(void *port, struct rte_mbuf **pkts,
	uint64_t pkts_mask)
{
	__rte_unused struct rte_port_sink *p = (struct rte_port_sink *) port;

	if ((pkts_mask & (pkts_mask + 1)) == 0) {
		uint64_t n_pkts = __builtin_popcountll(pkts_mask);
		uint32_t i;

		RTE_PORT_SINK_STATS_PKTS_IN_ADD(p, n_pkts);
		RTE_PORT_SINK_STATS_PKTS_DROP_ADD(p, n_pkts);

		if (p->dumper) {
			for (i = 0; i < n_pkts; i++) {
				struct rte_mbuf *pkt = pkts[i];

				pcap_sink_dump_pkt(p, pkt);
			}
		}

		for (i = 0; i < n_pkts; i++) {
			struct rte_mbuf *pkt = pkts[i];

			rte_pktmbuf_free(pkt);
		}

	} else {
		if (p->dumper) {
			uint64_t dump_pkts_mask = pkts_mask;
			uint32_t pkt_index;

			for ( ; dump_pkts_mask; ) {
				pkt_index = __builtin_ctzll(
					dump_pkts_mask);
				pcap_sink_dump_pkt(p, pkts[pkt_index]);
				dump_pkts_mask &= ~(1LLU << pkt_index);
			}
		}

		for ( ; pkts_mask; ) {
			uint32_t pkt_index = __builtin_ctzll(pkts_mask);
			uint64_t pkt_mask = 1LLU << pkt_index;
			struct rte_mbuf *pkt = pkts[pkt_index];

			RTE_PORT_SINK_STATS_PKTS_IN_ADD(p, 1);
			RTE_PORT_SINK_STATS_PKTS_DROP_ADD(p, 1);
			rte_pktmbuf_free(pkt);
			pkts_mask &= ~pkt_mask;
		}
	}

	return 0;
}

static int
rte_port_sink_flush(void *port)
{
	struct rte_port_sink *p = (struct rte_port_sink *)port;

	if (p->dumper != NULL)
		pcap_sink_flush_pkt(p->dumper);

	return 0;
}

static int
rte_port_sink_free(void *port)
{
	struct rte_port_sink *p =
			(struct rte_port_sink *)port;
	/* Check input parameters */
	if (p == NULL)
		return 0;

	if (p->dumper != NULL)
		pcap_sink_close(p->dumper);

	rte_free(p);

	return 0;
}

static int
rte_port_sink_stats_read(void *port, struct rte_port_out_stats *stats,
		int clear)
{
	struct rte_port_sink *p =
		(struct rte_port_sink *) port;

	if (stats != NULL)
		memcpy(stats, &p->stats, sizeof(p->stats));

	if (clear)
		memset(&p->stats, 0, sizeof(p->stats));

	return 0;
}

/*
 * Summary of port operations
 */
struct rte_port_in_ops rte_port_source_ops = {
	.f_create = rte_port_source_create,
	.f_free = rte_port_source_free,
	.f_rx = rte_port_source_rx,
	.f_stats = rte_port_source_stats_read,
};

struct rte_port_out_ops rte_port_sink_ops = {
	.f_create = rte_port_sink_create,
	.f_free = rte_port_sink_free,
	.f_tx = rte_port_sink_tx,
	.f_tx_bulk = rte_port_sink_tx_bulk,
	.f_flush = rte_port_sink_flush,
	.f_stats = rte_port_sink_stats_read,
};
