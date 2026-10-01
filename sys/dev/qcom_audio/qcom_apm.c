/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 James Kane
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
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

/*
 * The Qualcomm audio DSP's APM (audio processing manager), which runs
 * AudioReach graphs: subgraphs of containers of modules, connected port to
 * port, as the board's topology file describes them.  A stream's graph
 * (a shared-memory endpoint, decoding, conversion, volume) and a device's
 * (a codec DMA or other hardware endpoint) are opened separately, the
 * device's carrying the link from the stream's last module that a mixer
 * switch in the topology names; their modules are given their media
 * formats, and they are prepared and started.  The stream's samples are
 * then written in buffers of shared memory, which the DSP reads through
 * the "apps" SMMU, and hands back.
 *
 * The topology is the one Linux loads, qcom/<soc>/<card>-tplg.bin, from
 * firmware(9), and the board is found by its SMBIOS maker and product.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/condvar.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <dev/qcom_audio/qcom_apm.h>
#include <dev/qcom_smmu/qcom_apps_smmu.h>
#include <dev/qcom_audio/qcom_gpr.h>
#include <dev/qcom_audio/qcom_wcd938x.h>

static MALLOC_DEFINE(M_APM, "qcom_apm", "Qualcomm APM");

/* APM commands, to the APM's port */
#define	APM_CMD_GRAPH_OPEN			0x01001000
#define	APM_CMD_GRAPH_PREPARE			0x01001001
#define	APM_CMD_GRAPH_START			0x01001002
#define	APM_CMD_GRAPH_STOP			0x01001003
#define	APM_CMD_GRAPH_CLOSE			0x01001004
#define	APM_CMD_SET_CFG				0x01001006
#define	APM_CMD_SHARED_MEM_MAP_REGIONS		0x0100100c
#define	APM_CMD_SHARED_MEM_UNMAP_REGIONS	0x0100100d
#define	APM_CMD_RSP_SHARED_MEM_MAP_REGIONS	0x02001001
#define	APM_MODULE_INSTANCE_ID			0x00000001
#define	APM_MEMORY_MAP_SHMEM8_4K_POOL		3

/* Graph description */
#define	APM_PARAM_ID_CONTAINER_CONFIG		0x08001000
#define	APM_PARAM_ID_SUB_GRAPH_CONFIG		0x08001001
#define	APM_PARAM_ID_MODULE_LIST		0x08001002
#define	APM_PARAM_ID_MODULE_PROP		0x08001003
#define	APM_PARAM_ID_MODULE_CONN		0x08001004
#define	APM_PARAM_ID_SUB_GRAPH_LIST		0x08001005
#define	APM_SUB_GRAPH_PROP_ID_PERF_MODE		0x0800100e
#define	APM_SUB_GRAPH_PROP_ID_DIRECTION		0x0800100f
#define	APM_SUB_GRAPH_PROP_ID_SCENARIO_ID	0x08001010
#define	APM_CONTAINER_PROP_ID_CAPABILITY_LIST	0x08001011
#define	APM_CONTAINER_PROP_ID_GRAPH_POS		0x08001012
#define	APM_CONTAINER_PROP_ID_STACK_SIZE	0x08001013
#define	APM_CONTAINER_PROP_ID_PROC_DOMAIN	0x08001014
#define	APM_MODULE_PROP_ID_PORT_INFO		0x08001015

/* Modules and their parameters */
#define	MODULE_ID_WR_SHARED_MEM_EP		0x07001000
#define	MODULE_ID_RD_SHARED_MEM_EP		0x07001001
#define	MODULE_ID_PCM_CNV			0x07001003
#define	MODULE_ID_PCM_ENC			0x07001004
#define	MODULE_ID_PCM_DEC			0x07001005
#define	MODULE_ID_MFC				0x07001015
#define	MODULE_ID_DATA_LOGGING			0x0700101a
#define	MODULE_ID_VOL_CTRL			0x0700101b
#define	MODULE_ID_CODEC_DMA_SINK		0x07001023
#define	MODULE_ID_CODEC_DMA_SOURCE		0x07001024
#define	PARAM_ID_PCM_OUTPUT_FORMAT_CFG		0x08001008
#define	PARAM_ID_MEDIA_FORMAT			0x0800100c
#define	PARAM_ID_HW_EP_MF_CFG			0x08001017
#define	PARAM_ID_HW_EP_FRAME_SIZE_FACTOR	0x08001018
#define	PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT	0x08001024
#define	PARAM_ID_MODULE_ENABLE			0x08001026
#define	PARAM_ID_DATA_LOGGING_CONFIG		0x08001031
#define	PARAM_ID_VOL_CTRL_MASTER_GAIN		0x08001035
#define	PARAM_ID_CODEC_DMA_INTF_CFG		0x08001063
#define	PARAM_ID_HW_EP_POWER_MODE_CFG		0x08001176
#define	VOL_CTRL_UNITY				0x2000	/* Q13 */
#define	DATA_FORMAT_FIXED_POINT			1
#define	MEDIA_FMT_ID_PCM			0x09001000
#define	PCM_LSB_ALIGNED				1
#define	PCM_LITTLE_ENDIAN			1
#define	PCM_CHANNEL_FL				1
#define	PCM_CHANNEL_FR				2

/* Data, to and from the shared-memory endpoint's port */
#define	DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2	0x0400100a
#define	DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2 0x05001004
#define	WR_SH_MEM_NO_TIMESTAMP	0xff00
/* A write's token: ours in the low half, the length in the high. */
#define	APM_WRITE_TOKEN_MASK		0xffff
#define	APM_WRITE_TOKEN_LEN_SHIFT	16

/* Ports of ours for each graph's packets */
#define	APM_GRAPH_PORT_BASE			0x10000000

/* Topology (ALSA SoC topology, with AudioReach tuples) */
#define	TPLG_MAGIC				0x41536f43
#define	TPLG_TYPE_DAPM_GRAPH			4
#define	TPLG_TYPE_DAPM_WIDGET			5
#define	TPLG_DAPM_MIXER				3
#define	TPLG_NAME				44
#define	TPLG_TUPLE_UUID				0
#define	TPLG_TUPLE_STRING			1
#define	AR_TKN_DAI_INDEX			1
#define	AR_TKN_SUB_GRAPH_INSTANCE_ID		2
#define	AR_TKN_SUB_GRAPH_PERF_MODE		3
#define	AR_TKN_SUB_GRAPH_DIRECTION		4
#define	AR_TKN_SUB_GRAPH_SCENARIO_ID		5
#define	AR_TKN_CONTAINER_INSTANCE_ID		100
#define	AR_TKN_CONTAINER_CAPABILITY_ID		101
#define	AR_TKN_CONTAINER_STACK_SIZE		102
#define	AR_TKN_CONTAINER_GRAPH_POS		103
#define	AR_TKN_CONTAINER_PROC_DOMAIN		104
#define	AR_TKN_MODULE_ID			200
#define	AR_TKN_MODULE_INSTANCE_ID		201
#define	AR_TKN_MODULE_MAX_IP_PORTS		202
#define	AR_TKN_MODULE_MAX_OP_PORTS		203
#define	AR_TKN_MODULE_SRC_OP_PORT_ID		206
#define	AR_TKN_MODULE_DST_IN_PORT_ID		207
#define	AR_TKN_MODULE_DST_INSTANCE_ID		209
#define	AR_TKN_MODULE_HW_IF_IDX			250
#define	AR_TKN_MODULE_HW_IF_TYPE		251
#define	AR_TKN_MODULE_FMT_INTERLEAVE		252
#define	AR_TKN_MODULE_FMT_DATA			253
#define	AR_TKN_MODULE_LOG_CODE			259
#define	AR_TKN_MODULE_LOG_TAP_POINT_ID		260
#define	AR_TKN_MODULE_LOG_MODE			261

#define	APM_MAX_MODULES		96
#define	APM_MAX_CONTAINERS	32
#define	APM_MAX_SGS		32
#define	APM_MAX_MIXERS		32

/* The DSP's DMA into our buffers comes through this stream. */
#define	APM_LPASS_STREAM	0xc01
/*
 * The DSP takes a buffer's address with the low bits of its stream ID above
 * bit 32, which tell it which stream to issue the access on.
 */
#define	APM_DSP_ADDR(iova)	((iova) | (uint64_t)(APM_LPASS_STREAM & 0xf) << 32)

struct apm_board {
	const char	*maker;		/* SMBIOS system maker and product */
	const char	*product;
	const char	*topology;
};

static const struct apm_board apm_boards[] = {
	{ "Radxa Computer Co., Ltd.", "Radxa Dragon Q8B",
	  "qcom/sc8280xp/radxa/dragon-q8b/SC8280XP-Radxa-Dragon-Q8B-tplg.bin" },
};

struct apm_sg {
	uint32_t	id;
	uint32_t	graph;		/* the DAI index, which names the graph */
	uint32_t	perf_mode;
	uint32_t	direction;
	uint32_t	scenario;
};

struct apm_container {
	uint32_t	id;
	u_int		sg;
	uint32_t	capability;
	uint32_t	stack_size;
	uint32_t	graph_pos;
	uint32_t	proc_domain;
};

struct apm_module {
	char		name[TPLG_NAME];
	char		stream[TPLG_NAME];	/* an endpoint's stream name */
	u_int		container;
	uint32_t	iid;
	uint32_t	mid;
	uint32_t	max_ip_ports;
	uint32_t	max_op_ports;
	uint32_t	src_op_port;
	uint32_t	dst_ip_port;
	uint32_t	dst_iid;
	uint32_t	hw_if_type;
	uint32_t	hw_if_idx;
	uint32_t	interleave;
	uint32_t	data_format;
	uint32_t	log_code;
	uint32_t	log_tap;
	uint32_t	log_mode;
};

/* A switch in a mixer widget: links one graph's last module to another's first. */
struct apm_mixer {
	char		widget[TPLG_NAME];
	char		control[TPLG_NAME];
	uint32_t	src_graph;
	uint32_t	src_iid;
	uint32_t	dst_graph;
	uint32_t	dst_iid;
};

struct apm_topology {
	struct apm_sg	sgs[APM_MAX_SGS];
	u_int		nsgs;
	struct apm_container conts[APM_MAX_CONTAINERS];
	u_int		nconts;
	struct apm_module mods[APM_MAX_MODULES];
	u_int		nmods;
	struct apm_mixer mixers[APM_MAX_MIXERS];
	u_int		nmixers;
};

struct apm_graph {
	uint32_t	id;
	struct qcom_gpr_port *port;	/* its modules answer here */
	uint32_t	link_src;	/* a link into it, from a mixer */
	uint32_t	link_dst;
	uint32_t	sg_ids[APM_MAX_SGS];
	u_int		nsg;
};

struct apm_pcm_cfg {
	uint32_t	rate;
	uint16_t	bits;
	uint16_t	channels;
	uint8_t		chmap[8];
};

static struct {
	struct sx		lock;
	struct apm_topology	*tp;
	struct qcom_gpr_port	*port;	/* the APM's */
	struct qcom_apps_smmu_dom *dom;	/* the DSP's way into our memory */
	bool			busy;	/* playing */
} apm;

SX_SYSINIT(qcom_apm, &apm.lock, "qcom_apm");

/* Topology */

static const struct apm_board *
apm_find_board(void)
{
	const struct apm_board *b;
	char *maker, *product;
	u_int i;

	maker = kern_getenv("smbios.system.maker");
	product = kern_getenv("smbios.system.product");
	b = NULL;
	for (i = 0; maker != NULL && product != NULL && i < nitems(apm_boards);
	    i++)
		if (strcmp(maker, apm_boards[i].maker) == 0 &&
		    strcmp(product, apm_boards[i].product) == 0) {
			b = &apm_boards[i];
			break;
		}
	freeenv(maker);
	freeenv(product);
	return (b);
}

/* Whether this is a board whose audio we know. */
bool
qcom_apm_supported(void)
{

	return (apm_find_board() != NULL);
}

struct tplg_tuples {
	bool		has[300];
	uint32_t	val[300];
};

/* The value tuples in a private data block, by token. */
static int
tplg_tuples(const uint8_t *p, size_t len, struct tplg_tuples *t)
{
	uint32_t size, type, n, tok, i;
	size_t off, e, esz;

	memset(t, 0, sizeof(*t));
	for (off = 0; off + 12 <= len; off += size) {
		memcpy(&size, p + off, 4);
		memcpy(&type, p + off + 4, 4);
		memcpy(&n, p + off + 8, 4);
		if (size < 12 || off + size > len)
			return (EINVAL);
		esz = type == TPLG_TUPLE_UUID ? 20 :
		    type == TPLG_TUPLE_STRING ? 4 + TPLG_NAME : 8;
		for (i = 0, e = off + 12; i < n && e + esz <= off + size;
		    i++, e += esz) {
			if (esz != 8)
				continue;
			memcpy(&tok, p + e, 4);
			if (tok < nitems(t->has)) {
				t->has[tok] = true;
				memcpy(&t->val[tok], p + e + 4, 4);
			}
		}
	}
	return (0);
}

static void
tplg_name(char *dst, const uint8_t *src)
{

	memcpy(dst, src, TPLG_NAME);
	dst[TPLG_NAME - 1] = '\0';
}

static u_int
tplg_sg(struct apm_topology *tp, struct tplg_tuples *t)
{
	u_int i;

	for (i = 0; i < tp->nsgs; i++)
		if (tp->sgs[i].id == t->val[AR_TKN_SUB_GRAPH_INSTANCE_ID])
			return (i);
	if (tp->nsgs == APM_MAX_SGS)
		return (UINT_MAX);
	tp->sgs[i].id = t->val[AR_TKN_SUB_GRAPH_INSTANCE_ID];
	tp->sgs[i].graph = t->val[AR_TKN_DAI_INDEX];
	tp->sgs[i].perf_mode = t->val[AR_TKN_SUB_GRAPH_PERF_MODE];
	tp->sgs[i].direction = t->val[AR_TKN_SUB_GRAPH_DIRECTION];
	tp->sgs[i].scenario = t->val[AR_TKN_SUB_GRAPH_SCENARIO_ID];
	return (tp->nsgs++);
}

static u_int
tplg_container(struct apm_topology *tp, struct tplg_tuples *t, u_int sg)
{
	u_int i;

	for (i = 0; i < tp->nconts; i++)
		if (tp->conts[i].id == t->val[AR_TKN_CONTAINER_INSTANCE_ID])
			return (i);
	if (tp->nconts == APM_MAX_CONTAINERS)
		return (UINT_MAX);
	tp->conts[i].id = t->val[AR_TKN_CONTAINER_INSTANCE_ID];
	tp->conts[i].sg = sg;
	tp->conts[i].capability = t->val[AR_TKN_CONTAINER_CAPABILITY_ID];
	tp->conts[i].stack_size = t->val[AR_TKN_CONTAINER_STACK_SIZE];
	tp->conts[i].graph_pos = t->val[AR_TKN_CONTAINER_GRAPH_POS];
	tp->conts[i].proc_domain = t->val[AR_TKN_CONTAINER_PROC_DOMAIN];
	return (tp->nconts++);
}

static int
tplg_widget(struct apm_topology *tp, const uint8_t *w, size_t wlen,
    size_t *used)
{
	struct tplg_tuples *t;
	struct apm_module *m;
	struct apm_mixer *mx;
	uint32_t size, type, nk, psize, ksize, kpsize, i;
	char name[TPLG_NAME];
	const uint8_t *k;
	size_t off;
	u_int sg, c;
	int error;

	if (wlen < 132)
		return (EINVAL);
	memcpy(&size, w, 4);
	memcpy(&type, w + 4, 4);
	memcpy(&nk, w + 124, 4);
	memcpy(&psize, w + 128, 4);
	if (size < 132 || size + psize > wlen)
		return (EINVAL);
	t = malloc(sizeof(*t), M_APM, M_WAITOK);
	error = tplg_tuples(w + size, psize, t);
	if (error != 0)
		goto out;
	tplg_name(name, w + 8);

	if (t->has[AR_TKN_MODULE_INSTANCE_ID]) {
		sg = tplg_sg(tp, t);
		c = sg == UINT_MAX ? UINT_MAX : tplg_container(tp, t, sg);
		if (c == UINT_MAX || tp->nmods == APM_MAX_MODULES) {
			error = ENOSPC;
			goto out;
		}
		m = &tp->mods[tp->nmods++];
		strlcpy(m->name, name, sizeof(m->name));
		tplg_name(m->stream, w + 8 + TPLG_NAME);
		m->container = c;
		m->iid = t->val[AR_TKN_MODULE_INSTANCE_ID];
		m->mid = t->val[AR_TKN_MODULE_ID];
		m->max_ip_ports = t->val[AR_TKN_MODULE_MAX_IP_PORTS];
		m->max_op_ports = t->val[AR_TKN_MODULE_MAX_OP_PORTS];
		m->src_op_port = t->val[AR_TKN_MODULE_SRC_OP_PORT_ID];
		m->dst_ip_port = t->val[AR_TKN_MODULE_DST_IN_PORT_ID];
		m->dst_iid = t->val[AR_TKN_MODULE_DST_INSTANCE_ID];
		m->hw_if_type = t->val[AR_TKN_MODULE_HW_IF_TYPE];
		m->hw_if_idx = t->val[AR_TKN_MODULE_HW_IF_IDX];
		m->interleave = t->val[AR_TKN_MODULE_FMT_INTERLEAVE];
		m->data_format = t->val[AR_TKN_MODULE_FMT_DATA];
		m->log_code = t->val[AR_TKN_MODULE_LOG_CODE];
		m->log_tap = t->val[AR_TKN_MODULE_LOG_TAP_POINT_ID];
		m->log_mode = t->val[AR_TKN_MODULE_LOG_MODE];
	}

	/* The kcontrols follow; a mixer's are its switches. */
	off = size + psize;
	for (i = 0; i < nk; i++) {
		k = w + off;
		if (off + 208 > wlen) {
			error = EINVAL;
			goto out;
		}
		/* The control header is 204 bytes; then its own size. */
		memcpy(&ksize, k + 204, 4);
		if (ksize < 208 || off + ksize > wlen) {
			error = EINVAL;
			goto out;
		}
		memcpy(&kpsize, k + ksize - 4, 4);
		if (off + ksize + kpsize > wlen) {
			error = EINVAL;
			goto out;
		}
		if (type == TPLG_DAPM_MIXER && tp->nmixers < APM_MAX_MIXERS) {
			struct tplg_tuples *kt;

			kt = malloc(sizeof(*kt), M_APM, M_WAITOK);
			if (tplg_tuples(k + ksize, kpsize, kt) == 0 &&
			    kt->has[AR_TKN_DAI_INDEX]) {
				mx = &tp->mixers[tp->nmixers++];
				strlcpy(mx->widget, name, sizeof(mx->widget));
				tplg_name(mx->control, k + 8);
				mx->src_graph = kt->val[AR_TKN_DAI_INDEX];
				mx->dst_graph = t->val[AR_TKN_DAI_INDEX];
			}
			free(kt, M_APM);
		}
		off += ksize + kpsize;
	}
	*used = off;
out:
	free(t, M_APM);
	return (error);
}

static struct apm_module *
apm_module_by_name(struct apm_topology *tp, const char *name)
{
	u_int i;

	for (i = 0; i < tp->nmods; i++)
		if (strcmp(tp->mods[i].name, name) == 0)
			return (&tp->mods[i]);
	return (NULL);
}

/* A route ties a mixer's switch to the modules it links. */
static void
tplg_route(struct apm_topology *tp, const char *sink, const char *control,
    const char *source)
{
	struct apm_module *src, *dst;
	u_int i;

	src = apm_module_by_name(tp, source);
	dst = apm_module_by_name(tp, sink);
	for (i = 0; i < tp->nmixers; i++) {
		if (dst != NULL && src == NULL &&
		    strcmp(tp->mixers[i].widget, source) == 0)
			tp->mixers[i].dst_iid = dst->iid;
		else if (src != NULL && dst == NULL && control[0] != '\0' &&
		    strcmp(tp->mixers[i].widget, sink) == 0 &&
		    strcmp(tp->mixers[i].control, control) == 0)
			tp->mixers[i].src_iid = src->iid;
	}
}

static int
apm_topology_parse(const uint8_t *d, size_t len, struct apm_topology *tp)
{
	uint32_t magic, type, hsize, psize, count, i;
	char sink[TPLG_NAME], ctl[TPLG_NAME], src[TPLG_NAME];
	size_t off, p, used;
	int error;

	for (off = 0; off + 36 <= len; off += hsize + psize) {
		memcpy(&magic, d + off, 4);
		memcpy(&type, d + off + 12, 4);
		memcpy(&hsize, d + off + 16, 4);
		memcpy(&psize, d + off + 24, 4);
		memcpy(&count, d + off + 32, 4);
		if (magic != TPLG_MAGIC || hsize < 36 ||
		    off + hsize + psize > len)
			return (EINVAL);
		p = off + hsize;
		if (type == TPLG_TYPE_DAPM_WIDGET) {
			for (i = 0; i < count; i++) {
				error = tplg_widget(tp, d + p,
				    off + hsize + psize - p, &used);
				if (error != 0)
					return (error);
				p += used;
			}
		} else if (type == TPLG_TYPE_DAPM_GRAPH) {
			for (i = 0; i < count && p + 3 * TPLG_NAME <=
			    off + hsize + psize; i++, p += 3 * TPLG_NAME) {
				tplg_name(sink, d + p);
				tplg_name(ctl, d + p + TPLG_NAME);
				tplg_name(src, d + p + 2 * TPLG_NAME);
				tplg_route(tp, sink, ctl, src);
			}
		}
	}
	return (0);
}

static int
apm_load_topology(void)
{
	const struct apm_board *b;
	const struct firmware *fw;
	struct apm_topology *tp;
	int error;

	sx_assert(&apm.lock, SA_XLOCKED);
	if (apm.tp != NULL)
		return (0);
	b = apm_find_board();
	if (b == NULL)
		return (ENXIO);
	fw = firmware_get(b->topology);
	if (fw == NULL) {
		printf("qcom_apm: no topology %s\n", b->topology);
		return (ENOENT);
	}
	tp = malloc(sizeof(*tp), M_APM, M_WAITOK | M_ZERO);
	error = apm_topology_parse(fw->data, fw->datasize, tp);
	firmware_put(fw, FIRMWARE_UNLOAD);
	if (error != 0) {
		printf("qcom_apm: bad topology %s: %d\n", b->topology, error);
		free(tp, M_APM);
		return (error);
	}
	printf("qcom_apm: %s: %u subgraphs, %u containers, %u modules, "
	    "%u mixer switches\n", b->topology, tp->nsgs, tp->nconts,
	    tp->nmods, tp->nmixers);
	apm.tp = tp;
	return (0);
}

/* Packets */

struct apm_param {
	uint32_t	iid;
	uint32_t	param_id;
	uint32_t	param_size;
	uint32_t	error;
};

struct apm_buf {
	uint8_t		*p;
	size_t		len;
	size_t		size;
};

static void *
apm_put(struct apm_buf *b, size_t len)
{
	void *p;

	KASSERT(b->len + len <= b->size, ("apm packet overrun"));
	p = b->p + b->len;
	b->len += len;
	return (p);
}

static void
apm_put32(struct apm_buf *b, uint32_t v)
{

	memcpy(apm_put(b, 4), &v, 4);
}

/* A parameter's header; its size is fixed by apm_param_end(). */
static size_t
apm_param_begin(struct apm_buf *b, uint32_t iid, uint32_t param_id)
{
	struct apm_param *ph;
	size_t at;

	at = b->len;
	ph = apm_put(b, sizeof(*ph));
	ph->iid = iid;
	ph->param_id = param_id;
	ph->param_size = 0;
	ph->error = 0;
	return (at);
}

static void
apm_param_end(struct apm_buf *b, size_t at, size_t align)
{
	struct apm_param *ph;

	while ((b->len - at) % align != 0)
		*(uint8_t *)apm_put(b, 1) = 0;
	ph = (struct apm_param *)(b->p + at);
	ph->param_size = b->len - at - sizeof(*ph);
}

static void
apm_buf_init(struct apm_buf *b, size_t size)
{

	b->p = malloc(size, M_APM, M_WAITOK | M_ZERO);
	b->size = size;
	/* The APM command header: no out-of-band payload. */
	b->len = 16;
}

/* An APM command whose payload follows its command header. */
static int
apm_cmd(struct qcom_gpr_port *port, uint32_t dst, uint32_t opcode,
    struct apm_buf *b, uint32_t rsp_opcode, void *rsp, size_t *rsplen)
{
	uint32_t payload = b->len - 16;
	int error;

	memcpy(b->p + 12, &payload, 4);
	error = qcom_gpr_cmd(port, dst, opcode, b->p, b->len, rsp_opcode, rsp,
	    rsplen);
	free(b->p, M_APM);
	return (error);
}

static struct apm_container *
apm_cont(struct apm_module *m)
{

	return (&apm.tp->conts[m->container]);
}

static struct apm_sg *
apm_sg_of(struct apm_module *m)
{

	return (&apm.tp->sgs[apm_cont(m)->sg]);
}

static int
apm_graph_open_cmd(struct apm_graph *g)
{
	struct apm_topology *tp = apm.tp;
	struct apm_buf b;
	struct apm_module *m;
	struct apm_container *c;
	struct apm_sg *sg;
	size_t at, nconn, ncont, nmod;
	u_int i, j, k;
	bool in;

	apm_buf_init(&b, 8192);

	/* Subgraphs */
	at = apm_param_begin(&b, APM_MODULE_INSTANCE_ID,
	    APM_PARAM_ID_SUB_GRAPH_CONFIG);
	apm_put32(&b, g->nsg);
	for (i = 0; i < g->nsg; i++) {
		for (j = 0; tp->sgs[j].id != g->sg_ids[i]; j++)
			;
		sg = &tp->sgs[j];
		apm_put32(&b, sg->id);
		apm_put32(&b, 3);
		apm_put32(&b, APM_SUB_GRAPH_PROP_ID_PERF_MODE);
		apm_put32(&b, 4);
		apm_put32(&b, sg->perf_mode);
		apm_put32(&b, APM_SUB_GRAPH_PROP_ID_DIRECTION);
		apm_put32(&b, 4);
		apm_put32(&b, sg->direction);
		apm_put32(&b, APM_SUB_GRAPH_PROP_ID_SCENARIO_ID);
		apm_put32(&b, 4);
		apm_put32(&b, sg->scenario);
	}
	apm_param_end(&b, at, 8);

	/* Containers */
	ncont = 0;
	for (k = 0; k < tp->nconts; k++)
		for (i = 0; i < g->nsg; i++)
			if (tp->sgs[tp->conts[k].sg].id == g->sg_ids[i])
				ncont++;
	at = apm_param_begin(&b, APM_MODULE_INSTANCE_ID,
	    APM_PARAM_ID_CONTAINER_CONFIG);
	apm_put32(&b, ncont);
	for (i = 0; i < g->nsg; i++)
		for (k = 0; k < tp->nconts; k++) {
			c = &tp->conts[k];
			if (tp->sgs[c->sg].id != g->sg_ids[i])
				continue;
			apm_put32(&b, c->id);
			apm_put32(&b, 4);
			apm_put32(&b, APM_CONTAINER_PROP_ID_CAPABILITY_LIST);
			apm_put32(&b, 8);
			apm_put32(&b, 1);
			apm_put32(&b, c->capability);
			apm_put32(&b, APM_CONTAINER_PROP_ID_GRAPH_POS);
			apm_put32(&b, 4);
			apm_put32(&b, c->graph_pos);
			apm_put32(&b, APM_CONTAINER_PROP_ID_STACK_SIZE);
			apm_put32(&b, 4);
			apm_put32(&b, c->stack_size);
			apm_put32(&b, APM_CONTAINER_PROP_ID_PROC_DOMAIN);
			apm_put32(&b, 4);
			apm_put32(&b, c->proc_domain);
		}
	apm_param_end(&b, at, 8);

	/* Modules, by container */
	at = apm_param_begin(&b, APM_MODULE_INSTANCE_ID,
	    APM_PARAM_ID_MODULE_LIST);
	apm_put32(&b, ncont);
	nmod = 0;
	for (i = 0; i < g->nsg; i++)
		for (k = 0; k < tp->nconts; k++) {
			c = &tp->conts[k];
			if (tp->sgs[c->sg].id != g->sg_ids[i])
				continue;
			apm_put32(&b, g->sg_ids[i]);
			apm_put32(&b, c->id);
			{
				uint32_t *n = apm_put(&b, 4);

				*n = 0;
				for (j = 0; j < tp->nmods; j++) {
					if (tp->mods[j].container != k)
						continue;
					apm_put32(&b, tp->mods[j].mid);
					apm_put32(&b, tp->mods[j].iid);
					(*n)++;
					nmod++;
				}
			}
		}
	apm_param_end(&b, at, 8);

	/* Each module's ports */
	at = apm_param_begin(&b, APM_MODULE_INSTANCE_ID,
	    APM_PARAM_ID_MODULE_PROP);
	apm_put32(&b, nmod);
	for (j = 0; j < tp->nmods; j++) {
		m = &tp->mods[j];
		for (i = 0, in = false; i < g->nsg; i++)
			in |= apm_sg_of(m)->id == g->sg_ids[i];
		if (!in)
			continue;
		apm_put32(&b, m->iid);
		apm_put32(&b, 1);
		apm_put32(&b, APM_MODULE_PROP_ID_PORT_INFO);
		apm_put32(&b, 8);
		apm_put32(&b, m->max_ip_ports);
		apm_put32(&b, m->max_op_ports);
	}
	apm_param_end(&b, at, 8);

	/* Connections: the link in, then the modules' own */
	at = apm_param_begin(&b, APM_MODULE_INSTANCE_ID,
	    APM_PARAM_ID_MODULE_CONN);
	{
		uint32_t *n = apm_put(&b, 4);

		nconn = 0;
		if (g->link_src != 0 && g->link_dst != 0) {
			apm_put32(&b, g->link_src);
			apm_put32(&b, 1);
			apm_put32(&b, g->link_dst);
			apm_put32(&b, 2);
			nconn++;
		}
		for (j = 0; j < tp->nmods; j++) {
			m = &tp->mods[j];
			for (i = 0, in = false; i < g->nsg; i++)
				in |= apm_sg_of(m)->id == g->sg_ids[i];
			if (!in || m->max_op_ports == 0 || m->dst_iid == 0)
				continue;
			apm_put32(&b, m->iid);
			apm_put32(&b, m->src_op_port);
			apm_put32(&b, m->dst_iid);
			apm_put32(&b, m->dst_ip_port);
			nconn++;
		}
		*n = nconn;
	}
	apm_param_end(&b, at, 8);

	return (apm_cmd(apm.port, APM_MODULE_INSTANCE_ID, APM_CMD_GRAPH_OPEN,
	    &b, 0, NULL, NULL));
}

static int
apm_graph_mgmt(struct apm_graph *g, uint32_t opcode)
{
	struct apm_buf b;
	size_t at;
	u_int i;

	apm_buf_init(&b, 256);
	at = apm_param_begin(&b, APM_MODULE_INSTANCE_ID,
	    APM_PARAM_ID_SUB_GRAPH_LIST);
	apm_put32(&b, g->nsg);
	for (i = 0; i < g->nsg; i++)
		apm_put32(&b, g->sg_ids[i]);
	apm_param_end(&b, at, 8);
	return (apm_cmd(apm.port, APM_MODULE_INSTANCE_ID, opcode, &b, 0, NULL,
	    NULL));
}

static int
apm_set_u32(struct apm_module *m, uint32_t param_id, uint32_t v)
{
	struct apm_buf b;
	size_t at;

	apm_buf_init(&b, 64);
	at = apm_param_begin(&b, m->iid, param_id);
	apm_put32(&b, v);
	apm_param_end(&b, at, 4);
	return (apm_cmd(apm.port, APM_MODULE_INSTANCE_ID, APM_CMD_SET_CFG, &b,
	    0, NULL, NULL));
}

static void
apm_put_chmap8(struct apm_buf *b, const struct apm_pcm_cfg *cfg)
{
	u_int i;

	for (i = 0; i < cfg->channels; i++)
		*(uint8_t *)apm_put(b, 1) = cfg->chmap[i];
}

static int
apm_codec_dma_format(struct apm_module *m, const struct apm_pcm_cfg *cfg)
{
	struct apm_buf b;
	uint32_t mask;
	size_t at;
	u_int i;

	apm_buf_init(&b, 256);
	at = apm_param_begin(&b, m->iid, PARAM_ID_HW_EP_MF_CFG);
	apm_put32(&b, cfg->rate);
	apm_put32(&b, cfg->bits | (uint32_t)cfg->channels << 16);
	apm_put32(&b, m->data_format);
	apm_param_end(&b, at, 8);
	at = apm_param_begin(&b, m->iid, PARAM_ID_HW_EP_FRAME_SIZE_FACTOR);
	apm_put32(&b, 1);
	apm_param_end(&b, at, 8);
	at = apm_param_begin(&b, m->iid, PARAM_ID_CODEC_DMA_INTF_CFG);
	apm_put32(&b, m->hw_if_type);
	apm_put32(&b, m->hw_if_idx);
	for (i = 0, mask = 0; i < nitems(cfg->chmap); i++)
		if (cfg->chmap[i] != 0)
			mask |= 1u << i;
	apm_put32(&b, mask);
	apm_param_end(&b, at, 8);
	at = apm_param_begin(&b, m->iid, PARAM_ID_HW_EP_POWER_MODE_CFG);
	apm_put32(&b, 0);
	apm_param_end(&b, at, 8);
	return (apm_cmd(apm.port, APM_MODULE_INSTANCE_ID, APM_CMD_SET_CFG, &b,
	    0, NULL, NULL));
}

static int
apm_mfc_format(struct apm_module *m, const struct apm_pcm_cfg *cfg)
{
	struct apm_buf b;
	size_t at;
	u_int i;

	apm_buf_init(&b, 128);
	at = apm_param_begin(&b, m->iid, PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT);
	apm_put32(&b, cfg->rate);
	apm_put32(&b, cfg->bits | (uint32_t)cfg->channels << 16);
	for (i = 0; i < cfg->channels; i++) {
		uint16_t ch = cfg->chmap[i];

		memcpy(apm_put(&b, 2), &ch, 2);
	}
	apm_param_end(&b, at, 4);
	return (apm_cmd(apm.port, APM_MODULE_INSTANCE_ID, APM_CMD_SET_CFG, &b,
	    0, NULL, NULL));
}

static int
apm_pcm_format(struct apm_module *m, const struct apm_pcm_cfg *cfg)
{
	struct apm_buf b;
	size_t at;

	apm_buf_init(&b, 128);
	at = apm_param_begin(&b, m->iid, PARAM_ID_PCM_OUTPUT_FORMAT_CFG);
	apm_put32(&b, DATA_FORMAT_FIXED_POINT);
	apm_put32(&b, MEDIA_FMT_ID_PCM);
	apm_put32(&b, roundup2(16 + cfg->channels, 4));
	apm_put32(&b, cfg->bits | PCM_LSB_ALIGNED << 16);
	apm_put32(&b, cfg->bits | (uint32_t)(cfg->bits - 1) << 16);
	apm_put32(&b, PCM_LITTLE_ENDIAN | m->interleave << 16);
	apm_put32(&b, 0 | (uint32_t)cfg->channels << 16);
	apm_put_chmap8(&b, cfg);
	apm_param_end(&b, at, 8);
	return (apm_cmd(apm.port, APM_MODULE_INSTANCE_ID, APM_CMD_SET_CFG, &b,
	    0, NULL, NULL));
}

static int
apm_logging_format(struct apm_module *m)
{
	struct apm_buf b;
	size_t at;
	int error;

	error = apm_set_u32(m, PARAM_ID_MODULE_ENABLE, 1);
	if (error != 0)
		return (error);
	apm_buf_init(&b, 64);
	at = apm_param_begin(&b, m->iid, PARAM_ID_DATA_LOGGING_CONFIG);
	apm_put32(&b, m->log_code);
	apm_put32(&b, m->log_tap);
	apm_put32(&b, m->log_mode);
	apm_param_end(&b, at, 4);
	return (apm_cmd(apm.port, APM_MODULE_INSTANCE_ID, APM_CMD_SET_CFG, &b,
	    0, NULL, NULL));
}

/* The shared-memory endpoint's format goes to its own port. */
static int
apm_shmem_format(struct apm_graph *g, struct apm_module *m,
    const struct apm_pcm_cfg *cfg)
{
	struct apm_buf b;
	size_t at;

	apm_buf_init(&b, 128);
	at = apm_param_begin(&b, m->iid, PARAM_ID_MEDIA_FORMAT);
	apm_put32(&b, DATA_FORMAT_FIXED_POINT);
	apm_put32(&b, MEDIA_FMT_ID_PCM);
	apm_put32(&b, 0);	/* the payload's size, below */
	apm_put32(&b, cfg->rate);
	apm_put32(&b, cfg->bits | PCM_LSB_ALIGNED << 16);
	apm_put32(&b, cfg->bits | (uint32_t)(cfg->bits - 1) << 16);
	apm_put32(&b, PCM_LITTLE_ENDIAN | (uint32_t)cfg->channels << 16);
	apm_put_chmap8(&b, cfg);
	apm_param_end(&b, at, 8);
	{
		/* As Linux: the parameter's size less the header's 12. */
		struct apm_param *ph = (struct apm_param *)(b.p + at);
		uint32_t psz = ph->param_size + sizeof(*ph) - 12;

		memcpy(b.p + at + sizeof(*ph) + 8, &psz, 4);
	}
	return (apm_cmd(g->port, m->iid, APM_CMD_SET_CFG, &b, 0, NULL, NULL));
}

static bool
apm_module_in(struct apm_graph *g, struct apm_module *m)
{
	u_int i;

	for (i = 0; i < g->nsg; i++)
		if (apm_sg_of(m)->id == g->sg_ids[i])
			return (true);
	return (false);
}

/* Each module's media format, as its kind takes it. */
static int
apm_graph_formats(struct apm_graph *g, const struct apm_pcm_cfg *cfg)
{
	struct apm_module *m;
	u_int j;
	int error;

	for (j = 0; j < apm.tp->nmods; j++) {
		m = &apm.tp->mods[j];
		if (!apm_module_in(g, m))
			continue;
		switch (m->mid) {
		case MODULE_ID_DATA_LOGGING:
			error = apm_logging_format(m);
			break;
		case MODULE_ID_PCM_DEC:
		case MODULE_ID_PCM_ENC:
		case MODULE_ID_PCM_CNV:
			error = apm_pcm_format(m, cfg);
			break;
		case MODULE_ID_CODEC_DMA_SINK:
		case MODULE_ID_CODEC_DMA_SOURCE:
			error = apm_codec_dma_format(m, cfg);
			break;
		case MODULE_ID_MFC:
			error = apm_mfc_format(m, cfg);
			break;
		case MODULE_ID_WR_SHARED_MEM_EP:
			error = apm_shmem_format(g, m, cfg);
			break;
		default:
			error = 0;
			break;
		}
		if (error != 0) {
			printf("qcom_apm: %s's format: %d\n", m->name, error);
			return (error);
		}
	}
	return (0);
}

static int
apm_set_volume(struct apm_graph *g, uint16_t gain)
{
	struct apm_module *m;
	u_int j;
	int error;

	for (j = 0; j < apm.tp->nmods; j++) {
		m = &apm.tp->mods[j];
		if (!apm_module_in(g, m) || m->mid != MODULE_ID_VOL_CTRL)
			continue;
		error = apm_set_u32(m, PARAM_ID_VOL_CTRL_MASTER_GAIN, gain);
		if (error != 0)
			return (error);
	}
	return (0);
}

/* Map a buffer for the DSP: its handle. */
static int
apm_map(uint32_t graph_id, vm_paddr_t pa, size_t size, uint32_t *handle)
{
	uint32_t pkt[9] = { 0 };
	size_t len;
	int error;

	/* As Linux: in the command header's place, which stays sized. */
	pkt[0] = APM_MEMORY_MAP_SHMEM8_4K_POOL | 1 << 16;
	pkt[1] = 0;
	pkt[2] = (uint32_t)pa;
	pkt[3] = (uint32_t)((uint64_t)pa >> 32);
	pkt[4] = roundup2(size, PAGE_SIZE);
	len = sizeof(*handle);
	error = qcom_gpr_cmd(apm.port, APM_MODULE_INSTANCE_ID,
	    APM_CMD_SHARED_MEM_MAP_REGIONS, pkt, sizeof(pkt),
	    APM_CMD_RSP_SHARED_MEM_MAP_REGIONS, handle, &len);
	(void)graph_id;
	if (error == 0 && len != sizeof(*handle))
		error = EIO;
	return (error);
}

static int
apm_unmap(uint32_t handle)
{
	/* Like the map: the handle in the command header's place. */
	uint32_t pkt[5] = { handle, 0, 0, 0, 0 };

	return (qcom_gpr_cmd(apm.port, APM_MODULE_INSTANCE_ID,
	    APM_CMD_SHARED_MEM_UNMAP_REGIONS, pkt, sizeof(pkt), 0, NULL, NULL));
}

/* Graphs */

static void	apm_graph_rx(void *arg, const struct qcom_gpr_hdr *hdr,
		    const void *payload, size_t len);

static int
apm_graph_init(struct apm_graph *g, uint32_t id)
{
	u_int i;
	int error;

	memset(g, 0, sizeof(*g));
	g->id = id;
	for (i = 0; i < apm.tp->nsgs; i++)
		if (apm.tp->sgs[i].graph == id)
			g->sg_ids[g->nsg++] = apm.tp->sgs[i].id;
	if (g->nsg == 0)
		return (ENOENT);
	error = qcom_gpr_port_open(APM_GRAPH_PORT_BASE + id, apm_graph_rx, g,
	    &g->port);
	return (error);
}

static void
apm_graph_fini(struct apm_graph *g)
{

	if (g->port != NULL)
		qcom_gpr_port_close(g->port);
	g->port = NULL;
}

static struct apm_module *
apm_graph_module(struct apm_graph *g, uint32_t mid)
{
	u_int j;

	for (j = 0; j < apm.tp->nmods; j++)
		if (apm.tp->mods[j].mid == mid &&
		    apm_module_in(g, &apm.tp->mods[j]))
			return (&apm.tp->mods[j]);
	return (NULL);
}

static struct apm_mixer *
apm_mixer(const char *widget, const char *control)
{
	u_int i;

	for (i = 0; i < apm.tp->nmixers; i++)
		if (strcmp(apm.tp->mixers[i].widget, widget) == 0 &&
		    strcmp(apm.tp->mixers[i].control, control) == 0)
			return (&apm.tp->mixers[i]);
	return (NULL);
}

/*
 * Playback: the MultiMedia1 stream graph, fed from a buffer of ours, into
 * the RX_CODEC_DMA_RX_0 device graph and on through the codec to the
 * headphones, in Linux's order (q6apm-dai and q6apm-lpass-dais).
 */

struct qcom_apm_play {
	struct apm_graph	fe;	/* the stream graph */
	struct apm_graph	be;	/* the device graph */
	struct apm_module	*shm;	/* where samples go in */
	vm_offset_t		buf;
	size_t			size;
	uint64_t		iova;
	uint64_t		dsp_addr;	/* the buffer, as the DSP has it */
	uint32_t		handle;		/* of its mapping */
	qcom_apm_done_t		*done;
	void			*arg;
	bool			fe_open, be_open, started;
};

static void
apm_graph_rx(void *arg, const struct qcom_gpr_hdr *hdr, const void *payload,
    size_t len)
{
	struct apm_graph *g = arg;
	struct qcom_apm_play *p;
	const uint32_t *w = payload;
	u_int i;

	if (hdr->opcode == DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2) {
		p = __containerof(g, struct qcom_apm_play, fe);
		p->done(p->arg, hdr->token & APM_WRITE_TOKEN_MASK,
		    len >= 16 ? w[3] : 0);
		return;
	}
	printf("qcom_apm: graph port %#x: opcode %#x token %#x from %#x:",
	    hdr->dst_port, hdr->opcode, hdr->token, hdr->src_port);
	for (i = 0; i < len / 4 && i < 6; i++)
		printf(" %#x", w[i]);
	printf("\n");
}

static int
apm_setup(void)
{
	int error;

	sx_assert(&apm.lock, SA_XLOCKED);
	error = apm_load_topology();
	if (error != 0)
		return (error);
	if (apm.dom == NULL) {
		error = qcom_apps_smmu_attach(APM_LPASS_STREAM, 0, &apm.dom);
		if (error != 0) {
			printf("qcom_apm: the DSP's DMA stream: %d\n", error);
			return (error);
		}
	}
	if (apm.port == NULL)
		error = qcom_gpr_port_open(QCOM_GPR_PORT_APM, NULL, NULL,
		    &apm.port);
	return (error);
}

/* Undo what qcom_apm_play_open got done, in reverse. */
static void
apm_play_close(struct qcom_apm_play *p)
{

	sx_assert(&apm.lock, SA_XLOCKED);
	if (p->started) {
		(void)apm_graph_mgmt(&p->be, APM_CMD_GRAPH_STOP);
		(void)apm_graph_mgmt(&p->fe, APM_CMD_GRAPH_STOP);
	}
	(void)qcom_wcd938x_hph(false);
	if (p->be_open)
		(void)apm_graph_mgmt(&p->be, APM_CMD_GRAPH_CLOSE);
	if (p->fe_open)
		(void)apm_graph_mgmt(&p->fe, APM_CMD_GRAPH_CLOSE);
	if (p->handle != 0)
		(void)apm_unmap(p->handle);
	if (p->iova != 0)
		qcom_apps_smmu_unmap(apm.dom, p->iova, p->size);
	apm_graph_fini(&p->fe);
	apm_graph_fini(&p->be);
	qcom_wcd938x_down();
	/* Nothing else talks to the APM; let the module unload. */
	if (apm.port != NULL)
		qcom_gpr_port_close(apm.port);
	apm.port = NULL;
	apm.busy = false;
	free(p, M_APM);
}

/*
 * Start playing 48 kHz 16-bit stereo from buf, which is physically
 * contiguous and size bytes long, a multiple of the page size: the codec
 * up, the graphs open and running, and the buffer mapped into the DSP.
 * Then qcom_apm_play_write hands it periods, and done is called (from a
 * thread that must not send to the DSP itself) as each has been consumed.
 * Sleeps.
 */
int
qcom_apm_play_open(vm_offset_t buf, size_t size, qcom_apm_done_t *done,
    void *arg, struct qcom_apm_play **pp)
{
	struct qcom_apm_play *p;
	struct apm_mixer *mx;
	struct apm_pcm_cfg cfg;
	int error;

	memset(&cfg, 0, sizeof(cfg));
	cfg.rate = 48000;
	cfg.bits = 16;
	cfg.channels = 2;
	cfg.chmap[0] = PCM_CHANNEL_FL;
	cfg.chmap[1] = PCM_CHANNEL_FR;

	sx_xlock(&apm.lock);
	if (apm.busy) {
		sx_xunlock(&apm.lock);
		return (EBUSY);
	}
	apm.busy = true;
	p = malloc(sizeof(*p), M_APM, M_WAITOK | M_ZERO);
	p->buf = buf;
	p->size = size;
	p->done = done;
	p->arg = arg;
	error = apm_setup();
	if (error != 0)
		goto fail;
	mx = apm_mixer("RX_CODEC_DMA_RX_0 Audio Mixer", "MultiMedia1");
	if (mx == NULL || mx->src_iid == 0 || mx->dst_iid == 0) {
		printf("qcom_apm: no MultiMedia1 switch to RX_CODEC_DMA_RX_0\n");
		error = ENOENT;
		goto fail;
	}
	error = apm_graph_init(&p->fe, mx->src_graph);
	if (error == 0)
		error = apm_graph_init(&p->be, mx->dst_graph);
	if (error != 0)
		goto fail;
	p->be.link_src = mx->src_iid;
	p->be.link_dst = mx->dst_iid;
	p->shm = apm_graph_module(&p->fe, MODULE_ID_WR_SHARED_MEM_EP);
	if (p->shm == NULL) {
		error = ENOENT;
		goto fail;
	}

	/* The codec the device graph's DMA feeds: up before the graphs. */
	error = qcom_wcd938x_up();
	if (error != 0)
		goto fail;
	/* Source graph first, then sink, as Linux does. */
	error = apm_graph_open_cmd(&p->fe);
	if (error != 0)
		goto fail;
	p->fe_open = true;
	error = apm_graph_open_cmd(&p->be);
	if (error != 0)
		goto fail;
	p->be_open = true;
	error = apm_graph_formats(&p->be, &cfg);
	if (error == 0)
		error = apm_graph_mgmt(&p->be, APM_CMD_GRAPH_PREPARE);
	if (error != 0)
		goto fail;

	error = qcom_apps_smmu_map(apm.dom, vtophys(buf), size, &p->iova);
	if (error != 0) {
		p->iova = 0;
		goto fail;
	}
	p->dsp_addr = APM_DSP_ADDR(p->iova);
	error = apm_map(p->fe.id, p->dsp_addr, size, &p->handle);
	if (error != 0) {
		p->handle = 0;
		goto fail;
	}
	error = apm_graph_formats(&p->fe, &cfg);
	if (error == 0)
		error = apm_graph_mgmt(&p->fe, APM_CMD_GRAPH_PREPARE);
	if (error == 0)
		error = apm_graph_mgmt(&p->fe, APM_CMD_GRAPH_START);
	if (error != 0)
		goto fail;
	p->started = true;
	error = apm_set_volume(&p->fe, VOL_CTRL_UNITY);
	if (error == 0)
		error = qcom_wcd938x_hph(true);
	if (error == 0)
		error = apm_graph_mgmt(&p->be, APM_CMD_GRAPH_START);
	if (error != 0)
		goto fail;
	sx_xunlock(&apm.lock);
	*pp = p;
	return (0);
fail:
	printf("qcom_apm: playback not started: %d\n", error);
	apm_play_close(p);
	sx_xunlock(&apm.lock);
	return (error);
}

/* Hand the DSP len bytes at off in the buffer; done gets token back. */
int
qcom_apm_play_write(struct qcom_apm_play *p, size_t off, size_t len,
    u_int token)
{
	uint32_t cmd[11] = { 0 };

	KASSERT(off + len <= p->size, ("qcom_apm: write past the buffer"));
	cmd[0] = (uint32_t)(p->dsp_addr + off);
	cmd[1] = (uint32_t)((p->dsp_addr + off) >> 32);
	cmd[2] = p->handle;
	cmd[3] = len;
	cmd[6] = WR_SH_MEM_NO_TIMESTAMP;
	return (qcom_gpr_send(p->fe.port, p->shm->iid,
	    DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2,
	    (token & APM_WRITE_TOKEN_MASK) | len << APM_WRITE_TOKEN_LEN_SHIFT,
	    cmd, sizeof(cmd)));
}

/* The stream's gain, in Q13: 0x2000 is unity. */
int
qcom_apm_play_volume(struct qcom_apm_play *p, uint16_t gain)
{
	int error;

	sx_xlock(&apm.lock);
	error = apm_set_volume(&p->fe, gain);
	sx_xunlock(&apm.lock);
	return (error);
}

/* Stop: the DSP gives back what it held, then everything comes down. */
void
qcom_apm_play_close(struct qcom_apm_play *p)
{

	sx_xlock(&apm.lock);
	apm_play_close(p);
	sx_xunlock(&apm.lock);
}

/* The test tone, through the playback interface. */

#define	TONE_PERIOD	(48000 / 100 * 4)	/* 10 ms of 16-bit stereo */
#define	TONE_PERIODS	8

static const int16_t tone_sine[48] = {
	0, 535, 1060, 1567, 2045, 2485, 2879, 3219, 3500, 3714, 3858, 3929,
	3929, 3858, 3714, 3500, 3219, 2879, 2485, 2045, 1567, 1060, 535, 0,
	-535, -1060, -1567, -2045, -2485, -2879, -3219, -3500, -3714, -3858,
	-3929, -3929, -3858, -3714, -3500, -3219, -2879, -2485, -2045, -1567,
	-1060, -535, 0, 0,
};

static struct {
	struct mtx	mtx;
	struct cv	cv;
	u_int		done;
	u_int		errors;
} tone;

MTX_SYSINIT(qcom_apm_tone, &tone.mtx, "qcom_apm tone", MTX_DEF);

static void
apm_tone_done(void *arg __unused, u_int token __unused, uint32_t status)
{

	mtx_lock(&tone.mtx);
	if (status != 0)
		tone.errors++;
	tone.done++;
	cv_broadcast(&tone.cv);
	mtx_unlock(&tone.mtx);
}

static int
apm_tone(u_int seconds)
{
	struct qcom_apm_play *p;
	int16_t *buf;
	size_t size;
	u_int i, n, sent, total;
	int error;

	size = round_page(TONE_PERIOD * TONE_PERIODS);
	buf = kmem_alloc_contig(size, M_WAITOK | M_ZERO, 0, BUS_SPACE_MAXADDR,
	    PAGE_SIZE, 0, VM_MEMATTR_WRITE_COMBINING);
	for (i = 0; i < size / sizeof(*buf) / 2; i++)
		buf[2 * i] = buf[2 * i + 1] = tone_sine[i % nitems(tone_sine)];
	mtx_lock(&tone.mtx);
	tone.done = tone.errors = 0;
	mtx_unlock(&tone.mtx);
	error = qcom_apm_play_open((vm_offset_t)buf, size, apm_tone_done, NULL,
	    &p);
	if (error != 0) {
		kmem_free(buf, size);
		return (error);
	}

	/* Keep every period queued until enough have played. */
	total = seconds * 100;
	for (sent = 0; sent < TONE_PERIODS && sent < total && error == 0;
	    sent++)
		error = qcom_apm_play_write(p, sent * TONE_PERIOD,
		    TONE_PERIOD, sent);
	mtx_lock(&tone.mtx);
	while (error == 0 && tone.done < total) {
		n = tone.done;
		if (cv_timedwait(&tone.cv, &tone.mtx, hz) == EWOULDBLOCK &&
		    tone.done == n) {
			error = ETIMEDOUT;
			break;
		}
		while (error == 0 && sent < total &&
		    sent - tone.done < TONE_PERIODS) {
			mtx_unlock(&tone.mtx);
			error = qcom_apm_play_write(p,
			    (sent % TONE_PERIODS) * TONE_PERIOD, TONE_PERIOD,
			    sent % TONE_PERIODS);
			mtx_lock(&tone.mtx);
			if (error == 0)
				sent++;
		}
	}
	printf("qcom_apm: tone: %u of %u periods played, %u in error\n",
	    tone.done, total, tone.errors);
	mtx_unlock(&tone.mtx);
	qcom_apm_play_close(p);
	kmem_free(buf, size);
	return (error);
}

static int
apm_tone_sysctl(SYSCTL_HANDLER_ARGS)
{
	u_int seconds = 0;
	int error;

	error = sysctl_handle_int(oidp, &seconds, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (seconds == 0 || seconds > 60)
		return (EINVAL);
	return (apm_tone(seconds));
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_apm, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm audio DSP's APM");
SYSCTL_PROC(_hw_qcom_apm, OID_AUTO, tone, CTLTYPE_UINT | CTLFLAG_RW |
    CTLFLAG_MPSAFE, NULL, 0, apm_tone_sysctl, "IU",
    "Play a 1 kHz tone to the headphones for so many seconds");

static int
qcom_apm_modevent(module_t mod, int type, void *data)
{

	switch (type) {
	case MOD_LOAD:
		cv_init(&tone.cv, "qcom_apm tone");
		return (0);
	case MOD_UNLOAD:
		sx_xlock(&apm.lock);
		if (apm.busy) {
			sx_xunlock(&apm.lock);
			return (EBUSY);
		}
		free(apm.tp, M_APM);
		apm.tp = NULL;
		sx_xunlock(&apm.lock);
		cv_destroy(&tone.cv);
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t qcom_apm_mod = { "qcom_apm", qcom_apm_modevent, NULL };
DECLARE_MODULE(qcom_apm, qcom_apm_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_DEPEND(qcom_apm, qcom_gpr, 1, 1, 1);
MODULE_DEPEND(qcom_apm, firmware, 1, 1, 1);
MODULE_DEPEND(qcom_apm, qcom_smmu, 1, 1, 1);
MODULE_VERSION(qcom_apm, 1);
