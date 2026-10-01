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
 * sound(4) for the Qualcomm audio DSP: one playback channel, 48 kHz 16-bit
 * stereo to the headphones (vchans convert the rest), through qcom_apm.
 *
 * The channel's ring buffer is the DSP's: mapped into it whole, it hands
 * the DSP blocks by offset, a few ahead of the one playing.  Each block the
 * DSP finishes is an "interrupt": the hardware pointer moves on a block,
 * chn_intr() refills it, and the next block goes to the DSP.  Talking to
 * the DSP sleeps and sound(4) triggers with the channel locked, so starting
 * and stopping, and the work for each finished block, run on a task queue.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_snd.h"
#endif

#include <dev/sound/pcm/sound.h>

#include <dev/qcom_audio/qcom_apm.h>

#include "channel_if.h"

#define	PCM_RATE	48000
#define	PCM_FRAME	4		/* 16-bit stereo */
#define	PCM_BUFSZ	(64 * 1024)	/* the ring at most */
#define	PCM_BLKSZ	(PCM_RATE / 100 * PCM_FRAME)	/* 10 ms */
#define	PCM_AHEAD	2		/* blocks with the DSP */

struct qcom_pcm_softc {
	struct snddev_info	info;	/* first, for sound(4) */
	device_t		dev;
	struct pcm_channel	*pcm;
	struct snd_dbuf		*buf;
	void			*ring;
	struct taskqueue	*tq;
	struct task		start_task;
	struct task		stop_task;
	struct task		done_task;
	struct qcom_apm_play	*play;	/* the task's alone */
	u_int			blksz;
	u_int			blkcnt;
	/* Under the channel lock: */
	bool			running;
	u_int			played;	/* blocks finished since start */
	/* The task's: */
	u_int			sent;	/* blocks handed to the DSP */
	volatile u_int		finished; /* by the DSP, not yet counted */
};

static uint32_t qcom_pcm_fmt[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0
};
static struct pcmchan_caps qcom_pcm_caps = { PCM_RATE, PCM_RATE,
    qcom_pcm_fmt, 0 };

/* Channel */

static void *
qcom_pcm_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct qcom_pcm_softc *sc = devinfo;

	if (dir != PCMDIR_PLAY)
		return (NULL);
	if (sndbuf_setup(b, sc->ring, PCM_BUFSZ) != 0)
		return (NULL);
	sc->pcm = c;
	sc->buf = b;
	return (sc);
}

static int
qcom_pcm_chan_setformat(kobj_t obj, void *data, uint32_t format)
{

	return (format == qcom_pcm_fmt[0] ? 0 : EINVAL);
}

static uint32_t
qcom_pcm_chan_setspeed(kobj_t obj, void *data, uint32_t speed)
{

	return (PCM_RATE);
}

static uint32_t
qcom_pcm_chan_setblocksize(kobj_t obj, void *data, uint32_t blksz)
{
	struct qcom_pcm_softc *sc = data;

	/* Whole frames, from 5 ms up to a quarter of the ring. */
	blksz = rounddown2(blksz, PCM_FRAME);
	blksz = MAX(blksz, PCM_BLKSZ / 2);
	blksz = MIN(blksz, PCM_BUFSZ / 4);
	if (sndbuf_resize(sc->buf, PCM_BUFSZ / blksz, blksz) == 0) {
		sc->blksz = blksz;
		sc->blkcnt = PCM_BUFSZ / blksz;
	}
	return (sc->blksz);
}

static int
qcom_pcm_chan_trigger(kobj_t obj, void *data, int go)
{
	struct qcom_pcm_softc *sc = data;

	switch (go) {
	case PCMTRIG_START:
		sc->running = true;
		sc->played = 0;
		taskqueue_enqueue(sc->tq, &sc->start_task);
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (sc->running) {
			sc->running = false;
			taskqueue_enqueue(sc->tq, &sc->stop_task);
		}
		break;
	default:
		break;
	}
	return (0);
}

static uint32_t
qcom_pcm_chan_getptr(kobj_t obj, void *data)
{
	struct qcom_pcm_softc *sc = data;

	return ((sc->played % sc->blkcnt) * sc->blksz);
}

static struct pcmchan_caps *
qcom_pcm_chan_getcaps(kobj_t obj, void *data)
{

	return (&qcom_pcm_caps);
}

static kobj_method_t qcom_pcm_chan_methods[] = {
	KOBJMETHOD(channel_init,	qcom_pcm_chan_init),
	KOBJMETHOD(channel_setformat,	qcom_pcm_chan_setformat),
	KOBJMETHOD(channel_setspeed,	qcom_pcm_chan_setspeed),
	KOBJMETHOD(channel_setblocksize, qcom_pcm_chan_setblocksize),
	KOBJMETHOD(channel_trigger,	qcom_pcm_chan_trigger),
	KOBJMETHOD(channel_getptr,	qcom_pcm_chan_getptr),
	KOBJMETHOD(channel_getcaps,	qcom_pcm_chan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(qcom_pcm_chan);

/* The DSP's side, on the task queue */

/* Hand the DSP the next block. */
static int
qcom_pcm_send(struct qcom_pcm_softc *sc)
{
	u_int blk;

	blk = sc->sent % sc->blkcnt;
	sc->sent++;
	return (qcom_apm_play_write(sc->play, blk * sc->blksz, sc->blksz,
	    blk));
}

/* From the DSP link's thread, which mustn't talk to the DSP itself. */
static void
qcom_pcm_done(void *arg, u_int token __unused, uint32_t status)
{
	struct qcom_pcm_softc *sc = arg;

	if (status != 0)
		device_printf(sc->dev, "DSP status %#x for a block\n", status);
	atomic_add_int(&sc->finished, 1);
	taskqueue_enqueue(sc->tq, &sc->done_task);
}

static void
qcom_pcm_start_task(void *arg, int pending __unused)
{
	struct qcom_pcm_softc *sc = arg;
	u_int i;
	int error;

	if (sc->play != NULL)
		return;
	sc->sent = 0;
	atomic_store_int(&sc->finished, 0);
	error = qcom_apm_play_open((vm_offset_t)sc->ring, PCM_BUFSZ,
	    qcom_pcm_done, sc, &sc->play);
	if (error != 0) {
		device_printf(sc->dev, "playback not started: %d\n", error);
		sc->play = NULL;
		return;
	}
	for (i = 0, error = 0; i < PCM_AHEAD && error == 0; i++)
		error = qcom_pcm_send(sc);
	if (error != 0)
		device_printf(sc->dev, "write: %d\n", error);
}

static void
qcom_pcm_done_task(void *arg, int pending __unused)
{
	struct qcom_pcm_softc *sc = arg;
	u_int n;
	bool running;

	n = atomic_readandclear_int(&sc->finished);
	if (sc->play == NULL || n == 0)
		return;
	CHN_LOCK(sc->pcm);
	running = sc->running;
	if (running)
		sc->played += n;
	CHN_UNLOCK(sc->pcm);
	if (!running)
		return;
	while (n-- > 0) {
		chn_intr(sc->pcm);
		if (qcom_pcm_send(sc) != 0)
			break;
	}
}

static void
qcom_pcm_stop_task(void *arg, int pending __unused)
{
	struct qcom_pcm_softc *sc = arg;

	if (sc->play == NULL)
		return;
	qcom_apm_play_close(sc->play);
	sc->play = NULL;
}

/* Device */

static void
qcom_pcm_identify(driver_t *driver, device_t parent)
{

	if (!qcom_apm_supported() ||
	    device_find_child(parent, "pcm", DEVICE_UNIT_ANY) != NULL)
		return;
	BUS_ADD_CHILD(parent, 0, "pcm", DEVICE_UNIT_ANY);
}

static int
qcom_pcm_probe(device_t dev)
{

	if (!qcom_apm_supported())
		return (ENXIO);
	device_set_desc(dev, "Qualcomm audio DSP");
	return (BUS_PROBE_NOWILDCARD);
}

static int
qcom_pcm_attach(device_t dev)
{
	struct qcom_pcm_softc *sc = device_get_softc(dev);
	int error;

	sc->dev = dev;
	sc->blksz = PCM_BLKSZ;
	sc->blkcnt = PCM_BUFSZ / PCM_BLKSZ;
	/* Write-combined: the CPU only writes it, the DSP reads it. */
	sc->ring = kmem_alloc_contig(PCM_BUFSZ, M_WAITOK | M_ZERO, 0,
	    BUS_SPACE_MAXADDR, PAGE_SIZE, 0, VM_MEMATTR_WRITE_COMBINING);
	sc->tq = taskqueue_create("qcom_pcm", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PI_SOFT, "%s taskq",
	    device_get_nameunit(dev));
	TASK_INIT(&sc->start_task, 0, qcom_pcm_start_task, sc);
	TASK_INIT(&sc->stop_task, 0, qcom_pcm_stop_task, sc);
	TASK_INIT(&sc->done_task, 0, qcom_pcm_done_task, sc);

	pcm_init(dev, sc);
	error = pcm_addchan(dev, PCMDIR_PLAY, &qcom_pcm_chan_class, sc);
	if (error == 0)
		error = pcm_register(dev, "on the audio DSP");
	if (error != 0) {
		taskqueue_free(sc->tq);
		kmem_free(sc->ring, PCM_BUFSZ);
		return (error);
	}
	return (0);
}

static int
qcom_pcm_detach(device_t dev)
{
	struct qcom_pcm_softc *sc = device_get_softc(dev);
	int error;

	error = pcm_unregister(dev);
	if (error != 0)
		return (error);
	/* The channels are gone, so no more triggers: finish the stop. */
	taskqueue_enqueue(sc->tq, &sc->stop_task);
	taskqueue_drain_all(sc->tq);
	taskqueue_free(sc->tq);
	kmem_free(sc->ring, PCM_BUFSZ);
	return (0);
}

static device_method_t qcom_pcm_methods[] = {
	DEVMETHOD(device_identify,	qcom_pcm_identify),
	DEVMETHOD(device_probe,		qcom_pcm_probe),
	DEVMETHOD(device_attach,	qcom_pcm_attach),
	DEVMETHOD(device_detach,	qcom_pcm_detach),
	DEVMETHOD_END
};

static driver_t qcom_pcm_driver = {
	"pcm",
	qcom_pcm_methods,
	sizeof(struct qcom_pcm_softc),
};

DRIVER_MODULE(qcom_pcm, nexus, qcom_pcm_driver, NULL, NULL);
MODULE_DEPEND(qcom_pcm, sound, 1, 1, 1);
MODULE_VERSION(qcom_pcm, 1);
