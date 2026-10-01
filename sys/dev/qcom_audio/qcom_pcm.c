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
 * the DSP sleeps and sound(4) triggers with the channel locked, so a trigger
 * only says what's wanted, and one task brings the stream there; it and
 * the work for each finished block run on a task queue.
 *
 * The mixer's master volume is the stream's gain in the DSP; sound(4)
 * supplies the PCM control in software.
 *
 * The headphone jack: once the DSP is up the codec is brought up and left
 * idle, watching the jack, and a change wakes its TX link, whose wake-up
 * interrupt this takes.  While playing, the links run and nothing wakes,
 * so the jack is looked at every second.  A change shows in
 * dev.pcm.N.jack and goes to devd(8) as system SND, subsystem JACK, type
 * INSERT or REMOVE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sysctl.h>
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
#include <dev/qcom_audio/qcom_swr.h>
#include <dev/qcom_audio/qcom_wcd938x.h>

#include "channel_if.h"
#include "mixer_if.h"

#define	PCM_RATE	48000
#define	PCM_FRAME	4		/* 16-bit stereo */
#define	PCM_BUFSZ	(64 * 1024)	/* the ring at most */
#define	PCM_MS		(PCM_RATE / 1000 * PCM_FRAME)	/* bytes */
#define	PCM_BLKSZ	(10 * PCM_MS)
#define	PCM_AHEAD_MS	40		/* queued with the DSP, about */
#define	PCM_CODEC_TRIES	120		/* a second apart, for the DSP */

struct qcom_pcm_softc {
	struct snddev_info	info;	/* first, for sound(4) */
	device_t		dev;
	struct pcm_channel	*pcm;
	struct snd_dbuf		*buf;
	void			*ring;
	struct taskqueue	*tq;
	struct task		sync_task;	/* the stream as wanted */
	struct task		done_task;
	struct task		vol_task;
	struct timeout_task	codec_task;	/* bring the codec up */
	struct task		jack_task;
	struct timeout_task	jack_poll_task;	/* while playing */
	struct resource		*wake_res;
	void			*wake_cookie;
	u_int			codec_tries;
	int			jack;		/* -1 unknown, 0 empty, 1 in */
	struct qcom_apm_play	*play;	/* the task's alone */
	u_int			blksz;
	u_int			blkcnt;
	volatile u_int		volume;	/* 0 to 100 */
	/* Under the channel lock: */
	bool			running;	/* wanted, by the last trigger */
	u_int			gen;		/* STARTs so far */
	u_int			play_gen;	/* the START sc->play serves */
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

/* The DSP's gain for each master volume: 0.6 dB a step, 100 unity. */
static const uint16_t qcom_pcm_gain[101] = {
	0, 9, 9, 10, 11, 12, 12, 13, 14, 15,
	16, 18, 19, 20, 22, 23, 25, 27, 28, 30,
	33, 35, 37, 40, 43, 46, 49, 53, 57, 61,
	65, 70, 75, 80, 86, 92, 98, 106, 113, 121,
	130, 139, 149, 160, 171, 183, 197, 211, 226, 242,
	259, 278, 297, 319, 341, 366, 392, 420, 450, 482,
	517, 554, 593, 636, 681, 730, 782, 838, 898, 962,
	1031, 1105, 1184, 1269, 1360, 1457, 1561, 1673, 1792, 1920,
	2058, 2205, 2363, 2532, 2713, 2907, 3115, 3337, 3576, 3832,
	4106, 4399, 4714, 5051, 5412, 5799, 6214, 6659, 7135, 7645,
	8192,
};

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

static void	qcom_pcm_sync_task(void *arg, int pending);

/*
 * The channel is going (after its last trigger, unlocked): the stream
 * stops, and nothing on the task queue touches the channel again.
 */
static int
qcom_pcm_chan_free(kobj_t obj, void *data)
{
	struct qcom_pcm_softc *sc = data;

	CHN_LOCK(sc->pcm);
	sc->running = false;
	CHN_UNLOCK(sc->pcm);
	taskqueue_enqueue(sc->tq, &sc->sync_task);
	taskqueue_drain_all(sc->tq);
	taskqueue_drain_timeout(sc->tq, &sc->jack_poll_task);
	return (0);
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

	/*
	 * Whole milliseconds, which the DSP processes in: a block ending
	 * part way through one leaves a glitch.  From 5 ms up to a quarter
	 * of the ring.
	 */
	blksz = rounddown(blksz, PCM_MS);
	blksz = MAX(blksz, 5 * PCM_MS);
	blksz = MIN(blksz, rounddown(PCM_BUFSZ / 4, PCM_MS));
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

	/*
	 * Only say what's wanted: the task brings the stream there, however
	 * many triggers came meanwhile.  A START is always a fresh stream,
	 * from the start of the ring, as sound(4) expects.
	 */
	switch (go) {
	case PCMTRIG_START:
		sc->running = true;
		sc->gen++;
		sc->played = 0;
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		sc->running = false;
		break;
	default:
		return (0);
	}
	taskqueue_enqueue(sc->tq, &sc->sync_task);
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
	KOBJMETHOD(channel_free,	qcom_pcm_chan_free),
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

/*
 * Keep the DSP about PCM_AHEAD_MS ahead, so that a late task doesn't leave
 * it short: hand it the blocks after the last it has, as long as sound(4)
 * has filled them.
 */
static int
qcom_pcm_fill(struct qcom_pcm_softc *sc)
{
	u_int ahead, blk, queued, ready;
	int error;

	ahead = MAX(2, howmany(PCM_AHEAD_MS * PCM_MS, sc->blksz));
	ahead = MIN(ahead, sc->blkcnt - 1);
	for (error = 0; error == 0;) {
		CHN_LOCK(sc->pcm);
		queued = sc->sent > sc->played ? sc->sent - sc->played : 0;
		ready = sndbuf_getready(sc->buf);
		CHN_UNLOCK(sc->pcm);
		/*
		 * Two blocks always, so that the DSP keeps finishing them
		 * and this keeps being called; more only once filled.
		 */
		if (queued >= ahead ||
		    (queued >= 2 && (queued + 1) * sc->blksz > ready))
			break;
		blk = sc->sent % sc->blkcnt;
		sc->sent++;
		error = qcom_apm_play_write(sc->play, blk * sc->blksz,
		    sc->blksz, blk);
	}
	if (error != 0)
		device_printf(sc->dev, "write: %d\n", error);
	return (error);
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
qcom_pcm_close(struct qcom_pcm_softc *sc)
{

	if (sc->play == NULL)
		return;
	qcom_apm_play_close(sc->play);
	sc->play = NULL;
}

/* Bring the stream to what the last trigger wanted. */
static void
qcom_pcm_sync_task(void *arg, int pending __unused)
{
	struct qcom_pcm_softc *sc = arg;
	u_int gen;
	bool want;
	int error;

	CHN_LOCK(sc->pcm);
	want = sc->running;
	gen = sc->gen;
	CHN_UNLOCK(sc->pcm);

	/* Stopped, or started again since: this stream is done. */
	if (sc->play != NULL && (!want || sc->play_gen != gen))
		qcom_pcm_close(sc);
	if (!want || sc->play != NULL)
		return;

	/* The DSP's answers for an earlier stream are all in by now. */
	sc->sent = 0;
	atomic_store_int(&sc->finished, 0);
	error = qcom_apm_play_open((vm_offset_t)sc->ring, PCM_BUFSZ,
	    qcom_pcm_done, sc, &sc->play);
	if (error != 0) {
		device_printf(sc->dev, "playback not started: %d\n", error);
		sc->play = NULL;
		return;
	}
	CHN_LOCK(sc->pcm);
	sc->play_gen = gen;
	CHN_UNLOCK(sc->pcm);
	(void)qcom_apm_play_volume(sc->play, qcom_pcm_gain[sc->volume]);
	(void)qcom_pcm_fill(sc);
	taskqueue_enqueue_timeout(sc->tq, &sc->jack_poll_task, hz);
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
	/* Only for the stream sound(4) is running now. */
	CHN_LOCK(sc->pcm);
	running = sc->running && sc->play_gen == sc->gen;
	if (running)
		sc->played += n;
	CHN_UNLOCK(sc->pcm);
	if (!running)
		return;
	while (n-- > 0)
		chn_intr(sc->pcm);
	(void)qcom_pcm_fill(sc);
}

static void
qcom_pcm_vol_task(void *arg, int pending __unused)
{
	struct qcom_pcm_softc *sc = arg;

	if (sc->play != NULL)
		(void)qcom_apm_play_volume(sc->play,
		    qcom_pcm_gain[sc->volume]);
}

/* The jack */

static void
qcom_pcm_jack(void *arg, bool plugged)
{
	struct qcom_pcm_softc *sc = arg;
	char buf[32];

	sc->jack = plugged;
	if (bootverbose)
		device_printf(sc->dev, "headphone jack %s\n",
		    plugged ? "in use" : "empty");
	snprintf(buf, sizeof(buf), "cdev=dsp%d", device_get_unit(sc->dev));
	devctl_notify("SND", "JACK", plugged ? "INSERT" : "REMOVE", buf);
}

/* The codec's TX link asks for its clock: something changed at the jack. */
static void
qcom_pcm_wake(void *arg)
{
	struct qcom_pcm_softc *sc = arg;

	if (qcom_swr_wake_take(QCOM_SWR_TX))
		taskqueue_enqueue(sc->tq, &sc->jack_task);
}

static void
qcom_pcm_jack_task(void *arg, int pending __unused)
{

	qcom_wcd938x_jack_check();
}

static void
qcom_pcm_jack_poll_task(void *arg, int pending __unused)
{
	struct qcom_pcm_softc *sc = arg;

	if (sc->play == NULL)
		return;
	qcom_wcd938x_jack_check();
	taskqueue_enqueue_timeout(sc->tq, &sc->jack_poll_task, hz);
}

/*
 * Bring the codec up once the DSP is, which may be a while after boot, and
 * leave it idle and watching the jack.
 */
static void
qcom_pcm_codec_task(void *arg, int pending __unused)
{
	struct qcom_pcm_softc *sc = arg;

	if (qcom_wcd938x_up() == 0) {
		qcom_wcd938x_down();
		return;
	}
	if (++sc->codec_tries < PCM_CODEC_TRIES)
		taskqueue_enqueue_timeout(sc->tq, &sc->codec_task, hz);
	else
		device_printf(sc->dev, "codec not up; no jack detection\n");
}

/* Mixer */

static int
qcom_pcm_mixer_init(struct snd_mixer *m)
{

	mix_setdevs(m, SOUND_MASK_VOLUME);
	return (0);
}

static int
qcom_pcm_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left,
    unsigned right)
{
	struct qcom_pcm_softc *sc = mix_getdevinfo(m);

	if (dev != SOUND_MIXER_VOLUME)
		return (-1);
	/* One gain for both sides. */
	sc->volume = MIN((left + right) / 2, 100);
	taskqueue_enqueue(sc->tq, &sc->vol_task);
	return (left | right << 8);
}

static kobj_method_t qcom_pcm_mixer_methods[] = {
	KOBJMETHOD(mixer_init,		qcom_pcm_mixer_init),
	KOBJMETHOD(mixer_set,		qcom_pcm_mixer_set),
	KOBJMETHOD_END
};
MIXER_DECLARE(qcom_pcm_mixer);

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
	TASK_INIT(&sc->sync_task, 0, qcom_pcm_sync_task, sc);
	TASK_INIT(&sc->done_task, 0, qcom_pcm_done_task, sc);
	TASK_INIT(&sc->vol_task, 0, qcom_pcm_vol_task, sc);
	sc->volume = 100;

	TIMEOUT_TASK_INIT(sc->tq, &sc->codec_task, 0, qcom_pcm_codec_task, sc);
	TASK_INIT(&sc->jack_task, 0, qcom_pcm_jack_task, sc);
	TIMEOUT_TASK_INIT(sc->tq, &sc->jack_poll_task, 0,
	    qcom_pcm_jack_poll_task, sc);
	sc->jack = -1;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "jack",
	    CTLFLAG_RD, &sc->jack, 0,
	    "Headphone jack: 1 in use, 0 empty, -1 not known yet");

	pcm_init(dev, sc);
	pcm_setflags(dev, pcm_getflags(dev) | SD_F_MPSAFE | SD_F_SOFTPCMVOL);
	error = mixer_init(dev, &qcom_pcm_mixer_class, sc);
	if (error == 0)
		error = pcm_addchan(dev, PCMDIR_PLAY, &qcom_pcm_chan_class, sc);
	if (error == 0)
		error = pcm_register(dev, "on the audio DSP");
	if (error != 0) {
		taskqueue_free(sc->tq);
		kmem_free(sc->ring, PCM_BUFSZ);
		return (error);
	}

	qcom_wcd938x_jack_notify(qcom_pcm_jack, sc);
	error = qcom_swr_wake_intr(dev, QCOM_SWR_TX, qcom_pcm_wake, sc,
	    &sc->wake_res, &sc->wake_cookie);
	if (error != 0)
		device_printf(dev, "no jack wake-up interrupt: %d\n", error);
	taskqueue_enqueue_timeout(sc->tq, &sc->codec_task, 0);
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
	/* pcm_unregister took the mixer down too. */
	if (sc->wake_res != NULL) {
		bus_teardown_intr(dev, sc->wake_res, sc->wake_cookie);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->wake_res);
	}
	qcom_wcd938x_jack_notify(NULL, NULL);
	/*
	 * The channel's free stopped the stream.  The codec bring-up re-arms
	 * itself, which draining its timeout prevents.
	 */
	taskqueue_drain_timeout(sc->tq, &sc->codec_task);
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
