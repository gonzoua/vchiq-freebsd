/*-
 * Copyright (c) 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
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

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_snd.h"
#endif

#include <dev/sound/pcm/sound.h>
#include <dev/sound/chip.h>

#include "mixer_if.h"

#include "interface/compat/vchi_bsd.h"
#include "interface/vchi/vchi.h"

#include "vc_vchi_audioserv_defs.h"

SND_DECLARE_FILE("vchiq_audio.c");

#define	VCHIQ_AUDIO_PACKET_SIZE	4000
#define	VCHIQ_AUDIO_BUFFER_SIZE	128000

#define	VCHIQ_AUDIO_MAX_VOLUME	
/* volume in terms of 0.01dB */
#define VCHIQ_AUDIO_VOLUME_MIN -10239
#define VCHIQ_AUDIO_VOLUME(db100) (uint32_t)(-((db100) << 8)/100)

/* dB levels with 5% volume step */
static int db_levels[] = {
	VCHIQ_AUDIO_VOLUME_MIN, -4605, -3794, -3218, -2772,
	-2407, -2099, -1832, -1597, -1386,
	-1195, -1021, -861, -713, -575,
	-446, -325, -210, -102, 0,
};

static uint32_t vchiq_audio_playfmt[] = {
	SND_FORMAT(AFMT_U8, 1, 0),
	SND_FORMAT(AFMT_U8, 2, 0),
	SND_FORMAT(AFMT_S8, 1, 0),
	SND_FORMAT(AFMT_S8, 2, 0),
	SND_FORMAT(AFMT_S16_LE, 1, 0),
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	SND_FORMAT(AFMT_U16_LE, 1, 0),
	SND_FORMAT(AFMT_U16_LE, 2, 0),
	0
};

static struct pcmchan_caps vchiq_audio_playcaps = {8000, 48000, vchiq_audio_playfmt, 0};

struct vchiq_audio_info;

#define	PLAYBACK_IDLE		0
#define	PLAYBACK_STARTING	1
#define	PLAYBACK_PLAYING	2
#define	PLAYBACK_STOPPING	3

struct vchiq_audio_chinfo {
	struct vchiq_audio_info *parent;
	struct pcm_channel *channel;
	struct snd_dbuf *buffer;
	uint32_t fmt, spd, blksz;

	uint32_t complete_pos;
	uint32_t free_buffer;
	uint32_t buffered_ptr;
	int playback_state;
};

struct vchiq_audio_info {
	unsigned int bufsz;
    	struct vchiq_audio_chinfo pch;
	struct mtx *lock;

	/* VCHI data */
	struct mtx vchi_lock;

	/* MSG reply */
	struct mtx msg_avail_lock;
	struct cv msg_avail_cv;
	uint32_t msg_result;

	VCHI_INSTANCE_T vchi_instance;
	VCHI_CONNECTION_T *vchi_connection;
	VCHI_SERVICE_HANDLE_T vchi_handle;

	struct mtx data_lock;
	struct cv data_cv;

	/* Unloadign module */
	int unloading;
};

#define vchiq_audio_lock(_ess) snd_mtxlock((_ess)->lock)
#define vchiq_audio_unlock(_ess) snd_mtxunlock((_ess)->lock)
#define vchiq_audio_lock_assert(_ess) snd_mtxassert((_ess)->lock)

#define VCHIQ_VCHI_LOCK(sc)		mtx_lock(&(sc)->vchi_lock)
#define VCHIQ_VCHI_UNLOCK(sc)		mtx_unlock(&(sc)->vchi_lock)

static void
vchiq_audio_callback(void *param, const VCHI_CALLBACK_REASON_T reason, void *msg_handle)
{
	struct vchiq_audio_info *sc = (struct vchiq_audio_info *)param;
	int32_t status;
	uint32_t msg_len;
	VC_AUDIO_MSG_T m;

	if (reason != VCHI_CALLBACK_MSG_AVAILABLE)
		return;

	status = vchi_msg_dequeue(sc->vchi_handle,
	    &m, sizeof m, &msg_len, VCHI_FLAGS_NONE);
	if (m.type == VC_AUDIO_MSG_TYPE_RESULT) {
		sc->msg_result = m.u.result.success;
		cv_signal(&sc->msg_avail_cv);
	} else if (m.type == VC_AUDIO_MSG_TYPE_COMPLETE) {
		struct vchiq_audio_chinfo *ch = m.u.complete.cookie;

		int count = m.u.complete.count & 0xffff;
		int perr = (m.u.complete.count & (1U << 30)) != 0;

		ch->complete_pos = (ch->complete_pos + count) % sndbuf_getsize(ch->buffer);
		ch->free_buffer += count;

		if (perr || ch->free_buffer >= VCHIQ_AUDIO_PACKET_SIZE) {
			chn_intr(ch->channel);
			cv_signal(&sc->data_cv);
		}
	} else
		printf("%s: unknown m.type: %d\n", __func__, m.type);
}

/* VCHIQ stuff */
static void
vchiq_audio_init(struct vchiq_audio_info *sc)
{
	int status;

	mtx_init(&sc->vchi_lock, "vchiq_audio", "vchi_lock", MTX_DEF);
	mtx_init(&sc->msg_avail_lock, "msg_avail_mtx", "msg_avail_mtx", MTX_DEF);
	cv_init(&sc->msg_avail_cv, "msg_avail_cv");
	mtx_init(&sc->data_lock, "data_mtx", "data_mtx", MTX_DEF);
	cv_init(&sc->data_cv, "data_cv");

	/* Initialize and create a VCHI connection */
	status = vchi_initialise(&sc->vchi_instance);
	if (status != 0) {
		printf("vchi_initialise failed: %d\n", status);
		return;
	}

	status = vchi_connect(NULL, 0, sc->vchi_instance);
	if (status != 0) {
		printf("vchi_connect failed: %d\n", status);
		return;
	}

	SERVICE_CREATION_T params = {
	    VCHI_VERSION_EX(VC_AUDIOSERV_VER, VC_AUDIOSERV_MIN_VER),
	    VC_AUDIO_SERVER_NAME,   /* 4cc service code */
	    sc->vchi_connection,    /* passed in fn pointers */
	    0,  /* rx fifo size */
	    0,  /* tx fifo size */
	    vchiq_audio_callback,    /* service callback */
	    sc,   /* service callback parameter */
	    1,
	    1,
	    0   /* want crc check on bulk transfers */
	};

	status = vchi_service_open(sc->vchi_instance, &params,
	    &sc->vchi_handle);

	if (status == 0)
		/* Finished with the service for now */
		vchi_service_release(sc->vchi_handle);
}

static void
vchiq_audio_release(struct vchiq_audio_info *sc)
{
	int success;

	mtx_destroy(&sc->vchi_lock);
	mtx_destroy(&sc->msg_avail_lock);
	cv_destroy(&sc->msg_avail_cv);
	mtx_destroy(&sc->data_lock);
	cv_destroy(&sc->data_cv);

	vchi_service_use(sc->vchi_handle);
	success = vchi_service_close(sc->vchi_handle);
	if (success != 0)
		printf("vchi_service_close failed: %d\n", success);

	vchi_disconnect(sc->vchi_instance);
}

static void
vchiq_audio_reset_channel(struct vchiq_audio_chinfo *ch)
{
	ch->free_buffer = VCHIQ_AUDIO_BUFFER_SIZE;
	ch->playback_state = 0;
	ch->buffered_ptr = 0;
	ch->complete_pos = 0;

	sndbuf_reset(ch->buffer);
}

static void
vchiq_audio_start(struct vchiq_audio_chinfo *ch)
{
	VC_AUDIO_MSG_T m;
	int32_t success;
	int ret;
	struct vchiq_audio_info *sc = ch->parent;

	VCHIQ_VCHI_LOCK(sc);
	vchi_service_use(sc->vchi_handle);

	vchiq_audio_reset_channel(ch);

	m.type = VC_AUDIO_MSG_TYPE_START;
	ret = vchi_msg_queue(sc->vchi_handle,
	    &m, sizeof m, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);

	if (ret != 0)
		printf("%s: vchi_msg_queue failed (err %d)\n", __func__, ret);

	vchi_service_release(sc->vchi_handle);
	VCHIQ_VCHI_UNLOCK(sc);

}

static void
vchiq_audio_stop(struct vchiq_audio_chinfo *ch)
{
	VC_AUDIO_MSG_T m;
	int32_t success;
	int ret;
	struct vchiq_audio_info *sc = ch->parent;

	VCHIQ_VCHI_LOCK(sc);
	vchi_service_use(sc->vchi_handle);

	m.type = VC_AUDIO_MSG_TYPE_STOP;
	m.u.stop.draining = 0;

	ret = vchi_msg_queue(sc->vchi_handle,
	    &m, sizeof m, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);

	if (ret != 0)
		printf("%s: vchi_msg_queue failed (err %d)\n", __func__, ret);

	vchi_service_release(sc->vchi_handle);
	VCHIQ_VCHI_UNLOCK(sc);
}

static void
vchiq_audio_open(struct vchiq_audio_info *sc)
{
	VC_AUDIO_MSG_T m;
	int32_t success;
	int ret;

	VCHIQ_VCHI_LOCK(sc);
	vchi_service_use(sc->vchi_handle);

	m.type = VC_AUDIO_MSG_TYPE_OPEN;
	ret = vchi_msg_queue(sc->vchi_handle,
	    &m, sizeof m, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);

	if (ret != 0)
		printf("%s: vchi_msg_queue failed (err %d)\n", __func__, ret);

	vchi_service_release(sc->vchi_handle);
	VCHIQ_VCHI_UNLOCK(sc);
}

static void
vchiq_audio_set_volume(struct vchiq_audio_info *sc, uint32_t volume)
{
	VC_AUDIO_MSG_T m;
	int32_t success;
	int ret, db;

	VCHIQ_VCHI_LOCK(sc);
	vchi_service_use(sc->vchi_handle);

	sc->msg_result = -1;

	m.type = VC_AUDIO_MSG_TYPE_CONTROL;
	m.u.control.dest = 0;
	if (volume > 99)
		volume = 99;
	db = db_levels[volume/5];
	m.u.control.volume = VCHIQ_AUDIO_VOLUME(db);

	ret = vchi_msg_queue(sc->vchi_handle,
	    &m, sizeof m, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);

	if (ret != 0)
		printf("%s: vchi_msg_queue failed (err %d)\n", __func__, ret);

	mtx_lock(&sc->msg_avail_lock);
	cv_wait_sig(&sc->msg_avail_cv, &sc->msg_avail_lock);
	if (sc->msg_result)
		printf("%s failed: %d\n", __func__, sc->msg_result);
	mtx_unlock(&sc->msg_avail_lock);

	vchi_service_release(sc->vchi_handle);
	VCHIQ_VCHI_UNLOCK(sc);
}

static void
vchiq_audio_update_params(struct vchiq_audio_info *sc, struct vchiq_audio_chinfo *ch)
{
	VC_AUDIO_MSG_T m;
	int32_t success;
	int ret;


	VCHIQ_VCHI_LOCK(sc);
	vchi_service_use(sc->vchi_handle);

	sc->msg_result = -1;

	m.type = VC_AUDIO_MSG_TYPE_CONFIG;
	m.u.config.channels = AFMT_CHANNEL(ch->fmt);
	m.u.config.samplerate = ch->spd;
	m.u.config.bps = AFMT_BIT(ch->fmt);

	ret = vchi_msg_queue(sc->vchi_handle,
	    &m, sizeof m, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);

	if (ret != 0)
		printf("%s: vchi_msg_queue failed (err %d)\n", __func__, ret);

	mtx_lock(&sc->msg_avail_lock);
	cv_wait_sig(&sc->msg_avail_cv, &sc->msg_avail_lock);
	if (sc->msg_result)
		printf("%s failed: %d\n", __func__, sc->msg_result);
	mtx_unlock(&sc->msg_avail_lock);

	vchi_service_release(sc->vchi_handle);
	VCHIQ_VCHI_UNLOCK(sc);
}

static __inline uint32_t
vchiq_unbuffered_bytes(struct vchiq_audio_chinfo *ch)
{
	uint32_t size, ready, readyptr, readyend;

	size = sndbuf_getsize(ch->buffer);
	readyptr = sndbuf_getreadyptr(ch->buffer);
	ready = sndbuf_getready(ch->buffer);

	// printf("size: %d, readyptr: %d, ready: %d\n", size, readyptr, ready);

	readyend = readyptr + ready;
	/* Normal case */
	if (ch->buffered_ptr >= readyptr) {
		if (readyend > ch->buffered_ptr)
			return readyend - ch->buffered_ptr;
		else
			return 0;
	}
	else { /* buffered_ptr overflow */
		if (readyend > ch->buffered_ptr + size)
			return readyend - ch->buffered_ptr - size;
		else
			return 0;
	}
}

static void
vchiq_audio_write_samples(struct vchiq_audio_chinfo *ch)
{
	struct vchiq_audio_info *sc = ch->parent;
	VC_AUDIO_MSG_T m;
	void *buf;
	uint32_t count, ready, readyptr, size;
	int32_t success;
	int ret;

	VCHIQ_VCHI_LOCK(sc);
	vchi_service_use(sc->vchi_handle);

	size = sndbuf_getsize(ch->buffer);
	count = vchiq_unbuffered_bytes(ch);
	buf = sndbuf_getbuf(ch->buffer) + ch->buffered_ptr;

	if (ch->buffered_ptr + count > size)
		count = size - ch->buffered_ptr;

	if (count < VCHIQ_AUDIO_PACKET_SIZE)
		goto done;

	count = min(count, ch->free_buffer);
	count -= count % VCHIQ_AUDIO_PACKET_SIZE;

	m.type = VC_AUDIO_MSG_TYPE_WRITE;
	m.u.write.count = count;
	m.u.write.max_packet = VCHIQ_AUDIO_PACKET_SIZE;
	m.u.write.callback = NULL;
	m.u.write.cookie = ch;
	if (buf)
		m.u.write.silence = 0;
	else
		m.u.write.silence = 1;

	ret = vchi_msg_queue(sc->vchi_handle,
	    &m, sizeof m, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);

	if (ret != 0)
		printf("%s: vchi_msg_queue failed (err %d)\n", __func__, ret);

	if (buf) {
		while (count > 0) {
			int bytes = MIN((int)m.u.write.max_packet, (int)count);
			ch->free_buffer -= bytes;
			ch->buffered_ptr += bytes;
			ch->buffered_ptr = ch->buffered_ptr % size;
			ret = vchi_msg_queue(sc->vchi_handle,
			    buf, bytes, VCHI_FLAGS_BLOCK_UNTIL_QUEUED, NULL);
			if (ret != 0)
				printf("%s: vchi_msg_queue failed: %d\n",
				    __func__, ret);
			buf = (char *)buf + bytes;
			count -= bytes;
		}
	}
done:

	vchi_service_release(sc->vchi_handle);
	VCHIQ_VCHI_UNLOCK(sc);
}

static void
vchiq_audio_worker(void *data)
{
	struct vchiq_audio_info *sc = (struct vchiq_audio_info *)data;
	struct vchiq_audio_chinfo *ch = &sc->pch;
	mtx_lock(&sc->data_lock);
	while(1) {

		if (sc->unloading)
			break;

		if ((ch->playback_state == PLAYBACK_PLAYING) &&
		    (vchiq_unbuffered_bytes(ch) >= VCHIQ_AUDIO_PACKET_SIZE)
		    && (ch->free_buffer >= VCHIQ_AUDIO_PACKET_SIZE)) {
			vchiq_audio_write_samples(ch);
		} else {
			if (ch->playback_state == PLAYBACK_STOPPING) {
				vchiq_audio_reset_channel(&sc->pch);
				ch->playback_state = PLAYBACK_IDLE;
			}

			cv_wait_sig(&sc->data_cv, &sc->data_lock);

			if (ch->playback_state == PLAYBACK_STARTING) {
				/* Give it initial kick */
				chn_intr(sc->pch.channel);
				ch->playback_state = PLAYBACK_PLAYING;
			}
		}
	}
	mtx_unlock(&sc->data_lock);

	kproc_exit(0);
}

static void
vchiq_audio_create_worker(struct vchiq_audio_info *sc)
{
	struct proc *newp;

	if (kproc_create(vchiq_audio_worker, (void*)sc, &newp, 0, 0,
	    "vchiq_audio_worker") != 0) {
		printf("failed to create vchiq_audio_worker\n");
	}
}

/* -------------------------------------------------------------------- */
/* channel interface for ESS18xx */
static void *
bcmchan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b, struct pcm_channel *c, int dir)
{
	struct vchiq_audio_info *sc = devinfo;
	struct vchiq_audio_chinfo *ch = &sc->pch;
	void *buffer;

	if (dir == PCMDIR_REC)
		return NULL;

	ch->parent = sc;
	ch->channel = c;
	ch->buffer = b;

	/* default values */
	ch->spd = 44100;
	ch->fmt = SND_FORMAT(AFMT_S16_LE, 2, 0);
	ch->blksz = VCHIQ_AUDIO_PACKET_SIZE;

	buffer = malloc(sc->bufsz, M_DEVBUF, M_WAITOK | M_ZERO);

	if (sndbuf_setup(ch->buffer, buffer, sc->bufsz) != 0) {
		free(buffer, M_DEVBUF);
		return NULL;
	}

	vchiq_audio_update_params(sc, ch);

	return ch;
}

static int
bcmchan_free(kobj_t obj, void *data)
{
	struct vchiq_audio_chinfo *ch = data;
	void *buffer;

	buffer = sndbuf_getbuf(ch->buffer);
	if (buffer)
		free(buffer, M_DEVBUF);

	return (0);
}

static int
bcmchan_setformat(kobj_t obj, void *data, uint32_t format)
{
	struct vchiq_audio_chinfo *ch = data;
	struct vchiq_audio_info *sc = ch->parent;

	vchiq_audio_lock(sc);

	ch->fmt = format;
	vchiq_audio_update_params(sc, ch);

	vchiq_audio_unlock(sc);

	return 0;
}

static uint32_t
bcmchan_setspeed(kobj_t obj, void *data, uint32_t speed)
{
	struct vchiq_audio_chinfo *ch = data;
	struct vchiq_audio_info *sc = ch->parent;

	vchiq_audio_lock(sc);

	ch->spd = speed;
	vchiq_audio_update_params(sc, ch);

	vchiq_audio_unlock(sc);

	return ch->spd;
}

static uint32_t
bcmchan_setblocksize(kobj_t obj, void *data, uint32_t blocksize)
{
	struct vchiq_audio_chinfo *ch = data;

	return ch->blksz;
}

static int
bcmchan_trigger(kobj_t obj, void *data, int go)
{
	struct vchiq_audio_chinfo *ch = data;
	struct vchiq_audio_info *sc = ch->parent;
	uint8_t *buf;
	uint32_t size;

	if (!PCMTRIG_COMMON(go))
		return (0);

	vchiq_audio_lock(sc);

	switch (go) {
	case PCMTRIG_START:
		vchiq_audio_start(ch);
		ch->playback_state = PLAYBACK_STARTING;
		/* wakeup worker thread */
		cv_signal(&sc->data_cv);
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		ch->playback_state = 1;
		vchiq_audio_stop(ch);
		break;

	default:
		break;
	}

	vchiq_audio_unlock(sc);
	return 0;
}

static uint32_t
bcmchan_getptr(kobj_t obj, void *data)
{
	struct vchiq_audio_chinfo *ch = data;
	struct vchiq_audio_info *sc = ch->parent;
	uint32_t ret;

	vchiq_audio_lock(sc);

	ret = ch->complete_pos - (ch->complete_pos % VCHIQ_AUDIO_PACKET_SIZE);

	vchiq_audio_unlock(sc);

	return ret;
}

static struct pcmchan_caps *
bcmchan_getcaps(kobj_t obj, void *data)
{
	struct vchiq_audio_chinfo *ch = data;

	return &vchiq_audio_playcaps;
}

static kobj_method_t bcmchan_methods[] = {
    	KOBJMETHOD(channel_init,		bcmchan_init),
    	KOBJMETHOD(channel_free,		bcmchan_free),
    	KOBJMETHOD(channel_setformat,		bcmchan_setformat),
    	KOBJMETHOD(channel_setspeed,		bcmchan_setspeed),
    	KOBJMETHOD(channel_setblocksize,	bcmchan_setblocksize),
    	KOBJMETHOD(channel_trigger,		bcmchan_trigger),
    	KOBJMETHOD(channel_getptr,		bcmchan_getptr),
    	KOBJMETHOD(channel_getcaps,		bcmchan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(bcmchan);

/************************************************************/

static int
bcmmix_init(struct snd_mixer *m)
{
    	struct vchiq_audio_info *sc = mix_getdevinfo(m);

	mix_setdevs(m, SOUND_MASK_VOLUME);

	return 0;
}

static int
bcmmix_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
    	struct vchiq_audio_info *sc = mix_getdevinfo(m);

	switch (dev) {
	case SOUND_MIXER_VOLUME:
		vchiq_audio_set_volume(sc, left);
		break;

	default:
		break;
	}

    	return left | (left << 8);
}

static kobj_method_t bcmmixer_methods[] = {
    	KOBJMETHOD(mixer_init,		bcmmix_init),
    	KOBJMETHOD(mixer_set,		bcmmix_set),
	KOBJMETHOD_END
};

MIXER_DECLARE(bcmmixer);

static void
vchiq_audio_identify(driver_t *driver, device_t parent)
{

	BUS_ADD_CHILD(parent, 0, "pcm", 0);
}

static int
vchiq_audio_probe(device_t dev)
{

	device_set_desc(dev, "VCHQI audio");
	return (BUS_PROBE_DEFAULT);
}

static int
vchiq_audio_attach(device_t dev)
{
    	struct vchiq_audio_info *sc;
    	char status[SND_STATUSLEN];
	u_int16_t ddma;

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);

	sc->bufsz = VCHIQ_AUDIO_BUFFER_SIZE;

	sc->lock = snd_mtxcreate(device_get_nameunit(dev), "vchiq_audio softc");

	vchiq_audio_init(sc);
	vchiq_audio_open(sc);
	vchiq_audio_set_volume(sc, 75);

    	if (mixer_init(dev, &bcmmixer_class, sc))
		goto no;

    	snprintf(status, SND_STATUSLEN, "at VCHIQ");

    	if (pcm_register(dev, sc, 1, 1))
		goto no;

	pcm_addchan(dev, PCMDIR_PLAY, &bcmchan_class, sc);
	pcm_setstatus(dev, status);

	vchiq_audio_reset_channel(&sc->pch);
	vchiq_audio_create_worker(sc);

    	return 0;

no:
    	return ENXIO;
}

static int
vchiq_audio_detach(device_t dev)
{
	int r;
	struct vchiq_audio_info *sc;
	sc = pcm_getdevinfo(dev);

	/* Stop worker thread */
	sc->unloading = 1;
	cv_signal(&sc->data_cv);

	r = pcm_unregister(dev);
	if (r)
		return r;

	vchiq_audio_release(sc);

	if (sc->lock) {
		snd_mtxfree(sc->lock);
		sc->lock = NULL;
	}

    	free(sc, M_DEVBUF);

	return 0;
}

static device_method_t vchiq_audio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	vchiq_audio_identify),
	DEVMETHOD(device_probe,		vchiq_audio_probe),
	DEVMETHOD(device_attach,	vchiq_audio_attach),
	DEVMETHOD(device_detach,	vchiq_audio_detach),

	{ 0, 0 }
};

static driver_t vchiq_audio_driver = {
	"pcm",
	vchiq_audio_methods,
	PCM_SOFTC_SIZE,
};

DRIVER_MODULE(vchiq_audio, vchiq, vchiq_audio_driver, pcm_devclass, 0, 0);
MODULE_DEPEND(vchiq_audio, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);
MODULE_DEPEND(vchiq_audio, vchiq, 1, 1, 1);
MODULE_VERSION(vchiq_audio, 1);
