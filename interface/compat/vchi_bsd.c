/*-
 * Copyright (c) 2010 Max Khon <fjoe@freebsd.org>
 * All rights reserved.
 *
 * This software was developed by Max Khon under sponsorship from
 * the FreeBSD Foundation and Ethon Technologies GmbH.
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
 *
 * $Id: bsd-compat.c 9253 2010-09-02 10:12:09Z fjoe $
 */

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/firmware.h>
#include <sys/param.h>
#include <sys/proc.h>
#include <sys/syscallsubr.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <machine/stdarg.h>

#include <interface/compat/vchi_bsd.h>

MALLOC_DEFINE(M_VCHI, "VCHI", "VCHI");

/*
 * Timer API
 */
static void
run_timer(void *arg)
{
	struct timer_list *t = (struct timer_list *) arg;
	void (*function)(unsigned long);

	mtx_lock_spin(&t->mtx);
	if (callout_pending(&t->callout)) {
		/* callout was reset */
		mtx_unlock_spin(&t->mtx);
		return;
	}
	if (!callout_active(&t->callout)) {
		/* callout was stopped */
		mtx_unlock_spin(&t->mtx);
		return;
	}
	callout_deactivate(&t->callout);

	function = t->function;
	mtx_unlock_spin(&t->mtx);

	function(t->data);
}

void
init_timer(struct timer_list *t)
{
	mtx_init(&t->mtx, "dahdi timer lock", NULL, MTX_SPIN);
	callout_init(&t->callout, CALLOUT_MPSAFE);
	t->expires = 0;
	/*
	 * function and data are not initialized intentionally:
	 * they are not initialized by Linux implementation too
	 */
}

void
setup_timer(struct timer_list *t, void (*function)(unsigned long), unsigned long data)
{
	t->function = function;
	t->data = data;
	init_timer(t);
}

void
mod_timer(struct timer_list *t, unsigned long expires)
{
	mtx_lock_spin(&t->mtx);
	callout_reset(&t->callout, expires - jiffies, run_timer, t);
	mtx_unlock_spin(&t->mtx);
}

void
add_timer(struct timer_list *t)
{
	mod_timer(t, t->expires);
}

int
del_timer_sync(struct timer_list *t)
{
	mtx_lock_spin(&t->mtx);
	callout_stop(&t->callout);
	mtx_unlock_spin(&t->mtx);

	mtx_destroy(&t->mtx);
	return 0;
}

int
del_timer(struct timer_list *t)
{
	del_timer_sync(t);
	return 0;
}

/*
 * Completion API
 */
void
init_completion(struct completion *c)
{
	cv_init(&c->cv, "VCHI completion cv");
	mtx_init(&c->lock, "VCHI completion lock", "condvar", MTX_DEF);
	c->done = 0;
}

void
destroy_completion(struct completion *c)
{
	cv_destroy(&c->cv);
	mtx_destroy(&c->lock);
}

void
wait_for_completion(struct completion *c)
{
	mtx_lock(&c->lock);
	if (!c->done)
		cv_wait(&c->cv, &c->lock);
	c->done--;
	mtx_unlock(&c->lock);
}

int
try_wait_for_completion(struct completion *c)
{
	int res = 0;

	mtx_lock(&c->lock);
	if (!c->done)
		c->done--;
	else
		res = 1;
	mtx_unlock(&c->lock);
	return res == 0;
}


int
wait_for_completion_timeout(struct completion *c, unsigned long timeout)
{
	int res = 0;

	mtx_lock(&c->lock);
	if (!c->done)
		res = cv_timedwait(&c->cv, &c->lock, timeout);
	if (res == 0)
		c->done--;
	mtx_unlock(&c->lock);
	return res == 0;
}

int
wait_for_completion_interruptible_timeout(struct completion *c, unsigned long timeout)
{
	int res = 0;

	mtx_lock(&c->lock);
	if (!c->done)
		res = cv_timedwait_sig(&c->cv, &c->lock, timeout);
	if (res == 0)
		c->done--;
	mtx_unlock(&c->lock);
	return res == 0;
}

int
wait_for_completion_interruptible(struct completion *c)
{
	int res = 0;

	mtx_lock(&c->lock);
	if (!c->done)
		res = cv_wait_sig(&c->cv, &c->lock);
	if (res == 0)
		c->done--;
	mtx_unlock(&c->lock);
	return res == 0;
}

int
wait_for_completion_killable(struct completion *c)
{
	int res = 0;

	mtx_lock(&c->lock);
	if (!c->done)
		res = cv_wait_sig(&c->cv, &c->lock);
	/* TODO: check actual signals here ? */
	if (res == 0)
		c->done--;
	mtx_unlock(&c->lock);
	return res == 0;
}

void
complete(struct completion *c)
{
	mtx_lock(&c->lock);
	c->done++;
	cv_signal(&c->cv);
	mtx_unlock(&c->lock);
}

void
complete_all(struct completion *c)
{
	mtx_lock(&c->lock);
	c->done++;
	cv_broadcast(&c->cv);
	mtx_unlock(&c->lock);
}

/*
 * Semaphore API
 */

void sema_sysinit(void *arg)
{
	struct semaphore *s = arg;

	printf("sema_sysinit\n");
	_sema_init(s, 1);
}

void
_sema_init(struct semaphore *s, int value)
{
	bzero(s, sizeof(*s));
	mtx_init(&s->mtx, "sema lock", "VCHIQ sepmaphore backing lock",
		MTX_DEF | MTX_NOWITNESS | MTX_QUIET);
	cv_init(&s->cv, "sema cv");
	s->value = value;
}

void
_sema_destroy(struct semaphore *s)
{
	mtx_destroy(&s->mtx);
	cv_destroy(&s->cv);
}

void
down(struct semaphore *s)
{

	mtx_lock(&s->mtx);
	while (s->value == 0) {
		s->waiters++;
		cv_wait(&s->cv, &s->mtx);
		s->waiters--;
	}

	s->value--;
	mtx_unlock(&s->mtx);
}

int
down_interruptible(struct semaphore *s)
{
	int ret ;

	ret = 0;

	mtx_lock(&s->mtx);

	while (s->value == 0) {
		s->waiters++;
		ret = cv_wait_sig(&s->cv, &s->mtx);
		s->waiters--;

		if (ret == EINTR) {
			mtx_unlock(&s->mtx);
			return (-EINTR);
		}

		if (ret == ERESTART)
			continue;
	}

	s->value--;
	mtx_unlock(&s->mtx);

	return (0);
}

int
down_trylock(struct semaphore *s)
{
	int ret;

	ret = 0;

	mtx_lock(&s->mtx);

	if (s->value > 0) {
		/* Success. */
		s->value--;
		ret = 0;
	} else {
		ret = -EAGAIN;
	}

	mtx_unlock(&s->mtx);

	return (ret);
}

void
up(struct semaphore *s)
{
	mtx_lock(&s->mtx);
	s->value++;
	if (s->waiters && s->value > 0)
		cv_signal(&s->cv);

	mtx_unlock(&s->mtx);
}

/*
 * Logging API
 */
void
rlprintf(int pps, const char *fmt, ...)
{
	va_list ap;
	static struct timeval last_printf;
	static int count;

	if (ppsratecheck(&last_printf, &count, pps)) {
		va_start(ap, fmt);
		vprintf(fmt, ap);
		va_end(ap);
	}
}

void
device_rlprintf(int pps, device_t dev, const char *fmt, ...)
{
	va_list ap;
	static struct timeval last_printf;
	static int count;

	if (ppsratecheck(&last_printf, &count, pps)) {
		va_start(ap, fmt);
		device_print_prettyname(dev);
		vprintf(fmt, ap);
		va_end(ap);
	}
}

/*
 * Signals API
 */

void
flush_signals(struct proc *p)
{
	printf("Implement ME: %s\n", __func__);
}

int
fatal_signal_pending(struct proc *p)
{
	printf("Implement ME: %s\n", __func__);
	return (0);
}

/*
 * kthread API
 */

/*
 *  This is a hack to avoid memory leak
 */
#define MAX_THREAD_DATA_SLOTS	32
static int thread_data_slot = 0;

struct thread_data {
	void *data;
	int (*threadfn)(void *);
};

static struct thread_data thread_slots[MAX_THREAD_DATA_SLOTS];

static void 
kthread_wrapper(void *data)
{
	struct thread_data *slot;

	slot = data;
	slot->threadfn(slot->data);
}

struct proc *
kthread_create(int (*threadfn)(void *data),
	void *data,
	const char namefmt[], ...)
{
	struct proc *newp;
	va_list ap;
	char name[MAXCOMLEN+1];
	struct thread_data *slot;

	if (thread_data_slot >= MAX_THREAD_DATA_SLOTS) {
		printf("kthread_create: out of thread data slots\n");
		return (NULL);
	}

	slot = &thread_slots[thread_data_slot];
	slot->data = data;
	slot->threadfn = threadfn;

	va_start(ap, namefmt);
	vsnprintf(name, sizeof(name), namefmt, ap);
	va_end(ap);
	
	newp = NULL;
	if (kproc_create(kthread_wrapper, (void*)slot, &newp, 0, 0,
	    "%s", name) != 0) {
		/* Just to be sure */
		newp = NULL;
	}
	else
		thread_data_slot++;

	return newp;
}

void
set_user_nice(struct proc *p, int nice)
{
	/* NOOP */
}

void
wake_up_process(struct proc *p)
{
	/* NOOP */
}
