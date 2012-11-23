/*
 * Copyright (c) 2010-2011 Broadcom. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*=============================================================================
VideoCore OS Abstraction Layer - Linux kernel (partial) implementation.
=============================================================================*/

/* Do not include this file directly - instead include it via vcos.h */

/** @file
  *
  * Linux kernel (partial) implementation of VCOS.
  *
  */

#ifndef VCOS_PLATFORM_H
#define VCOS_PLATFORM_H

#include <sys/types.h>
#include <sys/param.h>
#include <sys/lock.h>
#include <sys/sema.h>
#include <sys/mutex.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/malloc.h>

#define VCOS_HAVE_RTOS         1
#define VCOS_HAVE_SEMAPHORE    1
#define VCOS_HAVE_EVENT        1
#define VCOS_HAVE_QUEUE        0
#define VCOS_HAVE_LEGACY_ISR   0
#define VCOS_HAVE_TIMER        1
#define VCOS_HAVE_CANCELLATION_SAFE_TIMER 0
#define VCOS_HAVE_MEMPOOL      0
#define VCOS_HAVE_ISR          0
#define VCOS_HAVE_ATOMIC_FLAGS 1
#define VCOS_HAVE_BLOCK_POOL   0
#define VCOS_HAVE_ONCE         1
#define VCOS_HAVE_FILE         0
#define VCOS_HAVE_USER_BUF     0
#define VCOS_HAVE_CFG          1
#define VCOS_HAVE_SPINLOCK     0
#define VCOS_HAVE_CMD          1
#define VCOS_HAVE_EVENT_FLAGS  1

/* Exclude many VCOS classes which don't have predicates */
#define VCOS_TLS_H
#define VCOS_NAMED_MUTEX_H
#define VCOS_REENTRANT_MUTEX_H
#define VCOS_NAMED_SEMAPHORE_H
#define VCOS_QUICKSLOW_MUTEX_H
/*#define VCOS_INIT_H */
/*#define VCOS_MEM_H */
/*#define VCOS_STRING_H */

/*
 * Use hand-made semaphore until FreeBSD adds sema_wait_sig
 */
struct xsema
{
	struct mtx	mtx;
	struct cv	cv;
	int		value;
	int		waiters;
};

/*
 * Use hand-made semaphore until FreeBSD adds sema_wait_sig
 */
struct event
{
	struct mtx	mtx;
	struct cv	cv;
	int		value;
};

typedef struct xsema           VCOS_SEMAPHORE_T;
typedef struct event           VCOS_EVENT_T;
typedef struct mtx            VCOS_MUTEX_T;
typedef volatile int          VCOS_ONCE_T;

typedef unsigned int          VCOS_UNSIGNED;
typedef unsigned int          VCOS_OPTION;
typedef volatile uint32_t     VCOS_ATOMIC_FLAGS_T;

typedef struct
{
    struct callout         callout;
    void                   *context;
    void                  (*expiration_routine)(void *context);

} VCOS_TIMER_T;

typedef struct VCOS_LLTHREAD_T
{
   struct proc *proc;             /**< The thread itself */
   VCOS_SEMAPHORE_T suspend;     /**< For support event groups and similar - a per thread semaphore */
} VCOS_LLTHREAD_T;

typedef enum
{
    VCOS_O_RDONLY   = 00000000,
    VCOS_O_WRONLY   = 00000001,
    VCOS_O_RDWR     = 00000002,
    VCOS_O_TRUNC    = 00001000,
} VCOS_FILE_FLAGS_T;

typedef struct file *VCOS_FILE_T;

#define VCOS_SUSPEND          -1
#define VCOS_NO_SUSPEND       0

#define VCOS_START 1
#define VCOS_NO_START 0

#define VCOS_THREAD_PRI_MIN   -20
#define VCOS_THREAD_PRI_MAX   19

#define VCOS_THREAD_PRI_INCREASE -1
#define VCOS_THREAD_PRI_HIGHEST  VCOS_THREAD_PRI_MIN
#define VCOS_THREAD_PRI_LOWEST   VCOS_THREAD_PRI_MAX
#define VCOS_THREAD_PRI_NORMAL ((VCOS_THREAD_PRI_MAX+VCOS_THREAD_PRI_MIN)/2)
#define VCOS_THREAD_PRI_ABOVE_NORMAL (VCOS_THREAD_PRI_NORMAL + VCOS_THREAD_PRI_INCREASE)
#define VCOS_THREAD_PRI_REALTIME VCOS_THREAD_PRI_HIGHEST

#define _VCOS_AFFINITY_DEFAULT 0
#define _VCOS_AFFINITY_CPU0 0
#define _VCOS_AFFINITY_CPU1 0
#define _VCOS_AFFINITY_MASK 0
#define VCOS_CAN_SET_STACK_ADDR  0

#define VCOS_TICKS_PER_SECOND HZ

#include "interface/vcos/generic/vcos_generic_event_flags.h"
#include "interface/vcos/generic/vcos_mem_from_malloc.h"
#include "interface/vcos/generic/vcos_joinable_thread_from_plain.h"

/***********************************************************
 *
 * Memory allcoation
 *
 ***********************************************************/

#define  _vcos_platform_malloc   vcos_platform_malloc
#define  _vcos_platform_free     vcos_platform_free

void *vcos_platform_malloc( VCOS_UNSIGNED required_size );
void  vcos_platform_free( void *ptr );

#if defined(VCOS_INLINE_BODIES)

#undef VCOS_ASSERT_LOGGING_DISABLE
#define VCOS_ASSERT_LOGGING_DISABLE 1

/***********************************************************
 *
 * Counted Semaphores
 *
 ***********************************************************/

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_semaphore_wait(VCOS_SEMAPHORE_T *sem) {
   int ret;

   mtx_lock(&sem->mtx);
   do {
      while (sem->value == 0) {
         sem->waiters++;
         ret = cv_wait_sig(&sem->cv, &sem->mtx);
         sem->waiters--;
      }

      if (ret == EINTR) {
         mtx_unlock(&sem->mtx);
         return VCOS_EINTR;
      }

   } while  (ret == ERESTART);

   sem->value--;
   mtx_unlock(&sem->mtx);

   return VCOS_SUCCESS;
}

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_semaphore_trywait(VCOS_SEMAPHORE_T *sem) {
   int ret;

   mtx_lock(&sem->mtx);

   if (sem->value > 0) {
      /* Success. */
      sem->value--;
      ret = VCOS_SUCCESS;
   } else {
      ret = VCOS_EAGAIN;
   }

   mtx_unlock(&sem->mtx);
   return (ret);
}

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_semaphore_create(VCOS_SEMAPHORE_T *sem,
                                    const char *name,
                                    VCOS_UNSIGNED initial_count) {
   bzero(sem, sizeof(*sem));
   mtx_init(&sem->mtx, name, "sema backing lock",
      MTX_DEF | MTX_NOWITNESS | MTX_QUIET);
   cv_init(&sem->cv, name);
   sem->value = initial_count;

   return VCOS_SUCCESS;
}

VCOS_INLINE_IMPL
void vcos_semaphore_delete(VCOS_SEMAPHORE_T *sem) {
   mtx_destroy(&sem->mtx);
   cv_destroy(&sem->cv);
}

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_semaphore_post(VCOS_SEMAPHORE_T *sem) {
   mtx_lock(&sem->mtx);
   sem->value++;
   if (sem->waiters && sem->value > 0)
      cv_signal(&sem->cv);

   mtx_unlock(&sem->mtx);
   return VCOS_SUCCESS;
}

/***********************************************************
 *
 * Threads
 *
 ***********************************************************/

#include "vcos_thread_map.h"

VCOS_INLINE_IMPL
VCOS_LLTHREAD_T *vcos_llthread_current(void) {
        return &vcos_kthread_current()->thread;
}

VCOS_INLINE_IMPL
void vcos_llthread_resume(VCOS_LLTHREAD_T *thread) {
   vcos_assert(0);
}

VCOS_INLINE_IMPL
void vcos_sleep(uint32_t ms) {
   pause("vcos_sleep", ms);
}

VCOS_INLINE_IMPL
void vcos_thread_set_priority(VCOS_THREAD_T *thread, VCOS_UNSIGNED p) {
   /* not implemented */
}
VCOS_INLINE_IMPL
VCOS_UNSIGNED vcos_thread_get_priority(VCOS_THREAD_T *thread) {
   /* not implemented */
   return 0;
}

/***********************************************************
 *
 * Miscellaneous
 *
 ***********************************************************/

VCOS_INLINE_IMPL
int vcos_strcasecmp(const char *s1, const char *s2) {
   return strcasecmp(s1,s2);
}


/***********************************************************
 *
 * Mutexes
 *
 ***********************************************************/

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_mutex_create(VCOS_MUTEX_T *m, const char *name) {
   mtx_init(m, name, NULL, MTX_DEF);
   return VCOS_SUCCESS;
}

VCOS_INLINE_IMPL
void vcos_mutex_delete(VCOS_MUTEX_T *m) {
   mtx_destroy(m);
}

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_mutex_lock(VCOS_MUTEX_T *m) {
   mtx_lock(m);

   return VCOS_SUCCESS;
}

VCOS_INLINE_IMPL
void vcos_mutex_unlock(VCOS_MUTEX_T *m) {
   mtx_unlock(m);
}

VCOS_INLINE_IMPL
int vcos_mutex_is_locked(VCOS_MUTEX_T *m) {
   if (mtx_trylock(m) != 0) {
      mtx_unlock(m);
      /* it wasn't locked */
      return 0;
   }
   return 1; /* it was locked */
}

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_mutex_trylock(VCOS_MUTEX_T *m) {
   if (mtx_trylock(m) != 0)
      return VCOS_SUCCESS;
   else
      return VCOS_EAGAIN;
}

/* For supporting event groups - per thread semaphore */
VCOS_INLINE_IMPL
void _vcos_thread_sem_wait(void) {
   VCOS_THREAD_T *t = vcos_thread_current();
   vcos_semaphore_wait(&t->suspend);
}

VCOS_INLINE_IMPL
void _vcos_thread_sem_post(VCOS_THREAD_T *target) {
   vcos_semaphore_post(&target->suspend);
}

/***********************************************************
 *
 * Events
 *
 ***********************************************************/

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_event_create(VCOS_EVENT_T *event, const char *debug_name)
{
   bzero(event, sizeof(*event));
   mtx_init(&event->mtx, debug_name, "event backing lock",
      MTX_DEF | MTX_NOWITNESS | MTX_QUIET);
   cv_init(&event->cv, debug_name);
   event->value = 0;
   return VCOS_SUCCESS;
}

VCOS_INLINE_IMPL
void vcos_event_signal(VCOS_EVENT_T *event)
{
   mtx_lock(&event->mtx);
   event->value = 1;
   cv_signal(&event->cv);
   mtx_unlock(&event->mtx);
}

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_event_wait(VCOS_EVENT_T *event)
{
   int ret = 0;

   mtx_lock(&event->mtx);
   if (event->value == 0) {
      do {
         ret = cv_wait_sig(&event->cv, &event->mtx);
      } while (ret == ERESTART);
   }

   if (ret != EINTR)
      event->value = 0;

   mtx_unlock(&event->mtx);

   if (ret == EINTR)
      return VCOS_EINTR;
   else
      return VCOS_SUCCESS;
}

VCOS_INLINE_DECL
VCOS_STATUS_T vcos_event_try(VCOS_EVENT_T *event)
{
   int ret = 0;

   mtx_lock(&event->mtx);
   if (event->value == 0)
      ret = VCOS_EAGAIN;
   else {
      ret = VCOS_SUCCESS;
      event->value = 0;
   }
   mtx_unlock(&event->mtx);
  
   return ret;
}

VCOS_INLINE_IMPL
void vcos_event_delete(VCOS_EVENT_T *event)
{
   mtx_destroy(&event->mtx);
   cv_destroy(&event->cv);
}

/***********************************************************
 *
 * Timers
 *
 ***********************************************************/

VCOS_INLINE_DECL
void vcos_timer_freebsd_func(void *data)
{
    VCOS_TIMER_T    *vcos_timer = (VCOS_TIMER_T *)data;

    vcos_timer->expiration_routine( vcos_timer->context );
}

VCOS_INLINE_DECL
VCOS_STATUS_T vcos_timer_create(VCOS_TIMER_T *timer,
                                const char *name,
                                void (*expiration_routine)(void *context),
                                void *context) {
   callout_init(&timer->callout, CALLOUT_MPSAFE);

   timer->context = context;
   timer->expiration_routine = expiration_routine;

    return VCOS_SUCCESS;
}

VCOS_INLINE_IMPL
void vcos_timer_set(VCOS_TIMER_T *timer, VCOS_UNSIGNED delay_ms) {
   /* XXXBSD: tvtohz? */
   callout_reset(&timer->callout, hz*1000/delay_ms, vcos_timer_freebsd_func, timer);
}

VCOS_INLINE_IMPL
void vcos_timer_cancel(VCOS_TIMER_T *timer) {
   callout_stop(&timer->callout);
}

VCOS_INLINE_IMPL
void vcos_timer_reset(VCOS_TIMER_T *timer, VCOS_UNSIGNED delay_ms) {
   callout_drain(&timer->callout);
   /* XXXBSD: tvtohz? */
   callout_reset(&timer->callout, hz*1000/delay_ms, vcos_timer_freebsd_func, timer);
}

VCOS_INLINE_IMPL
void vcos_timer_delete(VCOS_TIMER_T *timer) {
    callout_drain(&timer->callout);
    timer->context = NULL;
    timer->expiration_routine = NULL;
    return;
}

VCOS_INLINE_IMPL
VCOS_UNSIGNED vcos_process_id_current(void) {
   return (VCOS_UNSIGNED)curthread->td_proc->p_pid;
}


#if 0
XXXBSD: not yet
VCOS_INLINE_IMPL
int vcos_in_interrupt(void) {
   return in_interrupt();
}
#endif

/***********************************************************
 *
 * Atomic flags
 *
 ***********************************************************/

VCOS_INLINE_IMPL
VCOS_STATUS_T vcos_atomic_flags_create(VCOS_ATOMIC_FLAGS_T *atomic_flags)
{
   atomic_set_32(atomic_flags, 0);
   return VCOS_SUCCESS;
}

VCOS_INLINE_IMPL
void vcos_atomic_flags_or(VCOS_ATOMIC_FLAGS_T *atomic_flags, uint32_t flags)
{
   uint32_t old_flags;
   uint32_t new_flags;
   do {
      old_flags = *atomic_flags;
      new_flags = old_flags | flags;
   } while (atomic_cmpset_rel_32(atomic_flags, old_flags, new_flags) == 0);
}

VCOS_INLINE_IMPL
uint32_t vcos_atomic_flags_get_and_clear(VCOS_ATOMIC_FLAGS_T *atomic_flags)
{
   return atomic_readandclear_32(atomic_flags);
}

VCOS_INLINE_IMPL
void vcos_atomic_flags_delete(VCOS_ATOMIC_FLAGS_T *atomic_flags)
{
}

#undef VCOS_ASSERT_LOGGING_DISABLE
#define VCOS_ASSERT_LOGGING_DISABLE 0

#endif /* VCOS_INLINE_BODIES */

VCOS_INLINE_DECL void _vcos_thread_sem_wait(void);
VCOS_INLINE_DECL void _vcos_thread_sem_post(VCOS_THREAD_T *);

/***********************************************************
 *
 * Misc
 *
 ***********************************************************/
VCOS_INLINE_DECL char *vcos_strdup(const char *str);

/***********************************************************
 *
 * Logging
 *
 ***********************************************************/

VCOSPRE_ const char * VCOSPOST_ _vcos_log_level(void);
#define _VCOS_LOG_LEVEL() _vcos_log_level()

#define  vcos_log_platform_init()               _vcos_log_platform_init()
#define  vcos_log_platform_register(category)   _vcos_log_platform_register(category)
#define  vcos_log_platform_unregister(category) _vcos_log_platform_unregister(category)

struct VCOS_LOG_CAT_T;  /* Forward declaration since vcos_logging.h hasn't been included yet */

void _vcos_log_platform_init(void);
void _vcos_log_platform_register(struct VCOS_LOG_CAT_T *category);
void _vcos_log_platform_unregister(struct VCOS_LOG_CAT_T *category);

/***********************************************************
 *
 * Memory barriers
 *
 ***********************************************************/

#define vcos_wmb(x) do {\
	__asm __volatile ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0) : "memory"); \
} while (0);

#define vcos_rmb(x) do {\
	__asm __volatile ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0) : "memory"); \
} while (0);

#include "interface/vcos/generic/vcos_common.h"
/*#include "interface/vcos/generic/vcos_generic_quickslow_mutex.h" */

#endif /* VCOS_PLATFORM_H */

