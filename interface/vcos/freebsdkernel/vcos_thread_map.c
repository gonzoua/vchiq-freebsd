/*****************************************************************************
* Copyright 2009 - 2010 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

/** Support to allow VCOS thread-related functions to be called from
  * threads that were not created by VCOS.
  */

#include "vcos_thread_map.h"
#include "interface/vcos/vcos_logging.h"

/* XXXBSD: Naive implementation, should be replaced with rpoper one */ 
#define MAX_MAP_SIZE 256
struct map_entry {
   struct thread     *kernel_thread;
   VCOS_THREAD_T     *vcos_thread;
};

struct map_entry thread_map[MAX_MAP_SIZE];
static int map_entries;

/**
   @fn uint32_t vcos_add_thread(THREAD_MAP_T *vcos_thread);
*/
uint32_t vcos_add_thread(VCOS_THREAD_T *vcos_thread)
{
   struct thread *td = curthread;
   int i;

   for (i = 0; i < map_entries; i++) {
      if (thread_map[i].kernel_thread == curthread) {
         thread_map[i].vcos_thread = vcos_thread;
         return(0);
      }
   }

   if (map_entries == MAX_MAP_SIZE) {
      printf("%s: map_entries reached maximu\n", __func__);
   }

   thread_map[map_entries].kernel_thread = td;
   thread_map[map_entries].vcos_thread = vcos_thread;
   map_entries++;

   return(0);
}


/**
   @fn uint32_t vcos_remove_thread(struct task_struct * thread_id);
*/
uint32_t vcos_remove_thread(struct thread *td)
{
   /* Remove thread_id -> VCOS_THREAD_T relationship */
   int i,j;

   if( td != curthread )
      panic("vcos_remove_thread: thread_id != current");

   for (i = 0; i < map_entries; i++) {
      if (thread_map[i].kernel_thread == td) {
         for (j = i; j < map_entries - 1; j++) {
            thread_map[j].kernel_thread = thread_map[j+1].kernel_thread;
            thread_map[j].vcos_thread = thread_map[j+1].vcos_thread;
         }
         thread_map[map_entries - 1].kernel_thread = NULL;
         thread_map[map_entries - 1].vcos_thread = NULL;
         map_entries--;
         break;
      }
   }

   return(0);
}

VCOS_THREAD_T *vcos_kthread_current(void)
{
   int i;

   for (i = 0; i < map_entries; i++) {
      if (thread_map[i].kernel_thread == curthread)
         return thread_map[i].vcos_thread;
   }

   return (void *)NULL;
}
