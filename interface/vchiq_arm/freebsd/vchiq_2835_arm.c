/*
 * Copyright (c) 2010-2011 Broadcom Corporation. All rights reserved.
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

#define TOTAL_SLOTS (VCHIQ_SLOT_ZERO_SLOTS + 2 * 32)

#define VCHIQ_DOORBELL_IRQ IRQ_ARM_DOORBELL_0
#define VCHIQ_ARM_ADDRESS(x) ((void *)__virt_to_bus((unsigned)x))

#include "vchiq_arm.h"
#include "vchiq_2835.h"

#include <machine/bus.h>
#include <arm/broadcom/bcm2835/bcm2835_mbox.h>

#define MAX_FRAGMENTS (VCHIQ_NUM_CURRENT_BULKS * 2)

#define VCOS_LOG_CATEGORY (&vchiq_arm_log_category)

#define PAGE_ALIGN(addr) round_page(addr)

static char *g_slot_mem;
static int g_slot_mem_size;
vm_paddr_t g_slot_phys;
static FRAGMENTS_T *g_fragments_base;
static FRAGMENTS_T *g_free_fragments;
struct sema g_free_fragments_sema;


/* BSD DMA */
bus_dma_tag_t dma_tag;
bus_dmamap_t dma_map;

/* XXXBSD: destroy on unload? */
static struct mtx g_free_fragments_mutex;
MTX_SYSINIT(g_free_fragments_mutex, &g_free_fragments_mutex, "vcos_frags", MTX_DEF);

static void
vchiq_doorbell_irq(int irq, void *dev_id);

#ifdef notyet
static int
create_pagelist(char *buf, size_t count, unsigned short type,
	struct task_struct *task, PAGELIST_T ** ppagelist);

static void
free_pagelist(PAGELIST_T *pagelist, int actual);
#endif

static void
vchiq_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int err)
{
	bus_addr_t *addr;

	if (err)
		return;

	addr = (bus_addr_t*)arg;
	*addr = segs[0].ds_addr;
}

int
vchiq_platform_vcos_init(void)
{
	return (vcos_init() == VCOS_SUCCESS) ? 0 : EINVAL;
}

int
vchiq_platform_init(VCHIQ_STATE_T *state)
{
	VCHIQ_SLOT_ZERO_T *vchiq_slot_zero;
	int frag_mem_size;
	int err;
	int i;

	/* Allocate space for the channels in coherent memory */
	g_slot_mem_size = PAGE_ALIGN(TOTAL_SLOTS * VCHIQ_SLOT_SIZE);
	frag_mem_size = PAGE_ALIGN(sizeof(FRAGMENTS_T) * MAX_FRAGMENTS);

	err = bus_dma_tag_create(
	    NULL,
	    PAGE_SIZE, 0,	       /* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,    /* lowaddr */
	    BUS_SPACE_MAXADDR,	  /* highaddr */
	    NULL, NULL,		 /* filter, filterarg */
	    g_slot_mem_size + frag_mem_size, 1,		/* maxsize, nsegments */
	    g_slot_mem_size + frag_mem_size, 0,		/* maxsegsize, flags */
	    NULL, NULL,		 /* lockfunc, lockarg */
	    &dma_tag);

	err = bus_dmamem_alloc(dma_tag, (void **)&g_slot_mem,
	    0, &dma_map);
	if (err) {
		vcos_log_error("Unable to allocate channel memory");
		err = ENOMEM;
		goto failed_alloc;
	}

	err = bus_dmamap_load(dma_tag, dma_map, g_slot_mem,
	    g_slot_mem_size + frag_mem_size, vchiq_dmamap_cb,
	    &g_slot_phys, BUS_DMA_NOWAIT);

	if (err) {
		vcos_log_error("cannot load DMA map\n");
		goto failed_load;
	}

	vcos_assert(((int)g_slot_mem & (PAGE_SIZE - 1)) == 0);

	vchiq_slot_zero = vchiq_init_slots(g_slot_mem, g_slot_mem_size);
	if (!vchiq_slot_zero)
	{
	   err = EINVAL;
	   goto failed_init_slots;
	}

	vchiq_slot_zero->platform_data[VCHIQ_PLATFORM_FRAGMENTS_OFFSET_IDX] = (int)g_slot_phys + g_slot_mem_size;
	vchiq_slot_zero->platform_data[VCHIQ_PLATFORM_FRAGMENTS_COUNT_IDX] = MAX_FRAGMENTS;

	g_fragments_base = (FRAGMENTS_T *)(g_slot_mem + g_slot_mem_size);
	g_slot_mem_size += frag_mem_size;

	g_free_fragments = g_fragments_base;
	for (i = 0; i < (MAX_FRAGMENTS - 1); i++) {
		*(FRAGMENTS_T **) & g_fragments_base[i] =
			&g_fragments_base[i + 1];
	}
	*(FRAGMENTS_T **) & g_fragments_base[i] = NULL;
	sema_init(&g_free_fragments_sema, MAX_FRAGMENTS, "Fragments semaphore");

	if (vchiq_init_state(state, vchiq_slot_zero, 0/*slave*/) !=
		VCHIQ_SUCCESS)
	{
		err = EINVAL;
		goto failed_vchiq_init;
	}

#ifdef notyet
	err = request_irq(VCHIQ_DOORBELL_IRQ, vchiq_doorbell_irq,
		IRQF_SAMPLE_RANDOM | IRQF_IRQPOLL, "VCHIQ doorbell",
		state);
	if (err < 0)
	{
		printk( KERN_ERR "%s: failed to register irq=%d err=%d\n", __func__,
			VCHIQ_DOORBELL_IRQ, err );
		goto failed_request_irq;
	}

	/* Send the base address of the slots to VideoCore */

	__asm __volatile("dsb");
#endif

	bcm_mbox_write(BCM2835_MBOX_CHAN_VCHIQ, (unsigned int)g_slot_phys);

	vcos_log_info("vchiq_init - done (slots %x, phys %x)",
		(unsigned int)vchiq_slot_zero, g_slot_phys);

	return 0;

failed_request_irq:
failed_vchiq_init:
failed_init_slots:
failed_load:
	bus_dmamap_unload(dma_tag, dma_map);
failed_alloc:
	bus_dmamap_destroy(dma_tag, dma_map);
	bus_dma_tag_destroy(dma_tag);

	return err;
}

void 
vchiq_platform_exit(VCHIQ_STATE_T *state)
{
#ifdef notyet
	free_irq(VCHIQ_DOORBELL_IRQ, state);
#endif

	bus_dmamap_unload(dma_tag, dma_map);
	bus_dmamap_destroy(dma_tag, dma_map);
	bus_dma_tag_destroy(dma_tag);
}

void
remote_event_signal(REMOTE_EVENT_T *event)
{
	event->fired = 1;

	/* The test on the next line also ensures the write on the previous line
		has completed */

	if (event->armed) {
#ifdef notyet
		/* trigger vc interrupt */
		__asm __volatile("dsb");

		writel(0, __io_address(ARM_0_BELL2));
#endif
	}
}

int
vchiq_copy_from_user(void *dst, const void *src, int size)
{

	bcopy(src, dst, size);
	return 0;
}

VCHIQ_STATUS_T
vchiq_prepare_bulk_data(VCHIQ_BULK_T *bulk, VCHI_MEM_HANDLE_T memhandle,
	void *offset, int size, int dir)
{
#ifdef notyet
	PAGELIST_T *pagelist;
	int ret;

	vcos_assert(memhandle == VCHI_MEM_HANDLE_INVALID);

	ret = create_pagelist((char *)offset, size,
			(dir == VCHIQ_BULK_RECEIVE)
			? PAGELIST_READ
			: PAGELIST_WRITE,
			current,
			&pagelist);
	if (ret != 0)
		return VCHIQ_ERROR;

	bulk->handle = memhandle;
	bulk->data = VCHIQ_ARM_ADDRESS(pagelist);

	/* Store the pagelist address in remote_data, which isn't used by the
	   slave. */
	bulk->remote_data = pagelist;
#endif

	return VCHIQ_SUCCESS;
}

void
vchiq_complete_bulk(VCHIQ_BULK_T *bulk)
{
#ifdef notyet
	free_pagelist((PAGELIST_T *)bulk->remote_data, bulk->actual);
#endif
}

void
vchiq_transfer_bulk(VCHIQ_BULK_T *bulk)
{
	/*
	 * This should only be called on the master (VideoCore) side, but
	 * provide an implementation to avoid the need for ifdefery.
	 */
	vcos_assert(!"This code should not be called by the ARM on BCM2835");
}

void
vchiq_dump_platform_state(void *dump_context)
{
        char buf[80];
        int len;
        len = vcos_snprintf(buf, sizeof(buf),
                "  Platform: 2835 (VC master)");
        vchiq_dump(dump_context, buf, len + 1);
}

VCHIQ_STATUS_T
vchiq_platform_suspend(VCHIQ_STATE_T *state)
{
   vcos_unused(state);
   vcos_assert_msg(0, "Suspend/resume not supported");
   return VCHIQ_ERROR;
}

VCHIQ_STATUS_T
vchiq_platform_resume(VCHIQ_STATE_T *state)
{
   vcos_unused(state);
   vcos_assert_msg(0, "Suspend/resume not supported");
   return VCHIQ_ERROR;
}

void
vchiq_platform_paused(VCHIQ_STATE_T *state)
{
   vcos_unused(state);
   vcos_assert_msg(0, "Suspend/resume not supported");
}

void
vchiq_platform_resumed(VCHIQ_STATE_T *state)
{
   vcos_unused(state);
   vcos_assert_msg(0, "Suspend/resume not supported");
}

int
vchiq_platform_videocore_wanted(VCHIQ_STATE_T* state)
{
   vcos_unused(state);
   return 1; // autosuspend not supported - videocore always wanted
}

#if VCOS_HAVE_TIMER
int
vchiq_platform_use_suspend_timer(void)
{
   return 0;
}
#endif
void
vchiq_dump_platform_use_state(VCHIQ_STATE_T *state)
{
   vcos_unused(state);
}

VCHIQ_STATUS_T
vchiq_platform_init_state(VCHIQ_STATE_T *state)
{
   vcos_unused(state);
   return VCHIQ_SUCCESS;
}

VCHIQ_ARM_STATE_T*
vchiq_platform_get_arm_state(VCHIQ_STATE_T *state)
{
   vcos_unused(state);
   return NULL;
}

/*
 * Local functions
 */

static void
vchiq_doorbell_irq(int irq, void *dev_id)
{
	VCHIQ_STATE_T *state = dev_id;
	unsigned int status;

	/* Read (and clear) the doorbell */
#ifdef notyet
	status = readl(__io_address(ARM_0_BELL0));
#endif

	if (status & 0x4) {  /* Was the doorbell rung? */
		remote_event_pollall(state);
	}

}

#ifdef notyet
/* There is a potential problem with partial cache lines (pages?)
	at the ends of the block when reading. If the CPU accessed anything in
	the same line (page?) then it may have pulled old data into the cache,
	obscuring the new data underneath. We can solve this by transferring the
	partial cache lines separately, and allowing the ARM to copy into the
	cached area.

	N.B. This implementation plays slightly fast and loose with the Linux
	driver programming rules, e.g. its use of __virt_to_bus instead of
	dma_map_single, but it isn't a multi-platform driver and it benefits
	from increased speed as a result.
 */

static int
create_pagelist(char *buf, size_t count, unsigned short type,
	struct task_struct *task, PAGELIST_T ** ppagelist)
{
	PAGELIST_T *pagelist;
	struct page **pages;
	struct page *page;
	unsigned long *addrs;
	unsigned int num_pages, offset, i;
	char *addr, *base_addr, *next_addr;
	int run, addridx, actual_pages;

	offset = (unsigned int)buf & (PAGE_SIZE - 1);
	num_pages = (count + offset + PAGE_SIZE - 1) / PAGE_SIZE;

	*ppagelist = NULL;

	/* Allocate enough storage to hold the page pointers and the page list */
	pagelist = (PAGELIST_T *) kmalloc(sizeof(PAGELIST_T) +
		(num_pages * sizeof(unsigned long)) +
		(num_pages * sizeof(pages[0])),
		GFP_KERNEL);

	vcos_log_trace("create_pagelist - %x", (unsigned int)pagelist);
	if (!pagelist)
		return ENOMEM;

	addrs = pagelist->addrs;
	pages = (struct page **)(addrs + num_pages);

	down_read(&task->mm->mmap_sem);
	actual_pages = get_user_pages(task, task->mm,
		(unsigned long)buf & ~(PAGE_SIZE - 1), num_pages,
		(type == PAGELIST_READ) /*Write */ , 0 /*Force */ ,
		pages, NULL /*vmas */ );
	up_read(&task->mm->mmap_sem);

        if (actual_pages != num_pages)
	{
		for (i = 0; i < actual_pages; i++) {
			page_cache_release(pages[i]);
		}
		kfree(pagelist);
		return EINVAL;
	}

	pagelist->length = count;
	pagelist->type = type;
	pagelist->offset = offset;

	/* Group the pages into runs of contiguous pages */

	base_addr = VCHIQ_ARM_ADDRESS(page_address(pages[0]));
	next_addr = base_addr + PAGE_SIZE;
	addridx = 0;
	run = 0;

	for (i = 1; i < num_pages; i++) {
		addr = VCHIQ_ARM_ADDRESS(page_address(pages[i]));
		if ((addr == next_addr) && (run < (PAGE_SIZE - 1))) {
			next_addr += PAGE_SIZE;
			run++;
		} else {
			addrs[addridx] = (unsigned long)base_addr + run;
			addridx++;
			base_addr = addr;
			next_addr = addr + PAGE_SIZE;
			run = 0;
		}
	}

	addrs[addridx] = (unsigned long)base_addr + run;
	addridx++;

	/* Partial cache lines (fragments) require special measures */
	if ((type == PAGELIST_READ) &&
		((pagelist->offset & (CACHE_LINE_SIZE - 1)) ||
		((pagelist->offset + pagelist->length) & (CACHE_LINE_SIZE - 1)))) {
		FRAGMENTS_T *fragments;

		if (wait(&g_free_fragments_sema) != 0) {
			kfree(pagelist);
			return EINTR;
		}

		vcos_assert(g_free_fragments != NULL);

		down(&g_free_fragments_mutex);
		fragments = (FRAGMENTS_T *) g_free_fragments;
		vcos_assert(fragments != NULL);
		g_free_fragments = *(FRAGMENTS_T **) g_free_fragments;
		up(&g_free_fragments_mutex);
		pagelist->type =
			 PAGELIST_READ_WITH_FRAGMENTS + (fragments -
							 g_fragments_base);
	}

	for (page = virt_to_page(pagelist);
		page <= virt_to_page(addrs + num_pages - 1); page++) {
		flush_dcache_page(page);
	}

	*ppagelist = pagelist;

	return 0;
}

static void
free_pagelist(PAGELIST_T *pagelist, int actual)
{
	struct page **pages;
	unsigned int num_pages, i;

	vcos_log_trace("free_pagelist - %x, %d", (unsigned int)pagelist, actual);

	num_pages =
		 (pagelist->length + pagelist->offset + PAGE_SIZE - 1) / PAGE_SIZE;

	pages = (struct page **)(pagelist->addrs + num_pages);

	/* Deal with any partial cache lines (fragments) */
	if (pagelist->type >= PAGELIST_READ_WITH_FRAGMENTS) {
		FRAGMENTS_T *fragments =
			 g_fragments_base + (pagelist->type -
					PAGELIST_READ_WITH_FRAGMENTS);
		int head_bytes, tail_bytes;

		if (actual >= 0)
		{
			if ((head_bytes = (CACHE_LINE_SIZE - pagelist->offset) & (CACHE_LINE_SIZE - 1)) != 0) {
				if (head_bytes > actual)
					head_bytes = actual;

				memcpy((char *)page_address(pages[0]) +
						 pagelist->offset, fragments->headbuf,
						 head_bytes);
			}
			if ((head_bytes < actual) &&
				(tail_bytes =
				(pagelist->offset + actual) & (CACHE_LINE_SIZE -
										1)) != 0) {
				memcpy((char *)page_address(pages[num_pages - 1]) +
						 ((pagelist->offset + actual) & (PAGE_SIZE -
									1) & ~(CACHE_LINE_SIZE - 1)),
						 fragments->tailbuf, tail_bytes);
			}
		}

		down(&g_free_fragments_mutex);
		*(FRAGMENTS_T **) fragments = g_free_fragments;
		g_free_fragments = fragments;
		up(&g_free_fragments_mutex);
		post(&g_free_fragments_sema);
	}

	for (i = 0; i < num_pages; i++) {
		if (pagelist->type != PAGELIST_WRITE)
			set_page_dirty(pages[i]);
		page_cache_release(pages[i]);
	}

	kfree(pagelist);
}
#endif

