# $Id: Makefile 9227 2010-08-31 20:25:59Z fjoe $

.PATH: ${.CURDIR}/interface/vchiq_arm ${.CURDIR}/interface/vchiq_arm/freebsd ${.CURDIR}/interface/vcos/generic 

KMOD=		vchiq
SRCS=		vchiq_core.c vchiq_shim.c vchiq_util.c vchiq_kern_lib.c  \
	    	vcos_generic_event_flags.c vcos_logcat.c vcos_mem_from_malloc.c vcos_cmd.c
CFLAGS+=	-I${.CURDIR}/interface/vcos/freebsdkernel -I${.CURDIR}/interface/vchiq_arm  -DVCOS_VERIFY_BKPTS=1 -DUSE_VCHIQ_ARM -D__VCCOREVER__=0x04000000
CWARNFLAGS=

.include <bsd.kmod.mk>
.include <bsd.own.mk>
