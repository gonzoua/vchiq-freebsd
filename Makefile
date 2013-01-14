# $Id$

.PATH: ${.CURDIR}/interface/vchiq_arm ${.CURDIR}/interface/compat ${.CURDIR}/interface/vcos/generic  ${.CURDIR}/interface/vchi

KMOD=		vchiq

SRCS=		vchiq_core.c vchiq_shim.c vchiq_util.c vchiq_kern_lib.c vchiq_2835_arm.c 
SRCS+=		vchiq_arm.c vchiq_connected.c 
SRCS+=	    	vchi_bsd.c vchiq_kmod.c
SRCS+=	    	device_if.h bus_if.h ofw_bus_if.h

CFLAGS+=	-I${.CURDIR}/interface  -DVCOS_VERIFY_BKPTS=1 -DUSE_VCHIQ_ARM -D__VCCOREVER__=0x04000000
CWARNFLAGS=

.include <bsd.kmod.mk>
.include <bsd.own.mk>
