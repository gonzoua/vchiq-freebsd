# $Id$

KMOD=		vchiq_audio

SRCS=		vchiq_audio.c
SRCS+=	    	device_if.h bus_if.h channel_if.h mixer_if.h

CFLAGS+=	-I${.CURDIR}/../vchiq  -DUSE_VCHIQ_ARM -D__VCCOREVER__=0x04000000
CWARNFLAGS=

.include <bsd.kmod.mk>
