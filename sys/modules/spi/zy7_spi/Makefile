.PATH:	${SRCTOP}/sys/arm/xilinx

KMOD=	zy7_spi
SRCS=	zy7_spi.c

SRCS+=	\
	acpi_if.h \
	bus_if.h \
	device_if.h \
	opt_acpi.h \
	opt_platform.h \
	spibus_if.h \

.if !empty(OPT_FDT)
SRCS+=	ofw_bus_if.h
.endif

.include <bsd.kmod.mk>
