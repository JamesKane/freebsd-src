.PATH: ${SRCTOP}/sys/dev/gpio

KMOD=	cdns_gpio
SRCS=	cdns_gpio.c
SRCS+=	acpi_if.h bus_if.h device_if.h gpio_if.h gpiobus_if.h ofw_bus_if.h
SRCS+=	opt_acpi.h opt_platform.h

.include <bsd.kmod.mk>
