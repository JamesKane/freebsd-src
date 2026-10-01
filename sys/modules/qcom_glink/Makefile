.PATH: ${SRCTOP}/sys/dev/qcom_glink

KMOD=	qcom_glink
SRCS=	qcom_glink.c qcom_ipcc.c qcom_smem.c
SRCS+=	acpi_if.h bus_if.h device_if.h opt_acpi.h

.include <bsd.kmod.mk>
