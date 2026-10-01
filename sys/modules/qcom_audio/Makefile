.PATH: ${SRCTOP}/sys/dev/qcom_audio

KMOD=	qcom_audio
SRCS=	qcom_apm.c qcom_apps_smmu.c qcom_gpr.c qcom_lpass_macro.c qcom_prm.c qcom_swr.c
SRCS+=	acpi_if.h bus_if.h device_if.h opt_acpi.h

.include <bsd.kmod.mk>
