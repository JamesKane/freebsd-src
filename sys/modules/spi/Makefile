SUBDIR = \
	../spigen \
	at45d \
	${_atopcase} \
	mx25l \
	spibus \
	${_zy7_spi} \
	
.if ${MACHINE_CPUARCH} == "i386" || ${MACHINE_CPUARCH} == "amd64"
_atopcase=atopcase
.endif

.if ${MACHINE_CPUARCH} == "aarch64"
_zy7_spi=zy7_spi
.endif

.include <bsd.subdir.mk>
