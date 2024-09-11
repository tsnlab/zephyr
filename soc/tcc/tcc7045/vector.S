.section	.vector	,"ax"

#define REMAP_MICOM_BASE	0x1B936014
#define COLD_RESET_BASE	0x1B93617C

.global	_vector	
.global ARM_Reserved
.global _start
_start:

# POOKY 20240911
_vector:
    ldr pc, =reset_handler
#    ldr pc, =ARM_ExceptUndefInstrHndlr
#    ldr pc, =ARM_ExceptSwiHndlr
#    ldr pc, =ARM_ExceptPrefetchAbortHndlr
#    ldr pc, =ARM_ExceptDataAbortHndlr
#    ldr pc, =ARM_Reserved
#    ldr pc, =ARM_ExceptIrqHndlr
#    ldr pc, =ARM_ExceptFiqHndlr

	
reset_handler:
	ldr pc, = reset

ARM_Reserved:
	wfi
	b ARM_Reserved

	.end
