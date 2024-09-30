/* Init Stack Pointer */
#define	USERMODE		0x10
#define	FIQMODE			0x11
#define	IRQMODE			0x12
#define	SVCMODE			0x13
#define	ABORTMODE		0x17
#define	UNDEFMODE		0x1b
#define SYS_MODE       	0x1f
#define	MODEMASK		0x1f
#define	NOINT			0xc0

.equ	SRAM_OFFSET,	    (0x00000000)

.equ	StackEnd,		    (STACK_BASE_ADDR - 0x680)
.equ	SvcStackSt,		    (STACK_BASE_ADDR - 0x380)
.equ	UserStackSt,		(STACK_BASE_ADDR - 0x370)
.equ	UndefStackSt,		(STACK_BASE_ADDR - 0x368)
.equ	AbortStackSt,		(STACK_BASE_ADDR - 0x360)
.equ	IrqStackSt,		    (STACK_BASE_ADDR - 0x60)
.equ	FIQStackSt,		    (STACK_BASE_ADDR - 0x0)

.text
.global topst_reset

topst_reset:

/****** Don't touch below 10 words !!!! ******/
	b init				/* 0x00 : initial branch instruction */
.word 0x4855434D		/* 0x04 : MCUH */
.word __TEXT_START__ 	/* 0x08 : micom f/w load base */

// Don't delete below 3 words.
// This is to apply the 2oo3 voting method.
.word (0x56010000) 
.word (0x56010000)
.word (0x56010000)

.word 0x00000000 		/* 0x10 : reserved */
.word 0x00000000 		/* 0x14 : reserved */
.word 0x00000000 		/* 0x18 : reserved */
.word 0x00000000 		/* 0x1c : reserved */
/****** Don't touch - end !!!! ******/

init:
	/* check R5 Core Enter to Serial Flash*/
	mrc p15, 0, r0, c0, c0, 0
	ldr r1, =0xFFFF
	and r0, r0, r1
	ldr r1, =0xC153        /* 0xC15x      check it revision number */
	cmp r0, r1
	bne wfi_loop

	/* set the cpu to SVC32 mode and IRQ & FIQ disable */
	mrs	r0, cpsr
	bic	r0, r0, #0x3f
	orr	r0, r0, #0xd3
	msr	cpsr, r0

    /* PRELOAD_ONRAM Start */
    bl PRELOAD_loadOnRam
    dsb
        isb
        bl PRELOAD_JOB
    /* PRELOAD_ONRAM End */

 /* Stack Setup for each ARM Excution Mode */
stack_setup:
	mrs	r0, cpsr
	bic	r0, r0, #MODEMASK

	orr	r1, r0, #FIQMODE|NOINT
	msr	cpsr, r1
	ldr	sp, =FIQStackSt

	orr	r1, r0, #UNDEFMODE|NOINT
	msr	cpsr, r1
	ldr	sp, =UndefStackSt

	orr	r1, r0, #ABORTMODE|NOINT
	msr	cpsr, r1
	ldr	sp, =AbortStackSt

	orr	r1, r0, #IRQMODE|NOINT
	msr	cpsr_cxsf, r1
	ldr	sp, =IrqStackSt

	orr	r1, r0, #SYS_MODE|NOINT
	msr	cpsr_cxsf, r1
	ldr	sp, =UserStackSt

	orr	r1, r0, #SVCMODE|NOINT
	msr	cpsr_cxsf, r1
	ldr sp, =SvcStackSt

	/* Cache, and MPU Setup */

	/* Disable MPU, I-Cache, D-CAche */
	mrc p15, 0, r0, c1, c0, 0       // Read CP15 SCTLR
	bic r0, r0, #(0x1<<0)           // Disable MPU
	bic r0, r0, #(0x1<<2)           // Disable D-Cache
	bic r0, r0, #(0x1<<12)          // Disable I-Cache
	dsb
	mcr p15, 0, r0, c1, c0, 0       // Write CP15 SCTLR
	isb

	mrc p15, 0, r0, c1, c0, 1       // Read CP15, ACTLR
	orr r0, r0, #(0x1 << 17)        // Clear RSDIS bit 17 to enable return stack
	orr r0, r0, #(0x1 << 16)        // Clear BP bit 15 and set BP bit 16:
	bic r0, r0, #(0x1 << 15)        // Branch always not taken and history table updates disabled
	bic r0, r0, #(0x1 << 5)         // Generate abort on parity errors, with [5:3]=b 000
	bic r0, r0, #(0x1 << 4)
	bic r0, r0, #(0x1 << 3)
	mcr p15, 0, r0, c1, c0, 1       // Write ACTLR
	dsb                             // Complete all outstanding explicit memory operations

	/* Invalidate Caches */
	mov r0, #0
	dsb
	mcr p15, 0, r0, c7, c5, 0       // Invalidate I-Cache All
	mcr p15, 0, r0, c15, c5, 0      // Incalidate D-Cache All
	isb

	/* The default mpu table is not working in SRAM */
	/* bl MPU_Init */   /* MPU Initialize */

branch_prefiction:
	/* Enable Branch Predictions */
	mrc p15, 0, r0, c1, c0, 1       // Read ACTLR
	bic r0, r0, #(0x1<<17)          // Clear RSDIS bit 17 to enable returen stack
	bic r0, r0, #(0x1<<16)          // Clear BP bit 15 and BP bit 16
	bic r0, r0, #(0x1<<15)          // Normal operation BP is taken from the global history table
	mcr p15, 0, r0, c1, c0, 1       // Write ACTLR

	/* Enable I-Cache , D-Cache */
	mrc p15, 0, r1, c1, c0, 0

	orr r1, r1, #(0x1<<0)           // Enable MPU

	orr r1, r1, #(0x1<<2)           // Enable D-Cache
	orr r1, r1, #(0x1<<12)          // Enable I-Cache
	DSB
	mcr p15, 0, r1, c1, c0, 0       // Enable Cache
	ISB

        bl InitCVar

        /* Set Vector Table to high Vector */
        mrc p15, 0, r0, c1, c0, 0
        bic	r0, r0, #(0x1<<13)
        mcr p15, 0, r0, c1, c0, 0

    /* PFLASH_STANDALONE Start */
/* Copy Vector Table to SRAM_0 */
vector_copy:
	ldr r0, =__VECTOR_START_LOAD
	ldr r1, =__VECTOR_START__
	ldr r2, =__VECTOR_END__
vcopy_loop:
	cmp r1, r2
	ldrlt r3, [r0], #4
	strlt r3, [r1], #4
	blt vcopy_loop

    /* PFLASH_STANDALONE End */

	bl      vfp_init
	bl      PMIO_EarlyWakeUp 
	b       BSP_PreInit
	b       .

/*
 * Initialize RW/ZI Area
 */
InitCVar:
	ldr	r0, =(__DATA_START_LOAD) @ load address
	ldr	r1, =(__DATA_START__)
	ldr	r2, =(__DATA_END__)

	cmp	r0, r1
	beq	5f
4:	/* Copy RW Region */
	ldm	r0!, {r4-r11}
	stm	r1!, {r4-r11}
	cmp	r1, r2
	bne	4b
5:	/* Clear BSS Region */
	ldr	r0, =(__BSS_START__)
	ldr	r1, =(__BSS_END__)
	cmp	r0, r1
	moveq	pc, lr

	mov	r4, #0
	mov	r5, #0
	mov	r6, #0
	mov	r7, #0
	mov	r8, #0
	mov	r9, #0
	mov	r10, #0
	mov	r11, #0
6:
	stm	r0!, {r4-r11}
	cmp	r0, r1
	bne	6b

	mov	pc, lr

wfi_loop:
    wfi
    b wfi_loop

.word 0x564C4C50 		/* PLLV */
.word 0x2 /* MICOM_MAIN_PLL_SEL*/  	/* reference from pll_config.h header */
.word 0x00014dc4 /*MICOM_MAIN_PLL_VAL */ 	/* reference from pll_config.h header */
.word 0x00000000 		/* 0x2C : reserved */

.word __TEXT_START__    /* 0x30 : image length for secure boot */
.word __RODATA_END__
.word __DATA_START__
.word __DATA_END__

.bss
.align 2
stack_limit:
	.skip (0x20)
mon_stack_top:

.end
