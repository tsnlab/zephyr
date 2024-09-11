/*********************************************************************************************************
*                                          PUBLIC FUNCTIONS
*********************************************************************************************************/
                                                                @ External references.

                                                                @ Functions declared in this file.
                                                                @ Functions related to exception handling.
    .global  RTOS_ARM_ExceptUndefInstrHndlr
    .global  RTOS_ARM_ExceptSwiHndlr
    .global  RTOS_ARM_ExceptPrefetchAbortHndlr
    .global  RTOS_ARM_ExceptDataAbortHndlr
    .global  RTOS_ARM_ExceptIrqHndlr
    .global  RTOS_ARM_ExceptFiqHndlr

    .global  RTOS_ARM_DRegCntGet
    .global  CPU_WaitForEvent
    .global  CPU_DSB

    .global  CPU_SR_Save
    .global  CPU_SR_Restore

    .global  BSP_UndefAbortExceptionHook


/*********************************************************************************************************
*                                               EQUATES
*********************************************************************************************************/

#define RTOS_ARM_CONTROL_INT_DIS        0xC0                     // Disable both FIQ and IRQ.
#define RTOS_ARM_CONTROL_FIQ_DIS        0x40                     // Disable FIQ.
#define RTOS_ARM_CONTROL_IRQ_DIS        0x80                     // Disable IRQ.
#define RTOS_ARM_CONTROL_THUMB          0x20                     // Set THUMB mode.
#define RTOS_ARM_CONTROL_ARM            0x00                     // Set ARM mode.

#define RTOS_ARM_MODE_MASK              0x1F
#define RTOS_ARM_MODE_USR               0x10
#define RTOS_ARM_MODE_FIQ               0x11
#define RTOS_ARM_MODE_IRQ               0x12
#define RTOS_ARM_MODE_SVC               0x13
#define RTOS_ARM_MODE_ABT               0x17
#define RTOS_ARM_MODE_UND               0x1B
#define RTOS_ARM_MODE_SYS               0x1F

#define RTOS_ARM_EXCEPT_RESET           0x00
#define RTOS_ARM_EXCEPT_UNDEF_INSTR     0x01
#define RTOS_ARM_EXCEPT_SWI             0x02
#define RTOS_ARM_EXCEPT_PREFETCH_ABORT  0x03
#define RTOS_ARM_EXCEPT_DATA_ABORT      0x04
#define RTOS_ARM_EXCEPT_ADDR_ABORT      0x05
#define RTOS_ARM_EXCEPT_IRQ             0x06
#define RTOS_ARM_EXCEPT_FIQ             0x07

#define RTOS_ARM_FPEXC_EN               0x40000000


/*********************************************************************************************************
*                                     CODE GENERATION DIRECTIVES
*********************************************************************************************************/

//    RSEG CODE:CODE:NOROOT(2)
//    AAPCS INTERWORK
//    PRESERVE8
//    REQUIRE8
//    CODE32

/*********************************************************************************************************
*                               UNDEFINED INSTRUCTION EXCEPTION HANDLER
*
* Register Usage:  R0     Exception Type
*                  R1
*                  R2     Return PC
*********************************************************************************************************/

RTOS_ARM_ExceptUndefInstrHndlr:
                                                                @ LR offset to return from this exception:  0.
    STMFD   SP!, {R0-R3}                                        @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #RTOS_ARM_EXCEPT_UNDEF_INSTR                  @ Set exception ID to RTOS_ARM_EXCEPT_UNDEF_INSTR.
    BL BSP_UndefAbortExceptionHook
    B            RTOS_ARM_ExceptHndlr                         @ Branch to global exception handler.


/*********************************************************************************************************
*                                SOFTWARE INTERRUPT EXCEPTION HANDLER
*
* Register Usage:  R0     Exception Type
*                  R1
*                  R2     Return PC
*********************************************************************************************************/

RTOS_ARM_ExceptSwiHndlr:
    STMFD   SP!, {R0-R3}                                        @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #RTOS_ARM_EXCEPT_SWI                          @ Set exception ID to RTOS_ARM_EXCEPT_SWI.
    B            RTOS_ARM_ExceptHndlr                         @ Branch to global exception handler.


/*********************************************************************************************************
*                                  PREFETCH ABORT EXCEPTION HANDLER
*
* Register Usage:  R0     Exception Type
*                  R1
*                  R2     Return PC
*********************************************************************************************************/

RTOS_ARM_ExceptPrefetchAbortHndlr:
    SUB     LR, LR, #4                                          @ LR offset to return from this exception: -4.
    STMFD   SP!, {R0-R3}                                        @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #RTOS_ARM_EXCEPT_PREFETCH_ABORT               @ Set exception ID to RTOS_ARM_EXCEPT_PREFETCH_ABORT.
    BL BSP_UndefAbortExceptionHook
    B            RTOS_ARM_ExceptHndlr                         @ Branch to global exception handler.


/*********************************************************************************************************
*                                    DATA ABORT EXCEPTION HANDLER
*
* Register Usage:  R0     Exception Type
*                  R1
*                  R2     Return PC
*********************************************************************************************************/

RTOS_ARM_ExceptDataAbortHndlr:
    SUB     LR, LR, #8                                          @ LR offset to return from this exception: -8.
    STMFD   SP!, {R0-R3}                                        @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #RTOS_ARM_EXCEPT_DATA_ABORT                   @ Set exception ID to RTOS_ARM_EXCEPT_DATA_ABORT.
    BL BSP_UndefAbortExceptionHook
    B            RTOS_ARM_ExceptHndlr                         @ Branch to global exception handler.


/*********************************************************************************************************
*                                   ADDRESS ABORT EXCEPTION HANDLER
*
* Register Usage:  R0     Exception Type
*                  R1
*                  R2     Return PC
*********************************************************************************************************/

RTOS_ARM_ExceptAddrAbortHndlr:
    SUB     LR, LR, #8                                          @ LR offset to return from this exception: -8.
    STMFD   SP!, {R0-R3}                                        @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #RTOS_ARM_EXCEPT_ADDR_ABORT                   @ Set exception ID to RTOS_ARM_EXCEPT_ADDR_ABORT.
    BL BSP_UndefAbortExceptionHook
    B            RTOS_ARM_ExceptHndlr                         @ Branch to global exception handler.


/*********************************************************************************************************
*                                 INTERRUPT REQUEST EXCEPTION HANDLER
*
* Register Usage:  R0     Exception Type
*                  R1
*                  R2     Return PC
*********************************************************************************************************/

RTOS_ARM_ExceptIrqHndlr:
    SUB     LR, LR, #4                                          @ LR offset to return from this exception: -4.
    STMFD   SP!, {R0-R3}                                        @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #RTOS_ARM_EXCEPT_IRQ                          @ Set exception ID to RTOS_ARM_EXCEPT_IRQ.
    B            RTOS_ARM_ExceptHndlr                         @ Branch to global exception handler.


/*********************************************************************************************************
*                              FAST INTERRUPT REQUEST EXCEPTION HANDLER
*
* Register Usage:  R0     Exception Type
*                  R1
*                  R2     Return PC
*********************************************************************************************************/

RTOS_ARM_ExceptFiqHndlr:
    SUB     LR, LR, #4                                          @ LR offset to return from this exception: -4.
    STMFD   SP!, {R0-R3}                                        @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #RTOS_ARM_EXCEPT_FIQ                          @ Set exception ID to RTOS_ARM_EXCEPT_FIQ.
    B            RTOS_ARM_ExceptHndlr                         @ Branch to global exception handler.


/*********************************************************************************************************
*                                      GLOBAL EXCEPTION HANDLER
*
*********************************************************************************************************/

RTOS_ARM_ExceptHndlr:

     BX      LR


/*********************************************************************************************************
*                              VFP/NEON REGISTER COUNT
*
* Register Usage:  R0     Double Register Count
*********************************************************************************************************/

RTOS_ARM_DRegCntGet:
    MOV     R0, #0
    BX      LR


CPU_WaitForEvent:
        DSB
        WFE                                     @ Wait for exception
        BX      LR


CPU_DSB:
        DSB
        ISB
        BX      LR

/*********************************************************************************************************
*                                      CRITICAL SECTION FUNCTIONS
*
* Description : Disable/Enable interrupts by preserving the state of interrupts.  Generally speaking, the
*               state of the interrupt disable flag is stored in the local variable 'cpu_sr' & interrupts
*               are then disabled ('cpu_sr' is allocated in all functions that need to disable interrupts).
*               The previous interrupt state is restored by copying 'cpu_sr' into the CPU's status register.
*
* Prototypes  : CPU_SR  CPU_SR_Save   (void);
*               void    CPU_SR_Restore(CPU_SR  cpu_sr);
*
* Note(s)     : (1) These functions are used in general like this :
*
*                       void  Task (void  *p_arg)
*                       {
*                           CPU_SR_ALLOC();                     // Allocate storage for CPU status register
*                               :
*                               :
*                           CPU_CRITICAL_ENTER();               // cpu_sr = CPU_SR_Save();
*                               :
*                               :
*                           CPU_CRITICAL_EXIT();                // CPU_SR_Restore(cpu_sr);
*                               :
*                       }
*********************************************************************************************************/

CPU_SR_Save:
        MRS     R0, CPSR
        CPSID   IF                                              @ Set IRQ & FIQ bits in CPSR to DISABLE all interrupts
        DSB
        BX      LR                                              @ DISABLED, return the original CPSR contents in R0


CPU_SR_Restore:
        DSB
        MSR     CPSR_c, R0
        BX      LR


    .end
