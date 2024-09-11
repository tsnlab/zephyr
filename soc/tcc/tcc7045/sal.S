/*****************************************************
 *
 * osal.S
 * Telechips OS abstraction Layer
 *
 * History
 * -------------
 * Created by : Bong Sik, Kim  2017/10/31
 *
 *****************************************************/
 
    .global  ARM_ExceptUndefInstrHndlr
    .global  ARM_ExceptSwiHndlr
    .global  ARM_ExceptPrefetchAbortHndlr
    .global  ARM_ExceptDataAbortHndlr
    .global  ARM_ExceptIrqHndlr
    .global  ARM_ExceptFiqHndlr

/*********************************************************************************************************
*                               UNDEFINED INSTRUCTION EXCEPTION HANDLER
*********************************************************************************************************/

ARM_ExceptUndefInstrHndlr:
    B            RTOS_ARM_ExceptUndefInstrHndlr


/*********************************************************************************************************
*                                SOFTWARE INTERRUPT EXCEPTION HANDLER
*********************************************************************************************************/

ARM_ExceptSwiHndlr:
    B            RTOS_ARM_ExceptSwiHndlr


/*********************************************************************************************************
*                                  PREFETCH ABORT EXCEPTION HANDLER
*********************************************************************************************************/

ARM_ExceptPrefetchAbortHndlr:
    B            RTOS_ARM_ExceptPrefetchAbortHndlr


/*********************************************************************************************************
*                                    DATA ABORT EXCEPTION HANDLER
*********************************************************************************************************/

ARM_ExceptDataAbortHndlr:
    B            RTOS_ARM_ExceptDataAbortHndlr


/*********************************************************************************************************
*                                 INTERRUPT REQUEST EXCEPTION HANDLER
*********************************************************************************************************/

ARM_ExceptIrqHndlr:
    B            RTOS_ARM_ExceptIrqHndlr


/*********************************************************************************************************
*                              FAST INTERRUPT REQUEST EXCEPTION HANDLER
*********************************************************************************************************/

ARM_ExceptFiqHndlr:
    B            RTOS_ARM_ExceptFiqHndlr


	.end
