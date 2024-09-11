/*
***************************************************************************************************
*
*   FileName : sal_com.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*
*   TCC Version 1.0
*
*   This source code contains confidential information of Telechips.
*
*   Any unauthorized use without a written permission of Telechips including not limited to
*   re-distribution in source or binary form is strictly prohibited.
*
*   This source code is provided "AS IS" and nothing contained in this source code shall constitute
*   any express or implied warranty of any kind, including without limitation, any warranty of
*   merchantability, fitness for a particular purpose or non-infringement of any patent, copyright
*   or other third party intellectual property right. No warranty is made, express or implied,
*   regarding the information's accuracy,completeness, or performance.
*
*   In no event shall Telechips be liable for any claim, damages or other liability arising from,
*   out of or in connection with this source code or the use in the source code.
*
*   This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement between
*   Telechips and Company.
*   This source code is provided "AS IS" and nothing contained in this source code shall constitute
*   any express or implied warranty of any kind, including without limitation, any warranty
*   (of merchantability, fitness for a particular purpose or non-infringement of any patent,
*   copyright or other third party intellectual property right. No warranty is made, express or
*   implied, regarding the information's accuracy, completeness, or performance.
*   In no event shall Telechips be liable for any claim, damages or other liability arising from,
*   out of or in connection with this source code or the use in the source code.
*   This source code is provided subject to the terms of a Mutual Non-Disclosure Agreement
*   between Telechips and Company.
*
***************************************************************************************************
*/

/* MISRA C:2012 - 4.10 */
#if !defined(SAL_COMMON_HEADER)
#define SAL_COMMON_HEADER

/*
***************************************************************************************************
*                                             [INCLUDE FILES]
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                             [DEFINITIONS]
***************************************************************************************************
*/

/*
 * Primitive Type, AUTOSAR type, MISRA C:2012 - 4.6
 */
typedef char                            int8;       /*        -128 .. +127            */
typedef signed char                     sint8;      /*        -128 .. +127            */
typedef unsigned char                   uint8;      /*           0 .. 255             */
typedef short                           int16;      /*      -32768 .. +32767          */
typedef signed short                    sint16;     /*      -32768 .. +32767          */
typedef unsigned short                  uint16;     /*           0 .. 65535           */
typedef long                            int32;      /* -2147483648 .. +2147483647     */
typedef signed long                     sint32;     /* -2147483648 .. +2147483647     */
typedef unsigned long                   uint32;     /*           0 .. 4294967295      */
typedef long long                       int64;
typedef signed long long                sint64;
typedef unsigned long long              uint64;     /*       0..18446744073709551615  */

typedef int                             int32a;     /* -2147483648 .. +2147483647     */
typedef signed int                      sint32a;    /* -2147483648 .. +2147483647     */
typedef unsigned int                    uint32a;    /*           0 .. 4294967295      */

typedef uint32                          uintptr;
typedef sint32                          sintptr;

typedef unsigned char                   boolean;    /* for use with TRUE/FALSE        */

typedef uint32                          SALSize;

/* MISRA C:2012-11.8, The macro NULL shall be the only permitted form of integer null pointer constant */
#ifndef NULL
#define NULL                            (0)
#endif

#ifndef NULL_PTR
#define NULL_PTR                        ((void *)0)
#endif

#ifndef FALSE
#define FALSE                           (0U)
#endif

#ifndef TRUE
#define TRUE                            (1U)
#endif

#define SALDisabled                     (FALSE)
#define SALEnabled                      (TRUE)

/*
 * Address Type
 */
typedef uint32                          SALAddr;

typedef volatile uint32                 SALReg32;
typedef volatile uint64                 SALReg64;

#define SAL_WriteReg(v, a)              (*((volatile uint32 *)(a)) = (uint32)(v))
#define SAL_ReadReg(a)                  (*(volatile uint32 *)(a))

/*
 * Handles
 */
typedef uint32                          SALFileHandle;
typedef uint32                          SALSearchHandle;
typedef uint32                          SALSockHandle;

/*
 * Specific Values
 */
#define SAL_MAX_QUEUE                   (32)
#define SAL_MAX_TASK                    (32)
#define SAL_MAX_SEMA                    (32)
#define SAL_MAX_EVENT                   (32)

#define SAL_MAX_INT_VAL                 (4294967295UL)

#define SAL_ArraySize(x)                (sizeof(x)/sizeof((x)[0]))

#define SAL_OS_TIMER_ID                 (0UL)     /* HW Timer Resource for OS sheduling */

#define SAL_MEM_FUNC_IMPL      /* In case of using os memory library ex OS_MemCopy(), OS_MemSet()..etc   */
#define SAL_STRING_FUN_IMPL    /* In case of  using os string library. XXX_StrCopy(), XXX_Strlen()...etc */


/* MISRA C:2012-20.7
 * 4U          : 32-bit word size (in octets)
 * 8U          : Octet NBR bits
 * 4294967295U : Unsigned int Maximum value
 */
#define SAL_BitField(bit_field, bit_shift)                                       \
    ((((bit_field) >= (4U * 8U)) ? (4294967295UL)                                \
                                 : ((1u << (bit_field)) - 1UL)) << (bit_shift))

/*
 * Driver Identification
 */
typedef enum SALDriverIdTable
{
    SAL_DRVID_SAL                       = 100,
    SAL_DRVID_MPU                       = 300,
    SAL_DRVID_GIC                       = 400,
    SAL_DRVID_PEFALSH                   = 500,
    SAL_DRVID_DEFALSH                   = 600,
    SAL_DRVID_SFMC                      = 700,
    SAL_DRVID_CAN                       = 800,
    SAL_DRVID_GPSB                      = 900,
    SAL_DRVID_UART                      = 1000,
    SAL_DRVID_I2C                       = 1100,
    SAL_DRVID_PDM                       = 1200,
    SAL_DRVID_ICTC                      = 1300,
    SAL_DRVID_GDMA                      = 1400,
    SAL_DRVID_ADC                       = 1500,
    SAL_DRVID_I2S                       = 1600,
    SAL_DRVID_GMAC                      = 1700,
    SAL_DRVID_HSM                       = 1800,
    SAL_DRVID_TMR                       = 1900,
    SAL_DRVID_WDT                       = 2000,
    SAL_DRVID_CKC                       = 2100,
    SAL_DRVID_CMU                       = 2200,
    SAL_DRVID_GPIO                      = 2300,
    SAL_DRVID_PMIO                      = 2400,
    SAL_DRVID_PMU                       = 2500,
    SAL_DRVID_FMU                       = 2600,
    SAL_DRVID_RTC                       = 2700,
    SAL_DRVID_STSCFG                    = 2800,
    SAL_DRVID_DSE                       = 2900,
    SAL_DRVID_SM                        = 3000,
    SAL_DRVID_FWUD                      = 3100,
    SAL_DRVID_LIN                       = 3200,

} SALDriverId_t;

/*
 * Struct Types
 */
typedef struct SALVersionInfoTable
{
    uint8 viMajorVersion;
    uint8 viMinorVersion;
    uint8 viPatchVersion;
    uint8 viReserved;

} SALMcuVersionInfo_t;

typedef struct SALTimeValue
{
    uint32                              tvSec;     /* seconds           */
    uint32                              tvUsec;    /* and microseconds  */

} SALTimeValue_t;

/*
 * File open mode
 */
typedef enum SALFileMode
{
    SAL_FILE_READ                       = 1,  /**< Read                      */
    SAL_FILE_WRITE                      = 2,  /**< Write                     */
    SAL_FILE_READWRITE                  = 3,  /**< Read and Write            */
    SAL_FILE_CREATE                     = 4   /**< Create                    */

} SALFileMode_t;

/*
 * File seek mode
 */
typedef enum SALFileSeekOrigin
{
    SAL_SEEK_BEGIN                      = 0,  /**< Beginning of file         */
    SAL_SEEK_CUR                        = 1,  /**< Current position          */
    SAL_SEEK_END                        = 2   /**< End of file               */

} SALFileSeekOrigin_t;

/*
 * File Attribute
 */
#define SAL_FILE_ATTR_NORMAL            (0x00U)  /* Normal file - No read/write restrictions  */
#define SAL_FILE_ATTR_RDONLY            (0x01U)  /* Read only file                            */
#define SAL_FILE_ATTR_HIDDEN            (0x02U)  /* Hidden file                               */
#define SAL_FILE_ATTR_SYSTEM            (0x04U)  /* System file                               */
#define SAL_FILE_ATTR_SUBDIR            (0x10U)  /* Subdirectory                              */
#define SAL_FILE_ATTR_ARCH              (0x20U)  /* Archive file                              */

/*
 * Queue options, MISRA C:2012-7.3
 */
typedef enum SALBlockingOption
{
    SAL_OPT_BLOCKING                    = 0x00000000UL,
    SAL_OPT_NON_BLOCKING                = 0x00008000UL

} SALBlockingOption_t;

/*
 * Event options
 */
typedef enum SALEventOption
{
    SAL_EVENT_OPT_FLAG_SET              = 0x00000000UL,  /**< Initial state signaled                            */
    SAL_EVENT_OPT_CLR_ALL               = 0x00000001UL,  /**< Initial state nonsignaled                         */
    SAL_EVENT_OPT_CLR_ANY               = 0x00000002UL,  /**< Wait for ANY of the bits specified to be CLR      */
    SAL_EVENT_OPT_SET_ALL               = 0x00000004UL,  /**< Wait for ALL    the bits specified to be SET      */
    SAL_EVENT_OPT_SET_ANY               = 0x00000008UL,  /**< Wait for ANY of the bits specified to be SET      */
    SAL_EVENT_OPT_CONSUME               = 0x00000100UL,  /**< Consume the flags if condition(s) satisfied       */

} SALEventOption_t;

/*
 * Task Priority Setting
 */
typedef enum SALTaskPriority
{
    SAL_PRIO_APP_CFG                    = 2,  /**< APP TASK START task                  */
    SAL_PRIO_POWER_MANAGER              = 3,  /**< Power Manager APP task               */
    SAL_PRIO_IPC_CONTROL                = 3,  /**< IPC Control task                     */
    SAL_PRIO_SYSTEM_MON                 = 4,  /**< System Monitoring APP task           */
    SAL_PRIO_CAN_DEMO                   = 4,  /**< CAN DEMO APP task                    */
    SAL_PRIO_AUDIO_TEST                 = 5,  /**< Audio Test APP task                  */
    SAL_PRIO_VSDR_MONITOR               = 5,  /**< Vehicle Signal DEMO APP task         */
    SAL_PRIO_SPILED_DEMO                = 6,  /**< SPILED Demo Task                     */
    SAL_PRIO_IPC_PARSER                 = 6,  /**< IPC Parser task                      */
    SAL_PRIO_KEY_APP                    = 6,  /**< KEY APP task                         */
    SAL_PRIO_UART_TEST                  = 6,  /**< Uart Test APP task                   */
    SAL_PRIO_LIN_TEST                   = 6,  /**< Lin Test APP task                    */
    SAL_PRIO_ETH_TASK                   = 6,  /**< Ethernet TX/RX task                  */
    SAL_PRIO_CONSOLE_DEMO               = 6,  /**< Console DEMO APP task                */
    SAL_PRIO_NVM_MAMAGER                = 8,  /**< NVM Manager APP task                 */
    SAL_PRIO_CONSOLE                    = 10, /**< Console task                         */
    SAL_PRIO_FWUD_APP                   = 10, /**< FWUD APP task                        */
    SAL_PRIO_LOWEST                     = 12, /**< APP TASK START task                  */

} SALTaskPriority_t;


/*
 * Return Codes
 */
typedef enum SALRetCode
{
    SAL_RET_SUCCESS                     = 0,
    SAL_RET_FAILED                      = 1

} SALRetCode_t;

/*
 * Error Codes
 */
typedef enum SALErrorCode
{
    SAL_ERR_INIT                        = 100,  /**< Initialization error for each APIs             */
    SAL_ERR_NO_SPACE                    = 101,  /**< No more space (memory, channel, etc)           */
    SAL_ERR_INVALID_PARAMETER           = 102,  /**< Invalid parameter is passed                    */
    SAL_ERR_NOT_SUPPORT                 = 103,  /**< Not supported operation or resources           */
    SAL_ERR_TIMEOUT                     = 104,  /**< Timed out while processing                     */
    SAL_ERR_INVALID_HANDLE              = 105,  /**< Invalid handle is detected                     */
    SAL_ERR_NO_DATA                     = 106,  /**< No data                                        */
    SAL_ERR_UNDEF_STATE                 = 107,  /**< Not defined state                              */
    SAL_ERR_FAIL_CREATE                 = 108,  /**< Fail to create a component(Task, event, etc)   */
    SAL_ERR_FAIL_GET_DATA               = 109,  /**< Fail to get data from a component              */
    SAL_ERR_FAIL_SEND_DATA              = 110,  /**< Fail to send data to a component               */
    SAL_ERR_FAIL_START                  = 111,  /**< Fail to start a component                      */
    SAL_ERR_FAIL_DELETE                 = 112,  /**< Fail to delete a job of a component            */
    SAL_ERR_FAIL_RELEASE                = 113,  /**< Fail to release a job of a component           */
    SAL_ERR_UNINIT_ITEM                 = 114,  /**< Uninitialized item, variable, or contents      */
    SAL_ERR_OUTOF_SIZE                  = 115,  /**< Size overflow or is lower than threshold       */
    SAL_ERR_FAIL_GET_KEY                = 116,  /**< Fail to get key or ownership                   */
    SAL_ERR_FAIL_SET_CONFIG             = 117,  /**< Fail to set configuration or status            */
    SAL_ERR_NOT_USEFUL                  = 118   /**< The status is not available                    */

} SALErrorCode_t;

/*
 * Error Information
 */
typedef struct SALErrorInfoTable
{
    SALDriverId_t                       eiDrvId;    /* Driver/Module Id */
    uint32                              eiApiId;    /* API func Id      */
    SALErrorCode_t                      eiErrorId;  /* Error Id         */

} SALErrorInfo_t;

/*
***************************************************************************************************
*                                         [FUNCTION PROTOTYPES Types]
***************************************************************************************************
*/
/* Task start functions type */
typedef void                            (*SALTaskFunc) (void * pArg);

/* ARM Core */
typedef SALRetCode_t                    (*SALCoreMB) (void);
typedef SALRetCode_t                    (*SALCoreRMB) (void);
typedef SALRetCode_t                    (*SALCoreWMB) (void);
typedef SALRetCode_t                    (*SALCoreCriticalEnter) (void);
typedef SALRetCode_t                    (*SALCoreCriticalExit) (void);
typedef SALRetCode_t                    (*SALCoreWaitForEvent) (void);
typedef SALRetCode_t                    (*SALCoreDiv64To32) (uint64 * pullDividend, uint32 uiDivisor, uint32 * puiRem);

/* OS Runtime */
typedef SALRetCode_t                    (*SALOSInitFunctions) (void);
typedef SALRetCode_t                    (*SALOSStart) (void);

/* Memory */
typedef SALRetCode_t                    (*SALMemAlloc) (void * pMem, uint32 uiSize);
typedef SALRetCode_t                    (*SALMemCalloc) (void * pMem, uint32 uiNum, SALSize uiSize);
typedef SALRetCode_t                    (*SALMemFree) (void * pMem);
typedef SALRetCode_t                    (*SALMemSet) (void * pMem, uint8 ucValue, SALSize uiSize);
typedef SALRetCode_t                    (*SALMemCopy) (void * pDest, const void * pSrc, SALSize uiSize);
typedef SALRetCode_t                    (*SALMemCmp) (const void * pMem1, const void * pMem2, SALSize uiSize, sint32 * piRetCmp);
typedef SALRetCode_t                    (*SALStrCopy) (uint8 * pucDestString, const uint8 * pucSrcString);
typedef SALRetCode_t                    (*SALStrCmp) (const uint8 * puc1String, const uint8 * puc2String, sint32 * piRetCmp);
typedef SALRetCode_t                    (*SALStrNCmp) (const uint8 * puc1String, const uint8 * puc2String, SALSize uiLength, sint32 * piRetCmp);
typedef SALRetCode_t                    (*SALStrLength) (const int8 * pucString, SALSize * puiLength);

/* File / Directory */
typedef SALRetCode_t                    (*SALFileOpenA) (SALFileHandle * puiFile, sint8 * pcFilename, SALFileMode_t uiMode);
typedef SALRetCode_t                    (*SALFileOpenW) (SALFileHandle * puiFile, sint8 * puhwFilename, SALFileMode_t uiMode);
typedef SALRetCode_t                    (*SALFileClose) (SALFileHandle uiFile);
typedef SALRetCode_t                    (*SALFileRead) (SALFileHandle uiFile, void * pBuf, uint32 uiBytesToRead);
typedef SALRetCode_t                    (*SALFileWrite) (SALFileHandle uiFile, void * pBuf, uint32 uiBytesToWrite);
typedef SALRetCode_t                    (*SALFileSeek) (SALFileHandle uiFile, uint32 uiOffset, SALFileSeekOrigin_t uiOrigin);
typedef SALRetCode_t                    (*SALFileSize) (SALFileHandle uiFile);
typedef SALRetCode_t                    (*SALFileFreeDiskSpace) (sint8 * pcRootPathName);
typedef SALRetCode_t                    (*SALFileRemoveA) (sint8 * pcFilename );
typedef SALRetCode_t                    (*SALFileRemoveW) (sint8 * puhwFilename );
typedef SALRetCode_t                    (*SALFileRename) (sint8 * pcOldName, sint8 *pcNewName);
typedef SALRetCode_t                    (*SALFileGetAttribute) (sint8 * pcFilename, uint32 * puiAttribute);
typedef SALRetCode_t                    (*SALDirMake) (sint8 * pcDirname);
typedef SALRetCode_t                    (*SALDirRemove) (sint8 * pcDirname);
typedef SALRetCode_t                    (*SALFindClose) (SALSearchHandle uiHandle);
typedef SALRetCode_t                    (*SALFileSizeEx) (SALFileHandle uiFile, uint32 * puiSizeHigh, uint32 * puiSizeLow);
typedef SALRetCode_t                    (*SALFileFreeDiskSpaceEx) (sint8 * pcRootPathName, SALSize * puiSizeHigh, SALSize * puiSizeLow);

/* Data Structure - Queue */
typedef SALRetCode_t                    (*SALQueueCreate) (uint32 * puiId, const uint8 * pucName, uint32 uiDepth, SALSize uiDataSize);
typedef SALRetCode_t                    (*SALQueueDelete) (uint32 uiId);
typedef SALRetCode_t                    (*SALQueueGet) (uint32 uiId, void * pData, uint32 * puiSizeCopied, uint32a iTimeout, SALBlockingOption_t  ulOptions);
typedef SALRetCode_t                    (*SALQueuePut) (uint32 uiId, void * pData, SALSize uiSize, sint32 iTimeout, SALBlockingOption_t  ulOptions);

/* Event */
typedef SALRetCode_t                    (*SALEventCreate) (uint32 * puiId, const uint8 * pucName, uint32 uiInitialValue);
typedef SALRetCode_t                    (*SALEventDelete) (uint32 uiId);
typedef SALRetCode_t                    (*SALEventSet) (uint32 uiId, uint32 uiEvent, SALEventOption_t ulOptions);
typedef SALRetCode_t                    (*SALEventGet) (uint32 uiId, uint32 uiEvent, uint32 iTimeout, uint32 uiOptions, uint32 * puiFlags);

/* Mutex */
typedef SALRetCode_t                    (*SALMutexCreate) ( void );
typedef SALRetCode_t                    (*SALMutexDelete) (uint32 uiId);
typedef SALRetCode_t                    (*SALMutexLock) (uint32 uiId, uint32 uiTimeoutmsec);
typedef SALRetCode_t                    (*SALMutexUnlock) (uint32 uiId);

/* Semaphore */
typedef SALRetCode_t                    (*SALSemaphoreCreate) (uint32 * puiId, const uint8 * pucName, uint32 uiInitialValue, SALBlockingOption_t ulOptions);
typedef SALRetCode_t                    (*SALSemaphoreDelete) (uint32 uiId);
typedef SALRetCode_t                    (*SALSemaphoreRelease) (uint32 uiId);
typedef SALRetCode_t                    (*SALSemaphoreWait) (uint32 uiId, sint32 iTimeout, SALBlockingOption_t ulOptions);

/* Task */
typedef SALRetCode_t                    (*SALTaskCreate) (uint32 * puiTaskId, const uint8 * pucTaskName, SALTaskFunc fnTask, uint32 * puiStackPtr, uint32 uiStackSize, SALTaskPriority_t uiPriority, void * pTaskParam);
typedef SALRetCode_t                    (*SALTaskDelete) (uint32 uiTaskId);
typedef SALRetCode_t                    (*SALTaskCurrent) (uint32 * puiTaskId);
typedef SALRetCode_t                    (*SALTaskSleep) (uint32 uiMilliSec);

/* Socket */
typedef SALSockHandle                   (*SALNetSocket) (sint32 iType);
typedef SALRetCode_t                    (*SALNetClose) (SALSockHandle uiSock);
typedef SALRetCode_t                    (*SALNetConnect) (SALSockHandle uiSock, sint8 * pucAddr, uint16 uhwPort, uint32 uiTimeout);
typedef SALRetCode_t                    (*SALNetBind) (SALSockHandle uiSock, uint16 uhwPort);
typedef SALRetCode_t                    (*SALNetSendTo) (SALSockHandle uiSock, sint8 * pcBuf, uint32 uiLen, sint8 * pcAddr, uint16 uhwPort);
typedef SALRetCode_t                    (*SALNetSend) (SALSockHandle uiSock, sint8 * pcBuf, uint32 uiLen );
typedef SALRetCode_t                    (*SALNetRecvFrom) (SALSockHandle uiSock, sint8 * pcBuf, uint32 uiLen, SALAddr * puiAddr, uint16 * puhwPort, uint32 uiTimeout);
typedef SALRetCode_t                    (*SALNetRecv) (SALSockHandle uiSock, sint8 * pcBuf, uint32 uiLen, uint32 uiTimeout);

/* Debugging */
typedef SALRetCode_t                    (*SALReportError) (SALDriverId_t uiDriverId, uint32 uiApiId, uint32 uiErrorCode, const int8 * pucEtc);
typedef SALRetCode_t                    (*SALGetCurrentError) (SALErrorInfo_t * psInfo);

/* Etc */
/* Time */
typedef SALRetCode_t                    (*SALGetTickCount) (uint32 * puiTickCount);
typedef SALRetCode_t                    (*SALHandleTick) ( void );
typedef SALRetCode_t                    (*SALGetVersion) (SALMcuVersionInfo_t * psVersion);
typedef SALRetCode_t                    (*SALGetSystemTickCount) (uint64 * pullSysTickCount);
typedef SALRetCode_t                    (*SALIncreaseSystemTick) (uint64 * pullSysTickCount);

/*
***************************************************************************************************
*                                             [DEFINITIONS of TCSAL Functions]
***************************************************************************************************
*/
/* ARM Core */
typedef struct SALCoreTable
{
    SALCoreMB                           fnCoreMB;
    SALCoreRMB                          fnCoreRMB;
    SALCoreWMB                          fnCoreWMB;
    SALCoreCriticalEnter                fnCoreCriticalEnter;
    SALCoreCriticalExit                 fnCoreCriticalExit;
    SALCoreWaitForEvent                 fnCoreWaitForEvent;
    SALCoreDiv64To32                    fnCoreDiv64To32;

} SALCoreFunc_t;

/* OS Runtime */
typedef struct SALOSRunTable
{
    SALOSInitFunctions                  fnOsInitFuncs;
    SALOSStart                          fnOsStart;

} SALOSRunFunc_t;

/* Memory */
typedef struct SALMemTable
{
    SALMemAlloc                         fnMemAlloc;
    SALMemCalloc                        fnMemCalloc;
    SALMemFree                          fnMemFree;
    SALMemSet                           fnMemSet;
    SALMemCopy                          fnMemCopy;
    SALMemCmp                           fnMemCmp;
    SALStrCopy                          fnStrCopy;
    SALStrCmp                           fnStrCmp;
    SALStrNCmp                          fnStrNCmp;
    SALStrLength                        fnStrLength;

} SALMemFunc_t;

/* File / Directory */
typedef struct SALFileTable
{
    SALFindClose                        fnFindClose;
    SALFileFreeDiskSpace                fnFileFreeDiskSpace;
    SALFileOpenA                        fnFileOpenA;
    SALFileRead                         fnFileRead;
    SALFileRemoveA                      fnFileRemoveA;
    SALFileSeek                         fnFileSeek;
    SALFileSize                         fnFileSize;
    SALFileWrite                        fnFileWrite;
    SALFileRename                       fnFileRename;
    SALFileGetAttribute                 fnFileGetAttribute;
    SALDirMake                          fnDirMake;
    SALDirRemove                        fnDirRemove;
    SALFileClose                        fnFileClose;
    SALFileSizeEx                       fnFileSizeEx;
    SALFileFreeDiskSpaceEx              fnFileFreeDiskSpaceEx;
    SALFileOpenW                        fnFileOpenW;
    SALFileRemoveW                      fnFileRemoveW;

} SALFileFunc_t;

typedef struct SALDataStructureTable
{
    SALQueueCreate                      fnQueueCreate;
    SALQueueDelete                      fnQueueDelete;
    SALQueueGet                         fnQueueGet;
    SALQueuePut                         fnQueuePut;

} SALDataStructFunc_t;

/* Synchronization Object */
typedef struct SALSyncObjectTable
{
    /* Event */
    SALEventCreate                      fnEventCreate;
    SALEventDelete                      fnEventDelete;
    SALEventSet                         fnEventSet;
    SALEventGet                         fnEventGet;

    /* Mutex */
    SALMutexCreate                      fnMutexCreate;
    SALMutexDelete                      fnMutexDelete;
    SALMutexLock                        fnMutexLock;
    SALMutexUnlock                      fnMutexUnlock;

    /* Semaphore */
    SALSemaphoreCreate                  fnSemaphoreCreate;
    SALSemaphoreDelete                  fnSemaphoreDelete;
    SALSemaphoreRelease                 fnSemaphoreRelease;
    SALSemaphoreWait                    fnSemaphoreWait;

} SALSyncObjectFunc_t;

/* Task */
typedef struct SALTaskTable
{
    SALTaskCreate                       fnTaskCreate;
    SALTaskCurrent                      fnTaskCurrent;
    SALTaskDelete                       fnTaskDelete;
    SALTaskSleep                        fnTaskSleep;

} SALTaskFunc_t;

/* Socket */
typedef struct SALSocketTable
{
    SALNetSocket                        fnNetSocket;
    SALNetClose                         fnNetClose;
    SALNetConnect                       fnNetConnect;
    SALNetBind                          fnNetBind;
    SALNetSendTo                        fnNetSendTo;
    SALNetSend                          fnNetSend;
    SALNetRecvFrom                      fnNetRecvFrom;
    SALNetRecv                          fnNetRecv;

} SALSocketFunc_t;

/* Debug */
typedef struct SALTraceTable
{
    SALReportError                      fnReportError;
    SALGetCurrentError                  fnGetCurrentError;

} SALTraceFunc_t;

/* Etc */
typedef struct SALEtcTable
{
    SALGetTickCount                     fnGetTickCount;
    SALHandleTick                       fnHandleTick;
    SALGetVersion                       fnGetVersion;
    SALGetSystemTickCount               fnGetSystemTickCount;
    SALIncreaseSystemTick               fnIncreaseSystemTick;

} SALEtcFunc_t;

#endif // SAL_COMMON_HEADER

