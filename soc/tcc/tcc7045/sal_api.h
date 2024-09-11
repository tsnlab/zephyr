/*
***************************************************************************************************
*
*   FileName : sal_api.h
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

#if !defined(SAL_API_HEADER)
#define SAL_API_HEADER

/*
***************************************************************************************************
*                                             [INCLUDE FILES]
***************************************************************************************************
*/

#include "sal_com.h"

/*
***************************************************************************************************
*                                             [DEFINITIONS]
***************************************************************************************************
*/
/*
 * Function Indexes
 */
#define SAL_API_COMMON_INDEX            (0x0)
#define SAL_API_COMMON_INIT             (SAL_API_COMMON_INDEX + 0)
#define SAL_API_COMMON_REG_FUNC         (SAL_API_COMMON_INDEX + 1)

#define SAL_CORE_FUNCTIONS_INDEX        (0x0100)
#define SAL_API_CORE_MB                 (SAL_CORE_FUNCTIONS_INDEX + 0)
#define SAL_API_CORE_RMB                (SAL_CORE_FUNCTIONS_INDEX + 1)
#define SAL_API_CORE_WMB                (SAL_CORE_FUNCTIONS_INDEX + 2)
#define SAL_API_CORE_CRITICAL_ENTER     (SAL_CORE_FUNCTIONS_INDEX + 3)
#define SAL_API_CORE_CRITICAL_EXIT      (SAL_CORE_FUNCTIONS_INDEX + 4)
#define SAL_API_CORE_WAITFOREVENT       (SAL_CORE_FUNCTIONS_INDEX + 5)
#define SAL_API_CORE_DIV_64_32          (SAL_CORE_FUNCTIONS_INDEX + 6)

#define SAL_OS_RUN_FUNCTIONS_INDEX      (0x0200)
#define SAL_API_OS_INIT_FUNCS           (SAL_OS_RUN_FUNCTIONS_INDEX + 0)
#define SAL_API_OS_START                (SAL_OS_RUN_FUNCTIONS_INDEX + 1)

#define SAL_MEM_FUNCTIONS_INDEX         (0x0300)
#define SAL_API_MEM_ALLOC               (SAL_MEM_FUNCTIONS_INDEX + 0)
#define SAL_API_MEM_CALLOC              (SAL_MEM_FUNCTIONS_INDEX + 1)
#define SAL_API_MEM_FREE                (SAL_MEM_FUNCTIONS_INDEX + 2)
#define SAL_API_MEM_SET                 (SAL_MEM_FUNCTIONS_INDEX + 3)
#define SAL_API_MEM_COPY                (SAL_MEM_FUNCTIONS_INDEX + 4)
#define SAL_API_MEM_CMP                 (SAL_MEM_FUNCTIONS_INDEX + 5)
#define SAL_API_STR_COPY                (SAL_MEM_FUNCTIONS_INDEX + 6)
#define SAL_API_STR_CMP                 (SAL_MEM_FUNCTIONS_INDEX + 7)
#define SAL_API_STR_NCMP                (SAL_MEM_FUNCTIONS_INDEX + 8)
#define SAL_API_STR_LENGTH              (SAL_MEM_FUNCTIONS_INDEX + 9)

#define SAL_FILE_FUNCTIONS_INDEX        (0x0400)
#define SAL_API_FILE_CLOSE              (SAL_FILE_FUNCTIONS_INDEX + 0)
#define SAL_API_FILE_FREEDISKSPACE      (SAL_FILE_FUNCTIONS_INDEX + 1)
#define SAL_API_FILE_OPENA              (SAL_FILE_FUNCTIONS_INDEX + 2)
#define SAL_API_FILE_READ               (SAL_FILE_FUNCTIONS_INDEX + 3)
#define SAL_API_FILE_REMOVEA            (SAL_FILE_FUNCTIONS_INDEX + 4)
#define SAL_API_FILE_SEEK               (SAL_FILE_FUNCTIONS_INDEX + 5)
#define SAL_API_FILE_SIZE               (SAL_FILE_FUNCTIONS_INDEX + 6)
#define SAL_API_FILE_WRITE              (SAL_FILE_FUNCTIONS_INDEX + 7)
#define SAL_API_FILE_RENAME             (SAL_FILE_FUNCTIONS_INDEX + 8)
#define SAL_API_FILE_GET_ATTRIBUTE      (SAL_FILE_FUNCTIONS_INDEX + 9)
#define SAL_API_DIR_MAKE                (SAL_FILE_FUNCTIONS_INDEX + 10)
#define SAL_API_DIR_REMOVE              (SAL_FILE_FUNCTIONS_INDEX + 11)
#define SAL_API_FIND_CLOSE              (SAL_FILE_FUNCTIONS_INDEX + 12)
#define SAL_API_FILE_SIZEEX             (SAL_FILE_FUNCTIONS_INDEX + 13)
#define SAL_API_FILE_FREEDISKSPACEEX    (SAL_FILE_FUNCTIONS_INDEX + 14)
#define SAL_API_FILE_OPENW              (SAL_FILE_FUNCTIONS_INDEX + 15)
#define SAL_API_FILE_REMOVEW            (SAL_FILE_FUNCTIONS_INDEX + 16)

#define SAL_DATAS_FUNCTIONS_INDEX       (0x0500)
#define SAL_API_DATA_QUEUE_CREATE       (SAL_DATAS_FUNCTIONS_INDEX + 0)
#define SAL_API_DATA_QUEUE_DELETE       (SAL_DATAS_FUNCTIONS_INDEX + 1)
#define SAL_API_DATA_QUEUE_GET          (SAL_DATAS_FUNCTIONS_INDEX + 2)
#define SAL_API_DATA_QUEUE_PUT          (SAL_DATAS_FUNCTIONS_INDEX + 3)

#define SAL_SYNCOBJ_FUNCTIONS_INDEX     (0x0600)
#define SAL_API_EVENT_CREATE            (SAL_SYNCOBJ_FUNCTIONS_INDEX + 0)
#define SAL_API_EVENT_DELETE            (SAL_SYNCOBJ_FUNCTIONS_INDEX + 1)
#define SAL_API_EVENT_SET               (SAL_SYNCOBJ_FUNCTIONS_INDEX + 2)
#define SAL_API_EVENT_GET               (SAL_SYNCOBJ_FUNCTIONS_INDEX + 3)

#define SAL_API_MUTEX_CREATE            (SAL_SYNCOBJ_FUNCTIONS_INDEX + 4)
#define SAL_API_MUTEX_DELETE            (SAL_SYNCOBJ_FUNCTIONS_INDEX + 5)
#define SAL_API_MUTEX_LOCK              (SAL_SYNCOBJ_FUNCTIONS_INDEX + 6)
#define SAL_API_MUTEX_UNLOCK            (SAL_SYNCOBJ_FUNCTIONS_INDEX + 7)

#define SAL_API_SEMAPHORE_CREATE        (SAL_SYNCOBJ_FUNCTIONS_INDEX + 8)
#define SAL_API_SEMAPHORE_DELETE        (SAL_SYNCOBJ_FUNCTIONS_INDEX + 9)
#define SAL_API_SEMAPHORE_RELEASE       (SAL_SYNCOBJ_FUNCTIONS_INDEX + 10)
#define SAL_API_SEMAPHORE_WAIT          (SAL_SYNCOBJ_FUNCTIONS_INDEX + 11)

#define SAL_TASK_FUNCTIONS_INDEX        (0x0700)
#define SAL_API_TASK_CREATE             (SAL_TASK_FUNCTIONS_INDEX + 0)
#define SAL_API_TASK_CURRENT            (SAL_TASK_FUNCTIONS_INDEX + 1)
#define SAL_API_TASK_DELETE             (SAL_TASK_FUNCTIONS_INDEX + 2)
#define SAL_API_TASK_SLEEP              (SAL_TASK_FUNCTIONS_INDEX + 3)

#define SAL_SOCKET_FUNCTIONS_INDEX      (0x0800)
#define SAL_API_NET_SOCKET              (SAL_SOCKET_FUNCTIONS_INDEX + 0)
#define SAL_API_NET_CLOSE               (SAL_SOCKET_FUNCTIONS_INDEX + 1)
#define SAL_API_NET_CONNECT             (SAL_SOCKET_FUNCTIONS_INDEX + 2)
#define SAL_API_NET_BIND                (SAL_SOCKET_FUNCTIONS_INDEX + 3)
#define SAL_API_NET_SENDTO              (SAL_SOCKET_FUNCTIONS_INDEX + 4)
#define SAL_API_NET_SEND                (SAL_SOCKET_FUNCTIONS_INDEX + 5)
#define SAL_API_NET_RECVFROM            (SAL_SOCKET_FUNCTIONS_INDEX + 6)
#define SAL_API_NET_RECV                (SAL_SOCKET_FUNCTIONS_INDEX + 7)

#define SAL_TRACE_FUNCTIONS_INDEX       (0x0900)
#define SAL_API_DBG_REPORT_ERR          (SAL_TRACE_FUNCTIONS_INDEX + 0)
#define SAL_API_GET_CURRENT_ERR         (SAL_TRACE_FUNCTIONS_INDEX + 1)

#define SAL_ETC_FUNCTIONS_INDEX         (0x1000)
#define SAL_API_GET_TICKCOUNT           (SAL_ETC_FUNCTIONS_INDEX + 0)
#define SAL_API_HANDLE_TICK             (SAL_ETC_FUNCTIONS_INDEX + 1)
#define SAL_API_GET_VERSION             (SAL_ETC_FUNCTIONS_INDEX + 2)
#define SAL_API_GET_SYSTEM_TICK_CNT     (SAL_ETC_FUNCTIONS_INDEX + 3)
#define SAL_API_INCREASE_SYSTEM_TICK    (SAL_ETC_FUNCTIONS_INDEX + 4)

extern SALCoreFunc_t *                  gSALCoreTable;
extern SALOSRunFunc_t *                 gSALOSRunTable;
extern SALMemFunc_t *                   gSALMemoryTable;
extern SALFileFunc_t *                  gSALFileTable;
extern SALDataStructFunc_t *            gSALDataStructTable;
extern SALSyncObjectFunc_t *            gSALSyncObjectTable;
extern SALTaskFunc_t *                  gSALTaskTable;
extern SALSocketFunc_t *                gSALSocketTable;
extern SALTraceFunc_t *                 gSALTraceTable;
extern SALEtcFunc_t *                   gSALEtcTable;

#ifdef __cplusplus
extern "C"
{
#endif

/*
***************************************************************************************************
*                                         [FUNCTION PROTOTYPES]
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                          [SAL_Init]
*
* Initializes every elements and functions
*
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t SAL_Init
(
    void
);

/*
***************************************************************************************************
*                                          [SAL_RegisterFunction]
*
* Registers  functions needed by specific application.
*
* @param    uiFunctionIndex [in] Index of function to register.
* @param    pFunc [in] Pointer to function that corresponds with function index.
* @param    pPreFunc [in] Pointer to function that
* @return
*
* Notes
*
***************************************************************************************************
*/
SALRetCode_t SAL_RegisterFunction
(
    uint32                              uiFunctionIndex,
    void *                              pFunc,
    const void * const *                ppPreFunc
);

#ifdef __cplusplus
}
#endif

#endif // SAL_API_HEADER

