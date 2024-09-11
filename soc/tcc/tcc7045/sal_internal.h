/*
***************************************************************************************************
*
*   FileName : sal_internal.h
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

#if !defined(SAL_INTERNAL_HEADER)
#define SAL_INTERNAL_HEADER

#include "sal_api.h"

/* MISRA C:2012-20.7 */
#define SAL_Check0(func)                (((func)!=NULL_PTR) ? (SALRetCode_t)(func)() : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)
#define SAL_Check1(func,v)              (((func)!=NULL_PTR) ? (SALRetCode_t)(func)((v)) : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)
#define SAL_Check2(func,v,w)            (((func)!=NULL_PTR) ? (SALRetCode_t)(func)((v),(w)) : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)
#define SAL_Check3(func,v,w,x)          (((func)!=NULL_PTR) ? (SALRetCode_t)(func)((v),(w),(x)) : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)
#define SAL_Check4(func,v,w,x,y)        (((func)!=NULL_PTR) ? (SALRetCode_t)(func)((v),(w),(x),(y)) : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)
#define SAL_Check5(func,v,w,x,y,z)      (((func)!=NULL_PTR) ? (SALRetCode_t)(func)((v),(w),(x),(y),(z)) : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)
#define SAL_Check6(func,v,w,x,y,z,a)    (((func)!=NULL_PTR) ? (SALRetCode_t)(func)((v),(w),(x),(y),(z),(a)) : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)
#define SAL_Check7(func,v,w,x,y,z,a,b)  (((func)!=NULL_PTR) ? (SALRetCode_t)(func)((v),(w),(x),(y),(z),(a),(b)) : (SALRetCode_t)SAL_ERR_NOT_SUPPORT)

/* ARM Core */
#define SAL_CoreMB()                    (SAL_Check0(gSALCoreTable->fnCoreMB))
#define SAL_CoreRMB()                   (SAL_Check0(gSALCoreTable->fnCoreRMB))
#define SAL_CoreWMB()                   (SAL_Check0(gSALCoreTable->fnCoreWMB))
#define SAL_CoreCriticalEnter()         (SAL_Check0(gSALCoreTable->fnCoreCriticalEnter))
#define SAL_CoreCriticalExit()          (SAL_Check0(gSALCoreTable->fnCoreCriticalExit))
#define SAL_CoreWaitForEvent()          (SAL_Check0(gSALCoreTable->fnCoreWaitForEvent))
#define SAL_CoreDiv64To32(v,w,x)        (SAL_Check3(gSALCoreTable->fnCoreDiv64To32, v, w, x))

/* OS Runtime */
#define SAL_OsInitFuncs()               (SAL_Check0(gSALOSRunTable->fnOsInitFuncs))
#define SAL_OsStart()                   (SAL_Check0(gSALOSRunTable->fnOsStart))

/* Memory */
#define SAL_MemAlloc(v,w)               (SAL_Check2(gSALMemoryTable->fnMemAlloc, v, w))
#define SAL_MemCalloc(v,w,x)            (SAL_Check3(gSALMemoryTable->fnMemCalloc, v, w, x))
#define SAL_MemFree(v)                  (SAL_Check1(gSALMemoryTable->fnMemFree, v))
#define SAL_MemSet(v,w,x)               (SAL_Check3(gSALMemoryTable->fnMemSet, v, w, x))
#define SAL_MemCopy(v,w,x)              (SAL_Check3(gSALMemoryTable->fnMemCopy, v, w, x))
#define SAL_MemCmp(v,w,x,y)             (SAL_Check4(gSALMemoryTable->fnMemCmp, v, w, x, y))
#define SAL_StrCopy(v,w)                (SAL_Check2(gSALMemoryTable->fnStrCopy, v, w))
#define SAL_StrCmp(v,w,x)               (SAL_Check3(gSALMemoryTable->fnStrCmp, v, w, x))
#define SAL_StrNCmp(v,w,x,y)            (SAL_Check4(gSALMemoryTable->fnStrNCmp, v, w, x, y))
#define SAL_StrLength(v,w)              (SAL_Check2(gSALMemoryTable->fnStrLength, v, w))

/* File */
#define SAL_FileOpenA(v,w,x)            (SAL_Check3(gSALFileTable->fnFileOpenA, v, w, x))
#define SAL_FileOpenW(v,w,x)            (SAL_Check3(gSALFileTable->fnFileOpenW, v, w, x))
#define SAL_FileClose(v)                (SAL_Check1(gSALFileTable->fnFileClose, v))
#define SAL_FileRead(v,w,x)             (SAL_Check3(gSALFileTable->fnFileRead, v, w, x))
#define SAL_FileWrite(v,w,x)            (SAL_Check3(gSALFileTable->fnFileWrite, v, w, x))
#define SAL_FileSeek(v,w,x)             (SAL_Check3(gSALFileTable->fnFileSeek, v, w, x))
#define SAL_FileSize(v)                 (SAL_Check1(gSALFileTable->fnFileSize, v))
#define SAL_FileFreeDiskSpace(v)        (SAL_Check1(gSALFileTable->fnFileFreeDiskSpace, v))
#define SAL_FileRemoveA(v)              (SAL_Check1(gSALFileTable->fnFileRemoveA, v))
#define SAL_FileRemoveW(v)              (SAL_Check1(gSALFileTable->fnFileRemoveW, v))
#define SAL_FileRename(v,w)             (SAL_Check2(gSALFileTable->fnFileRename, v, w))
#define SAL_FileGetAttribute(v,w)       (SAL_Check2(gSALFileTable->fnFileGetAttribute, v, w))
#define SAL_DirMake(v)                  (SAL_Check1(gSALFileTable->fnDirMake, v))
#define SAL_DirRemove(v)                (SAL_Check1(gSALFileTable->fnDirRemove, v))
#define SAL_FindClose(v)                (SAL_Check1(gSALFileTable->fnFindClose, v))
#define SAL_FileSizeEx(v,w,x)           (SAL_Check3(gSALFileTable->fnFileSizeEx, v, w, x))
#define SAL_FileFreeDiskSpaceEx(v,w,x)  (SAL_Check3(gSALFileTable->fnFileFreeDiskSpaceEx, v, w, x))

#ifdef UNICODE
#   define SAL_FileOpen                 (SAL_FileOpenW)
#   define SAL_FileRemove               (SAL_FileRemoveW)
#else
#   define CSAL_FileOpen                (SAL_FileOpenA)
#   define SAL_FileRemove               (SAL_FileRemoveA)
#endif

/* Data Structure */
/* Queue */
#define SAL_QueueCreate(v,w,x,y)        (SAL_Check4(gSALDataStructTable->fnQueueCreate, v, w, x, y))
#define SAL_QueueDelete(v)              (SAL_Check1(gSALDataStructTable->fnQueueDelete, v))
#define SAL_QueueGet(v,w,x,y,z)         (SAL_Check5(gSALDataStructTable->fnQueueGet, v, w, x, y, z))
#define SAL_QueuePut(v,w,x,y,z)         (SAL_Check5(gSALDataStructTable->fnQueuePut, v, w, x, y, z))

/* Synchronization Objects*/
/* Event */
#define SAL_EventCreate(v,w,x)          (SAL_Check3(gSALSyncObjectTable->fnEventCreate, v, w, x))
#define SAL_EventDelete(v)              (SAL_Check1(gSALSyncObjectTable->fnEventDelete, v))
#define SAL_EventSet(v,w,x)             (SAL_Check3(gSALSyncObjectTable->fnEventSet, v, w, x))
#define SAL_EventGet(v,w,x,y,z)         (SAL_Check5(gSALSyncObjectTable->fnEventGet, v, w, x, y, z))

/* Mutex */
#define SAL_MutexCreate()               (SAL_Check0(gSALSyncObjectTable->fnMutexCreate))
#define SAL_MutexDelete(v)              (SAL_Check1(gSALSyncObjectTable->fnMutexDelete, v))
#define SAL_MutexLock(v,w)              (SAL_Check2(gSALSyncObjectTable->fnMutexLock, v, w))
#define SAL_MutexUnlock(v)              (SAL_Check1(gSALSyncObjectTable->fnMutexUnlock, v))

/* Semaphore */
#define SAL_SemaphoreCreate(v,w,x,y)    (SAL_Check4(gSALSyncObjectTable->fnSemaphoreCreate, v, w, x, y))
#define SAL_SemaphoreDelete(v)          (SAL_Check1(gSALSyncObjectTable->fnSemaphoreDelete, v))
#define SAL_SemaphoreRelease(v)         (SAL_Check1(gSALSyncObjectTable->fnSemaphoreRelease, v))
#define SAL_SemaphoreWait(v,w,x)        (SAL_Check3(gSALSyncObjectTable->fnSemaphoreWait, v, w, x))

/* Task */
#define SAL_TaskCreate(v,w,x,y,z,a,b)   (SAL_Check7(gSALTaskTable->fnTaskCreate, v, w, x, y, z, a, b))
#define SAL_TaskDelete(v)               (SAL_Check1(gSALTaskTable->fnTaskDelete, v))
#define SAL_TaskCurrent(v)              (SAL_Check1(gSALTaskTable->fnTaskCurrent, v))
#define SAL_TaskSleep(v)                (SAL_Check1(gSALTaskTable->fnTaskSleep, v))

/* Socket */
#define SAL_NetSocket(v)                (SAL_Check1(gSALSocketTable->fnNetSocket, v))
#define SAL_NetClose(v)                 (SAL_Check1(gSALSocketTable->fnNetClose, v))
#define SAL_NetConnect(v,w,x,y)         (SAL_Check4(gSALSocketTable->fnNetConnect, v, w, x, y))
#define SAL_NetBind(v,w)                (SAL_Check2(gSALSocketTable->fnNetBind, v, w))
#define SAL_NetSendTo(v,w,x,y,z)        (SAL_Check5(gSALSocketTable->fnNetSendTo, v, w, x, y, z))
#define SAL_NetSend(v,w,x)              (SAL_Check3(gSALSocketTable->fnNetSend, v, w, x))
#define SAL_NetRecvFrom(v,w,x,y,z,a)    (SAL_Check6(gSALSocketTable->fnNetRecvFrom, v, w, x, y, z, a))
#define SAL_NetRecv(v,w,x,y)            (SAL_Check4(gSALSocketTable->fnNetRecv, v, w, x, y))

/* Debugging */
#define SAL_DbgReportError(v,w,x,y)     (SAL_Check4(gSALTraceTable->fnReportError, v, w, x, y))
#define SAL_GetCurError(v)              (SAL_Check1(gSALTraceTable->fnGetCurrentError, v))

/* ETC */
/* Time */
#define SAL_GetTickCount(v)             (SAL_Check1(gSALEtcTable->fnGetTickCount, v))
#define SAL_HandleTick()                (SAL_Check0(gSALEtcTable->fnHandleTick))
#define SAL_GetVersion(v)               (SAL_Check1(gSALEtcTable->fnGetVersion, v))
#define SAL_GetSystemTickCount(v)       (SAL_Check1(gSALEtcTable->fnGetSystemTickCount, v))
#define SAL_IncreaseSystemTick(v)       (SAL_Check1(gSALEtcTable->fnIncreaseSystemTick, v))

#endif // SAL_INTERNAL_HEADER

