/**
  ******************************************************************************
  * @file    ll_aton_osal_utkernel.h
  * @brief   micro T-Kernel 3.0 OSAL for the ST Neural-ART (ATON) runtime.
  *
  * Implements the same macro contract as ll_aton_osal_freertos.h. The NPU
  * runtime uses four synchronisation objects:
  *
  *   DaoMutexNoWaiters  a try-lock, never blocks            -> owner task ID,
  *                                                             no kernel object
  *   DaoWaitQueue       queue of tasks waiting for the NPU  -> binary semaphore,
  *                                                             created EMPTY
  *   WfeSemaphore       "NPU epoch finished" signal from ISR-> counting semaphore
  *   CacheMutex         guards cache maintenance            -> mutex
  *
  * DaoMutexNoWaiters is only ever touched with dispatching disabled, and
  * tk_wai_sem() returns E_CTX in that state even with TMO_POL, so it cannot
  * be a semaphore here. DaoWaitQueue must start empty, as the FreeRTOS binary
  * semaphore does. If it started at 1, two tasks could own the NPU at once.
  * The .c implements the lock directly rather than including
  * ll_aton_osal_rtos_template.c. The macros below record the mapping for
  * side-by-side reading.
  *
  * PRIORITY INVERSION WARNING
  * --------------------------
  * FreeRTOS priorities count UP (larger number = higher priority). micro
  * T-Kernel priorities count DOWN (1 is the highest). Every priority
  * comparison in the runtime therefore has to be inverted -- see
  * _FirstPrioHigherThanScnd_ below. Copying the FreeRTOS macro verbatim
  * compiles cleanly and then mis-schedules the NPU under contention, which is
  * extremely hard to diagnose from the symptom.
  ******************************************************************************
  * @attention
  *
  * Port of ll_aton_osal_freertos.h from the STMicroelectronics Neural-ART
  * runtime (ThirdParty/AI_Runtime/Npu/ll_aton), Copyright (c) 2024 STMicroelectronics.
  * All rights reserved. Licensed under the terms in the LICENSE file in the
  * root directory of this software component (SLA0044 for ThirdParty/AI_Runtime).
  ******************************************************************************
  */

#ifndef __LL_ATON_OSAL_UTKERNEL_H
#define __LL_ATON_OSAL_UTKERNEL_H

#include <tk/tkernel.h>

#include "ll_aton_attributes.h"
#include "ll_aton_osal.h"
#include "ll_aton_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /* API functions -- called through the LL_ATON_OSAL_* macros */
  void aton_osal_utkernel_init(void);
  void aton_osal_utkernel_deinit(void);

  void aton_osal_utkernel_wfe(void);
  void aton_osal_utkernel_signal_event(void);

  void aton_osal_utkernel_dao_lock(void);
  void aton_osal_utkernel_dao_unlock(void);

  void aton_osal_utkernel_lock(void);
  void aton_osal_utkernel_unlock(void);

  /* Helpers implemented in the .c -- they wrap object creation so the macros
   * below stay one-liners. */
  BOOL utk_osal_create_empty_bin_sem(ID *id);
  BOOL utk_osal_create_cnt_sem(ID *id);
  BOOL utk_osal_create_mutex(ID *id);
  PRI  utk_osal_get_task_priority(ID tskid);

/*** type macros ***/
#define _DaoMutexNoWaitersType_ ID
#define _DaoWaitQueueType_      ID
#define _DaoWaitQueueValueType_ INT
#define _WfeSemaphoreType_      ID
#define _CacheMutexType_        ID
#define _TaskHandleType_        ID
#define _PriorityType_          PRI
#define _ReturnType_            BOOL

/*** values ***/
#define _OsTrue_     TRUE
#define _NullHandle_ 0

/* micro T-Kernel creates objects dynamically; there is no static-buffer form,
 * so the _static_buffer arguments are accepted and ignored. Object IDs are
 * small integers from a compile-time-sized table (CNF_MAX_SEMID etc. in
 * config), so this is still statically bounded -- see README. */
#define _CreateDaoMutexNoWaiters_(_obj, _buf) ((_obj) = 0, TRUE)
#define _CreateDaoWaitQueue_(_obj, _buf)      utk_osal_create_empty_bin_sem(&(_obj))
#define _CreateWfeSemaphore_(_obj, _buf)      utk_osal_create_cnt_sem(&(_obj))
#define _CreateCacheMutex_(_obj, _buf)        utk_osal_create_mutex(&(_obj))

/* The NPU interrupt must not run above the level at which kernel service calls
 * are safe. In micro T-Kernel that ceiling is the SysTick/PendSV arrangement
 * set by BSP2's sys_start.c, so use the same numeric level the kernel uses for
 * its own managed interrupts. */
#define _FinalizeIRQHandling_() LL_ATON_OSAL_SET_PRIORITY(ATON_STD_IRQ_LINE, MTK_MANAGED_IRQ_PRIORITY)

/* tk_dis_dsp() suspends dispatching without masking interrupts -- the exact
 * analogue of vTaskSuspendAll(). */
#define _DisablePreemption_() tk_dis_dsp()
#define _EnablePreemption_()  tk_ena_dsp()

#define _InitNonDao_()   /* nothing to do */
#define _DeInitNonDao_() /* nothing to do */

#define _GetCurrentTaskHandle_()        tk_get_tid()
#define _GetTaskPriority_(_task)        utk_osal_get_task_priority(_task)
#define _SetTaskPriority_(_task, _prio) tk_chg_pri((_task), (_prio))
#define _YieldCurrentTask_()            tk_rot_rdq(TPRI_RUN)

/* Owner-ID try-lock: valid only with dispatching disabled (see top). */
#define _MakeDaoMutexNoWaitersAvailable_(_m) ((_m) = 0)
#define _GetDaoMutexNoWaiters_(_m)           ((_m) == 0 ? ((_m) = tk_get_tid(), TRUE) : FALSE)
#define _ReleaseDaoMutexNoWaiters_(_m)       ((_m) = 0, TRUE)

#define _MakeDaoWaitQueueUnavailable_(_wq)
#define _GetDaoWaitQueueValue_(_wq) utk_osal_sem_count(_wq)
#define _GetDaoWaitQueue_(_wq)      tk_wai_sem((_wq), 1, TMO_FEVR)
#define _ReleaseDaoWaitQueue_(_wq)  tk_sig_sem((_wq), 1)

#define _MakeWfeSemaphoreUnavailable_(_s)
#define _GetWfeSemaphore_(_s)             tk_wai_sem((_s), 1, TMO_FEVR)
#define _ReleaseWfeSemaphore_(_s)         tk_sig_sem((_s), 1)
/* tk_sig_sem() is callable from the task-independent portion, so the ISR form
 * is identical and there is no "higher priority task woken" token to return. */
#define _ReleaseWfeSemaphoreISR_(_s, ...) tk_sig_sem((_s), 1)

#define _MakeCacheMutexAvailable_(_m)
#define _GetCacheMutex_(_m)     tk_loc_mtx((_m), TMO_FEVR)
#define _ReleaseCacheMutex_(_m) tk_unl_mtx(_m)

/* micro T-Kernel reschedules on return from the task-independent portion by
 * itself, so no explicit yield-from-ISR epilogue is needed. */
#define _HeadIsrCode_(...)
#define _TailIsrCode_(...)

/* INVERTED relative to FreeRTOS: in micro T-Kernel, SMALLER means HIGHER. */
#define _FirstPrioHigherThanScnd_(_first, _second) ((_first) < (_second))

  INT utk_osal_sem_count(ID semid);

#ifdef __cplusplus
}
#endif

#endif /* __LL_ATON_OSAL_UTKERNEL_H */
