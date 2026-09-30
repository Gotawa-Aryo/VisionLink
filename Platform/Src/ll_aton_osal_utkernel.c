/**
  ******************************************************************************
  * @file    ll_aton_osal_utkernel.c
  * @brief   micro T-Kernel 3.0 OSAL for the ST Neural-ART (ATON) runtime.
  *
  * Object-creation helpers plus the eight entry points the runtime calls.
  * The structure mirrors ll_aton_osal_freertos.c so the two can be diffed.
  ******************************************************************************
  * @attention
  *
  * Port of ll_aton_osal_freertos.c from the STMicroelectronics Neural-ART
  * runtime (ThirdParty/AI_Runtime/Npu/ll_aton), Copyright (c) 2024 STMicroelectronics.
  * All rights reserved. Licensed under the terms in the LICENSE file in the
  * root directory of this software component (SLA0044 for ThirdParty/AI_Runtime).
  ******************************************************************************
  */

#include "ll_aton_osal_utkernel.h"

#include "ll_aton_attributes.h"
#include "ll_aton_osal.h"
#include "ll_aton_platform.h"
#include "ll_aton_runtime.h"

/* --- object handles ------------------------------------------------------ */
static ID dao_wait_queue;        /* binary, created empty */
static ID wfe_semaphore;         /* counting */
static ID cache_mutex;           /* mutex    */
static BOOL is_initialized;

/* --- deferred ATON owner (DAO) state -------------------------------------
 * Read and written only with dispatching disabled. dao_owner doubles as ST's
 * _dao_mutex try-lock: see aton_osal_utkernel_dao_lock(). */
static ID            dao_owner;          /* 0 = NPU free */
static PRI           dao_owner_orig_pri; /* to undo priority inheritance */
static volatile UINT dao_nr_waiters;     /* tasks counted onto dao_wait_queue */

/* --- creation helpers ---------------------------------------------------- */

/* Created EMPTY, like xSemaphoreCreateBinaryStatic(). The DAO wait queue
 * depends on this: if it started at 1, the first task to find the NPU busy
 * would pass straight through tk_wai_sem() and two tasks would own the NPU. */
BOOL utk_osal_create_empty_bin_sem(ID *id)
{
  T_CSEM csem = {
      .exinf   = NULL,
      .sematr  = TA_TFIFO | TA_FIRST,
      .isemcnt = 0,   /* created unavailable: first waiter blocks */
      .maxsem  = 1,
  };
  ID sid = tk_cre_sem(&csem);

  if (sid <= E_OK)
    return FALSE;

  *id = sid;

  return TRUE;
}

BOOL utk_osal_create_cnt_sem(ID *id)
{
  T_CSEM csem = {
      .exinf   = NULL,
      .sematr  = TA_TFIFO | TA_FIRST,
      .isemcnt = 0,      /* created unavailable: first waiter blocks */
      .maxsem  = 0x7FFF, /* effectively unbounded, as FreeRTOS uses 0xFFFFFFFF */
  };
  ID sid = tk_cre_sem(&csem);

  if (sid <= E_OK)
    return FALSE;

  *id = sid;

  return TRUE;
}

BOOL utk_osal_create_mutex(ID *id)
{
  /* TA_INHERIT gives priority inheritance, matching the FreeRTOS mutex the
   * reference OSAL creates with xSemaphoreCreateMutexStatic(). Without it a
   * low-priority task holding the cache mutex can stall NPU completion. */
  T_CMTX cmtx = {
      .exinf  = NULL,
      .mtxatr = TA_TFIFO | TA_INHERIT,
      .ceilpri = 0,
  };
  ID mid = tk_cre_mtx(&cmtx);

  if (mid <= E_OK)
    return FALSE;

  *id = mid;

  return TRUE;
}

INT utk_osal_sem_count(ID semid)
{
  T_RSEM rsem;

  if (tk_ref_sem(semid, &rsem) != E_OK)
    return 0;

  return rsem.semcnt;
}

PRI utk_osal_get_task_priority(ID tskid)
{
  T_RTSK rtsk;

  if (tk_ref_tsk(tskid, &rtsk) != E_OK)
    return 0;

  return rtsk.tskpri;
}

/* --- runtime entry points ------------------------------------------------ */

void aton_osal_utkernel_init(void)
{
  BOOL ok;

  if (is_initialized)
    return;

  dao_owner          = 0;
  dao_owner_orig_pri = 0;
  dao_nr_waiters     = 0;

  ok = utk_osal_create_empty_bin_sem(&dao_wait_queue);
  LL_ATON_ASSERT(ok);
  ok = utk_osal_create_cnt_sem(&wfe_semaphore);
  LL_ATON_ASSERT(ok);
  ok = utk_osal_create_mutex(&cache_mutex);
  LL_ATON_ASSERT(ok);

  is_initialized = TRUE;
}

void aton_osal_utkernel_deinit(void)
{
  if (!is_initialized)
    return;

  tk_del_sem(dao_wait_queue);
  tk_del_sem(wfe_semaphore);
  tk_del_mtx(cache_mutex);

  dao_wait_queue = wfe_semaphore = cache_mutex = 0;
  dao_owner = 0;
  dao_owner_orig_pri = 0;
  dao_nr_waiters = 0;
  is_initialized = FALSE;
}

/* Block until the NPU signals epoch completion from its ISR. */
void aton_osal_utkernel_wfe(void)
{
  tk_wai_sem(wfe_semaphore, 1, TMO_FEVR);
}

/* Called from the ATON ISR. */
void aton_osal_utkernel_signal_event(void)
{
  tk_sig_sem(wfe_semaphore, 1);
}

/* "Deferred ATON owner" lock, ported from ll_aton_osal_rtos_template.c.
 *
 * Unlock does not hand the NPU to a waiter. It wakes one waiter to retry, so
 * a higher-priority task that arrives meanwhile can take the NPU first. While
 * the NPU is held, a higher-priority waiter lifts the owner to its own
 * priority. Unlock restores it. This hand-made inheritance assumes nothing
 * else changes an inference task's priority during an epoch block.
 *
 * One deliberate difference from ST: the template takes _dao_mutex, a
 * semaphore, with preemption disabled. With CHK_CTX on (config.h),
 * tk_wai_sem() returns E_CTX while dispatching is disabled, even with
 * TMO_POL, so that try-lock would never succeed. Here "dao_owner != 0" is the
 * lock instead. ST sets and clears _current_aton_owner in the same critical
 * sections as _dao_mutex, so the two are equivalent. */
void aton_osal_utkernel_dao_lock(void)
{
  ID  self;
  PRI self_pri;
  ER  ercd;

  tk_dis_dsp();

  self = tk_get_tid();
  LL_ATON_ASSERT(self != 0);

  for (;;) {
    /* Re-read on every pass: it may have changed while we waited. */
    self_pri = utk_osal_get_task_priority(self);

    if (dao_owner == 0) {
      dao_owner          = self;
      dao_owner_orig_pri = self_pri;
      tk_ena_dsp();
      return;
    }

    LL_ATON_ASSERT(dao_owner != self);
    if (_FirstPrioHigherThanScnd_(self_pri, utk_osal_get_task_priority(dao_owner)))
      tk_chg_pri(dao_owner, self_pri);

    /* Count ourselves before re-enabling dispatch. An unlock that runs
     * before we reach tk_wai_sem() still sees us and leaves the semaphore
     * at 1, so the wake-up is not lost. */
    dao_nr_waiters++;
    tk_ena_dsp();

    ercd = tk_wai_sem(dao_wait_queue, 1, TMO_FEVR);
    LL_ATON_ASSERT(ercd == E_OK);
    LL_ATON_LIB_UNUSED(ercd);

    tk_dis_dsp();
    LL_ATON_ASSERT(dao_nr_waiters > 0);
    dao_nr_waiters--;
    /* retry */
  }
}

void aton_osal_utkernel_dao_unlock(void)
{
  BOOL do_yield = FALSE;
  INT  wq_count;

  tk_dis_dsp();

  LL_ATON_ASSERT(dao_owner == tk_get_tid());
  if (utk_osal_get_task_priority(dao_owner) != dao_owner_orig_pri)
    tk_chg_pri(dao_owner, dao_owner_orig_pri); /* undo inheritance */
  dao_owner          = 0;
  dao_owner_orig_pri = 0;

  /* Wake one waiter to retry, unless a wake-up is already pending. */
  wq_count = utk_osal_sem_count(dao_wait_queue);
  LL_ATON_ASSERT(wq_count <= 1);
  if (dao_nr_waiters > 0) {
    do_yield = TRUE;
    if (wq_count == 0)
      tk_sig_sem(dao_wait_queue, 1);
  }

  tk_ena_dsp();

  /* Let a same-priority waiter run now, not at the next rotation. */
  if (do_yield)
    tk_rot_rdq(TPRI_RUN);
}

void aton_osal_utkernel_lock(void)
{
  tk_loc_mtx(cache_mutex, TMO_FEVR);
}

void aton_osal_utkernel_unlock(void)
{
  tk_unl_mtx(cache_mutex);
}
