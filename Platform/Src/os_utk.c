/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    os_utk.c
  * @brief   Thin OS abstraction over micro T-Kernel 3.0.
  ******************************************************************************
  */

#include "os_utk.h"

#include <assert.h>

/* BSP2 declares this nowhere public: sys_start.c defines it and the CubeIDE
 * templates call it straight out of main(). Declare it here. */
IMPORT void knl_start_mtkernel(void);

/* --- counting semaphore --------------------------------------------------- */

int os_sem_create(os_sem_t *sem, int initial, int max)
{
  T_CSEM csem = {
      .exinf   = NULL,
      .sematr  = TA_TFIFO | TA_FIRST,
      .isemcnt = initial,
      .maxsem  = max,
  };
  ID id = tk_cre_sem(&csem);

  if (id <= E_OK)
    return OS_ERROR;

  *sem = id;

  return OS_OK;
}

/* Take a semaphore, waiting forever or just polling. */
int os_sem_take(os_sem_t sem, int blocking)
{
  ER er = tk_wai_sem(sem, 1, blocking ? TMO_FEVR : TMO_POL);

  return (er == E_OK) ? OS_OK : OS_ERROR;
}

int os_sem_give(os_sem_t sem)
{
  ER er = tk_sig_sem(sem, 1);

  /* E_QOVR means the count is already at maxsem. For the "wake the consumer"
   * semaphores here that is harmless -- the consumer is already going to run --
   * so treat it as success rather than tripping an assert in an ISR. */
  return (er == E_OK || er == E_QOVR) ? OS_OK : OS_ERROR;
}

int os_sem_count(os_sem_t sem)
{
  T_RSEM rsem;

  if (tk_ref_sem(sem, &rsem) != E_OK)
    return 0;

  return rsem.semcnt;
}

/* --- mutex ---------------------------------------------------------------- */

int os_mutex_create(os_mutex_t *mtx)
{
  /* TA_INHERIT: priority inheritance, matching the FreeRTOS mutex this
   * replaces. Without it the low-priority display task can hold the display
   * lock while the higher-priority nn task waits on it. */
  T_CMTX cmtx = {
      .exinf   = NULL,
      .mtxatr  = TA_TFIFO | TA_INHERIT,
      .ceilpri = 0,
  };
  ID id = tk_cre_mtx(&cmtx);

  if (id <= E_OK)
    return OS_ERROR;

  *mtx = id;

  return OS_OK;
}

int os_mutex_lock(os_mutex_t mtx)
{
  return (tk_loc_mtx(mtx, TMO_FEVR) == E_OK) ? OS_OK : OS_ERROR;
}

int os_mutex_unlock(os_mutex_t mtx)
{
  return (tk_unl_mtx(mtx) == E_OK) ? OS_OK : OS_ERROR;
}

/* --- task ----------------------------------------------------------------- */

int os_task_create(os_task_t *task, void (*entry)(void *), const char *name,
                   void *stack, int stack_bytes, void *arg, PRI prio)
{
  T_CTSK ctsk = {
      .exinf   = arg,
      /* TA_HLNG   : entry uses the C calling convention
       * TA_RNG0   : run at privileged level, as the HAL and NPU driver require
       * TA_USERBUF: use the caller's stack buffer instead of the kernel heap
       * TA_FPU    : save/restore S16-S31 -- mandatory, every task here touches
       *             floats, and omitting it corrupts them silently */
      .tskatr  = TA_HLNG | TA_RNG0 | TA_USERBUF | TA_FPU,
      .task    = (FP)entry,
      .itskpri = prio,
      .stksz   = stack_bytes,
      .bufptr  = stack,
  };
  ID id;
  ER er;

  (void)name;

  id = tk_cre_tsk(&ctsk);
  if (id <= E_OK)
    return OS_ERROR;

  /* The entry point receives exinf as its argument, so the application's
   * void(*)(void *) signature carries through unchanged. */
  er = tk_sta_tsk(id, 0);
  if (er != E_OK) {
    tk_del_tsk(id);
    return OS_ERROR;
  }

  *task = id;

  return OS_OK;
}

void os_task_exit(void)
{
  tk_exd_tsk();  /* exit and delete self; does not return */
}

/* Sleep forever. usermain() calls this because it must not return. */
void os_park_forever(void)
{
  for (;;)
    tk_slp_tsk(TMO_FEVR);
}

/* --- time ----------------------------------------------------------------- */

uint32_t os_tick_ms(void)
{
  SYSTIM tim;

  if (tk_get_otm(&tim) != E_OK)
    return 0;

  /* SYSTIM is a 64-bit millisecond count split hi/lo. The application only
   * ever takes differences, so truncating to 32 bits is fine (49-day wrap,
   * same as FreeRTOS's tick count at 1 kHz). */
  return (uint32_t)tim.lo;
}

void os_delay_ms(uint32_t ms)
{
  tk_dly_tsk((RELTIM)ms);
}

/* --- scheduler ------------------------------------------------------------ */

void os_start(void)
{
  knl_start_mtkernel();   /* never returns; calls usermain() once running */
  assert(0);
  while (1)
    ;
}
