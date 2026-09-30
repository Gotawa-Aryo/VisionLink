/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    os_utk.h
  * @brief   Thin OS abstraction over micro T-Kernel 3.0.
  *
  * The application was written against FreeRTOS. Rather than scatter tk_*
  * calls through app.c, this layer names the handful of primitives it actually
  * uses. Everything here is a direct, thin mapping -- no emulation, no hidden
  * queues.
  *
  * PRIORITY DIRECTION
  * ------------------
  * FreeRTOS: larger number = higher priority.
  * micro T-Kernel: SMALLER number = higher priority (1 is the highest).
  *
  * The application expresses priorities as a signed offset where positive
  * means "more urgent" (isp +2, nn +1, dp -2). OS_PRIORITY() performs the
  * inversion in exactly one place so the call sites keep reading the way their
  * author intended.
  ******************************************************************************
  */

#ifndef OS_UTK_H
#define OS_UTK_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Mid-scale base. micro T-Kernel supports 1..CNF_MAX_TSKPRI; the kernel's own
 * init task sits well above this, so offsets of +/-5 stay in range. */
#define OS_PRIORITY_BASE 20
#define OS_PRIORITY(p)   ((PRI)(OS_PRIORITY_BASE - (p)))

/* Bytes, not words -- unlike FreeRTOS's configMINIMAL_STACK_SIZE, whose unit is
 * StackType_t (uint32_t). The application's FreeRTOSConfig.h sets it to 1024,
 * i.e. 1024 * 4 = 4096 BYTES. Declaring 1024 here gave every task a quarter of
 * the stack it used to have; main_thread_fct() runs deep into the HAL and the
 * camera middleware with large local structs and overflowed silently, which
 * presents as the scheduler mysteriously never resuming a delayed task. */
#define OS_MINIMAL_STACK_SIZE 4096

#define OS_OK    0
#define OS_ERROR (-1)

#define OS_BLOCKING     1
#define OS_NON_BLOCKING 0

typedef ID os_sem_t;
typedef ID os_mutex_t;
typedef ID os_task_t;

/* --- counting semaphore --------------------------------------------------- */
int os_sem_create(os_sem_t *sem, int initial, int max);
/** @retval OS_OK if taken. With OS_NON_BLOCKING, OS_ERROR means "was zero". */
int os_sem_take(os_sem_t sem, int blocking);
/** Safe from both task and interrupt context: tk_sig_sem() is callable from
 *  the task-independent portion, so no separate FromISR variant is needed. */
int os_sem_give(os_sem_t sem);
int os_sem_count(os_sem_t sem);

/* --- mutex (priority inheritance) ---------------------------------------- */
int os_mutex_create(os_mutex_t *mtx);
int os_mutex_lock(os_mutex_t mtx);
int os_mutex_unlock(os_mutex_t mtx);

/* --- task ----------------------------------------------------------------- */
/**
  * @param stack  caller-owned stack buffer (TA_USERBUF), keeping the static
  *               allocation the application already relies on
  * @param prio   result of OS_PRIORITY()
  */
int os_task_create(os_task_t *task, void (*entry)(void *), const char *name,
                   void *stack, int stack_bytes, void *arg, PRI prio);
void os_task_exit(void);

/**
  * @brief  Park the calling task forever.
  *
  * Required at the end of usermain(). usermain() runs in the kernel's initial
  * task, and inittask.c treats its RETURN VALUE AS A SHUTDOWN CODE -- 0 means
  * "power off". Returning therefore tears the system down and halts the CPU
  * with interrupts masked, before any task created in usermain() has ever been
  * dispatched. There is no FreeRTOS equivalent of this: there,
  * vTaskStartScheduler() never returns and the creating context simply ends.
  */
void os_park_forever(void);

/* --- time ----------------------------------------------------------------- */
uint32_t os_tick_ms(void);
void     os_delay_ms(uint32_t ms);

/* --- scheduler ------------------------------------------------------------ */
void os_start(void);   /* never returns */

#endif /* OS_UTK_H */
