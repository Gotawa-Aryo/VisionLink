/**
  ******************************************************************************
  * @file    utk_bsp.c
  * @brief   HAL tick + TIM4 glue for micro T-Kernel 3.0.
  *
  * Replaces freertos_bsp.c. Three jobs:
  *   1. redirect HAL_GetTick / HAL_Delay onto the kernel
  *   2. re-arm SysTick after SystemClock_Config() changes the core clock
  *   3. start TIM4 and measure idle time for the CPU-load display
  ******************************************************************************
  * @attention
  *
  * Port of freertos_bsp.c from STMicroelectronics' x-cube-n6-ai-hand-landmarks,
  * Copyright (c) 2024 STMicroelectronics. All rights reserved.
  * Licensed under the terms in the LICENSE file in the root directory of this
  * software component (SLA0044 for Platform/).
  ******************************************************************************
  */

#include <assert.h>
#include <stdint.h>

#include "cmsis_compiler.h"
#include "os_utk.h"
#include "stm32n6xx_hal.h"

#if UTK_TRACE
void kdbgn(const char *, long);
extern volatile unsigned long irq_count_csi;
extern volatile unsigned long irq_count_dcmipp;
#endif

#define IS_IRQ_MODE() (__get_IPSR() != 0U)

static TIM_HandleTypeDef tim4;
static volatile int is_kernel_running;

/* Called once the kernel runs: HAL timing switches to the kernel tick. */
void utk_bsp_kernel_started(void)
{
  is_kernel_running = 1;
}

/*
 * The HAL is used during early bring-up, before any task exists. Until the
 * kernel is running there is no tick, so fall back to the cycle counter --
 * main.c enables DWT before anything else for exactly this reason.
 */
uint32_t HAL_GetTick(void)
{
  if (!is_kernel_running) {
    uint32_t hz = SystemCoreClock ? SystemCoreClock : 400000000u;

    return DWT->CYCCNT / (hz / 1000u);
  }

  return os_tick_ms();
}

/* Busy-wait before the kernel starts; after that, put the calling task to sleep. */
void HAL_Delay(uint32_t Delay)
{
  if (IS_IRQ_MODE())
    assert(0);

  if (!is_kernel_running) {
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < Delay)
      ;

    return;
  }

  os_delay_ms(Delay);
}

/* The kernel owns SysTick; the HAL must not touch it. */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  return HAL_OK;
}

/*
 * SystemClock_Config() runs after the kernel has started and changes
 * SystemCoreClock, which invalidates the SysTick reload the kernel programmed
 * at boot. FreeRTOS exposes vPortSetupTimerInterrupt() for this; micro
 * T-Kernel's equivalent is an Inline that bakes in the compile-time clock, so
 * re-arm from the live value instead.
 */
void utk_restart_systick(void)
{
  /* TIMER_PERIOD comes from the kernel's own configuration, so the hardware
   * and the kernel's idea of a tick cannot drift apart. Using a literal here
   * instead would make tk_get_otm() run fast or slow by that ratio. */
  uint32_t reload = (SystemCoreClock / 1000u) * TIMER_PERIOD - 1u;

  assert(reload <= 0x00FFFFFFu);   /* SysTick RVR is 24-bit */

  SysTick->CTRL = 0;
  SysTick->LOAD = reload;
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/* Start TIM4 as a free-running 100 kHz counter for the CPU-load figure. */
void TIM4_Config(void)
{
  const uint32_t tmr_clk_freq = 100000;
  int ret;

  __HAL_RCC_TIM4_CLK_ENABLE();

  tim4.Instance           = TIM4;
  tim4.Init.Period        = ~0;
  tim4.Init.Prescaler     = (HAL_RCC_GetPCLK1Freq() / tmr_clk_freq) - 1;
  tim4.Init.ClockDivision = 0;
  tim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
  ret = HAL_TIM_Base_Init(&tim4);
  assert(ret == HAL_OK);

  ret = HAL_TIM_Base_Start(&tim4);
  assert(ret == HAL_OK);
}

/* TIM4 count in 10 us ticks; 0 until TIM4_Config() has run. */
uint32_t TIM4_Get_Value(void)
{
  /* Under FreeRTOS this timer was started by the scheduler through
   * portCONFIGURE_TIMER_FOR_RUN_TIME_STATS(). That hook does not exist here,
   * so TIM4_Config() is called explicitly from main_thread_fct -- and until it
   * has run, tim4.Instance is NULL. The idle task polls this in a tight loop
   * from the moment the kernel starts, so without this guard it dereferences
   * NULL and takes a BusFault as soon as the main task first blocks. */
  if (tim4.Instance == NULL)
    return 0;

  return __HAL_TIM_GET_COUNTER(&tim4);
}

/* ---------------------------------------------------------------------------
 * CPU load measurement
 *
 * FreeRTOS offers portGET_RUN_TIME_COUNTER_VALUE() and
 * ulTaskGetIdleRunTimeCounter(). micro T-Kernel keeps no per-task run-time
 * accounting, so idle time is measured directly: a task at the lowest
 * priority samples the free-running 100 kHz TIM4 in a tight loop.
 *
 * While that task is the one running, consecutive samples differ by a tick or
 * two and the deltas telescope into the true elapsed idle time. When a real
 * task preempts it, the next delta spans the whole preemption -- so deltas
 * larger than IDLE_DELTA_CAP are discarded rather than counted. The result
 * under-reports idle by at most one cap per preemption (20 us), which is well
 * inside the noise of a figure displayed to one decimal place.
 * ------------------------------------------------------------------------- */

#define IDLE_DELTA_CAP 2u   /* TIM4 ticks; 1 tick = 10 us */

static volatile uint32_t idle_runtime;

uint32_t utk_get_total_runtime(void)
{
  return TIM4_Get_Value();
}

uint32_t utk_get_idle_runtime(void)
{
  return idle_runtime;
}

/* Idle task: adds up idle time for the CPU-load figure. */
void utk_idle_task_fct(void *arg)
{
  uint32_t prev = TIM4_Get_Value();
#if UTK_TRACE
  /* Heartbeat: the idle task runs precisely when every other task is
   * blocked, so this reports liveness even when the pipeline is stalled.
   * Frame counters tell us whether the camera is actually delivering. */
  uint32_t last_beat = prev;
#endif

  (void)arg;

  for (;;) {
    uint32_t now = TIM4_Get_Value();
    uint32_t delta = now - prev;

    prev = now;
    if (delta <= IDLE_DELTA_CAP)
      idle_runtime += delta;

    /* The idle task must make no kernel calls. HAL_GetTick() calls
     * tk_get_otm(), which masks interrupts via disint(); calling that every
     * iteration of a tight lowest-priority loop leaves the CPU interrupt-masked
     * a large fraction of the time, delaying or dropping the DCMIPP/CSI frame
     * interrupts the camera pipeline depends on. Hence TIM4 above. */
#if UTK_TRACE
    /* The one exception, in trace builds only: kdbgn() goes through
     * HAL_UART_Transmit(), whose timeout uses HAL_GetTick(). Once per 2 s is
     * harmless; it is why this is not left on in normal builds. */
    if ((now - last_beat) >= 200000u) {   /* 200000 * 10 us = 2 s */
      last_beat = now;
      kdbgn("HB dcmipp_irq=", (long)irq_count_dcmipp);
      kdbgn("HB csi_irq=", (long)irq_count_csi);
      /* Are they even enabled in the NVIC? Zero counts mean either the
       * peripheral never fires, or the line was never unmasked. */
      kdbgn("HB dcmipp_en=", (long)NVIC_GetEnableIRQ(DCMIPP_IRQn));
      kdbgn("HB csi_en=", (long)NVIC_GetEnableIRQ(CSI_IRQn));
    }
#endif /* UTK_TRACE */
  }
}
