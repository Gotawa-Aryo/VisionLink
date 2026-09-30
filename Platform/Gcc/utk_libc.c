/**
  ******************************************************************************
  * @file    utk_libc.c
  * @brief   newlib re-entrancy locks backed by a micro T-Kernel mutex.
  *
  * Replaces freertos_libc.c. newlib's malloc is not re-entrant; without these
  * two hooks, concurrent allocation from the nn and display tasks corrupts the
  * heap in a way that surfaces much later and elsewhere.
  ******************************************************************************
  * @attention
  *
  * Port of freertos_libc.c from STMicroelectronics' x-cube-n6-ai-hand-landmarks,
  * Copyright (c) 2024 STMicroelectronics. All rights reserved.
  * Licensed under the terms in Platform/LICENSE.md (SLA0044).
  ******************************************************************************
  */

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

#include "cmsis_compiler.h"
#include "os_utk.h"

#define IS_IRQ_MODE() (__get_IPSR() != 0U)

static os_mutex_t libc_lock;
static int is_rt_init_done;

/* Create the malloc lock. Called once the kernel is running. */
void utk_libc_init(void)
{
  int ret = os_mutex_create(&libc_lock);

  assert(ret == OS_OK);
  (void)ret;

  is_rt_init_done = 1;
}

/* newlib hook: one task at a time inside malloc and free. */
void __malloc_lock(struct _reent *reent)
{
  int ret;

  /* Allocations happen during early bring-up too, before the mutex exists.
   * Single-threaded at that point, so no lock is needed. */
  if (!is_rt_init_done)
    return;

  assert(!IS_IRQ_MODE());

  ret = os_mutex_lock(libc_lock);
  assert(ret == OS_OK);
  (void)ret;
}

/* newlib hook: release the malloc lock. */
void __malloc_unlock(struct _reent *reent)
{
  int ret;

  if (!is_rt_init_done)
    return;

  assert(!IS_IRQ_MODE());

  ret = os_mutex_unlock(libc_lock);
  assert(ret == OS_OK);
  (void)ret;
}
