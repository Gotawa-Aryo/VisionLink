/* SPDX-License-Identifier: Apache-2.0 */
/**
  ******************************************************************************
  * @file    ll_aton_osal_user_impl.h
  * @brief   Custom-OSAL hook for the ATON runtime.
  *
  * ll_aton_osal.h includes this exact filename when LL_ATON_OSAL is set to
  * LL_ATON_OSAL_USER_IMPL. It binds the runtime's generic macros to the micro
  * T-Kernel implementation. Anything left undefined here falls back to the
  * default in ll_aton_osal.h (IRQ install/enable via CMSIS NVIC, etc.).
  ******************************************************************************
  */

#ifndef __LL_ATON_OSAL_USER_IMPL_H
#define __LL_ATON_OSAL_USER_IMPL_H

#include "ll_aton_osal_utkernel.h"

#define LL_ATON_OSAL_INIT()   aton_osal_utkernel_init()
#define LL_ATON_OSAL_DEINIT() aton_osal_utkernel_deinit()

#define LL_ATON_OSAL_WFE()          aton_osal_utkernel_wfe()
#define LL_ATON_OSAL_SIGNAL_EVENT() aton_osal_utkernel_signal_event()

#if APP_HAS_PARALLEL_NETWORKS
#define LL_ATON_OSAL_LOCK_ATON()   aton_osal_utkernel_dao_lock()
#define LL_ATON_OSAL_UNLOCK_ATON() aton_osal_utkernel_dao_unlock()
#endif /* APP_HAS_PARALLEL_NETWORKS */

#define LL_ATON_OSAL_LOCK_NPU_CACHE()   aton_osal_utkernel_lock()
#define LL_ATON_OSAL_UNLOCK_NPU_CACHE() aton_osal_utkernel_unlock()

#endif /* __LL_ATON_OSAL_USER_IMPL_H */
