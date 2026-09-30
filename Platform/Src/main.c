 /**
 ******************************************************************************
 * @file    main.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <assert.h>

#include "app.h"
#include "app_config.h"
#include "app_fuseprogramming.h"
#include "main.h"
#include "npu_cache.h"
#ifdef STM32N6570_DK_REV
#include "stm32n6570_discovery.h"
#include "stm32n6570_discovery_bus.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32n6570_discovery_xspi.h"
#else
#include "stm32n6xx_nucleo.h"
#include "stm32n6xx_nucleo_bus.h"
#include "stm32n6xx_nucleo_xspi.h"
#endif
#include <stdio.h>
#include "stm32n6xx_hal_rif.h"
#include "os_utk.h"

/* utk_bsp.c / utk_libc.c */
void utk_bsp_kernel_started(void);
void utk_restart_systick(void);
void utk_libc_init(void);
void utk_idle_task_fct(void *arg);
void TIM4_Config(void);

UART_HandleTypeDef huart1;
GPU2D_HandleTypeDef hgpu2d;

static os_task_t main_thread;
static uint8_t main_thread_stack[OS_MINIMAL_STACK_SIZE] __attribute__((aligned(8)));
static os_task_t idle_thread;
static uint8_t idle_thread_stack[OS_MINIMAL_STACK_SIZE] __attribute__((aligned(8)));

static void SystemClock_Config(void);
static void NPURam_enable();
static void NPUCache_config();
static void Security_Config();
static void IAC_Config();
static void CONSOLE_Config(void);
/* Malloc-free trace hook, called from inside the vendored kernel when built
 * with -DMTK_TRACE. Avoids printf, which buffers and can allocate -- not safe
 * mid-kernel-bring-up. Requires CONSOLE_Config() to have run. */
void kdbg(const char *s)
{
  static const uint8_t crlf[2] = { 13, 10 };
  const char *p = s;
  int n = 0;

  while (*p++) n++;
  HAL_UART_Transmit(&huart1, (uint8_t *)s, n, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t *)crlf, 2, 1000);
}

/* kdbg() with a number appended. Deliberately not printf: newlib buffers, and
 * a buffered line never reaches the UART if the next call hangs. */
void kdbgn(const char *tag, long v)
{
  char buf[24];
  int i = sizeof(buf) - 1;
  int neg = (v < 0);
  unsigned long u = neg ? (unsigned long)(-v) : (unsigned long)v;

  buf[i] = 0;
  do { buf[--i] = (char)('0' + (u % 10)); u /= 10; } while (u && i > 1);
  if (neg && i > 0) buf[--i] = '-';

  kdbg(tag);
  kdbg(&buf[i]);
}

static int main_utkernel(void);
static void main_thread_fct(void *arg);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Power on ICACHE */
  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_ICACTIVE_Msk;

  /* Enable DWT early: utk_bsp.c uses the cycle counter as the HAL tick source
   * until the kernel is running. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Set back system and CPU clock source to HSI */
  __HAL_RCC_CPUCLK_CONFIG(RCC_CPUCLKSOURCE_HSI);
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

  HAL_Init();

  SCB_EnableICache();

#if defined(USE_DCACHE)
  /* Power on DCACHE */
  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_DCACTIVE_Msk;
  SCB_EnableDCache();
#endif

  /* Console early, before the kernel: without it the MTK_TRACE output from
   * kernel bring-up has nowhere to go. Re-initialised later by
   * main_thread_fct() once the clocks are at their final speed. */
  CONSOLE_Config();

  return main_utkernel();
}

/* Power the NPU and its four AXI SRAM banks. */
static void NPURam_enable()
{
  __HAL_RCC_NPU_CLK_ENABLE();
  __HAL_RCC_NPU_FORCE_RESET();
  __HAL_RCC_NPU_RELEASE_RESET();

  /* Enable NPU RAMs (4x448KB) */
  __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
  __HAL_RCC_RAMCFG_CLK_ENABLE();
  RAMCFG_HandleTypeDef hramcfg = {0};
  hramcfg.Instance =  RAMCFG_SRAM3_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM4_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM5_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
  hramcfg.Instance =  RAMCFG_SRAM6_AXI;
  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
}


/* Enable the NPU cache. */
static void NPUCache_config()
{
  npu_cache_init();
  npu_cache_enable();
}

/* Give the NPU, camera, display, GPU and USB secure, privileged bus access. */
static void Security_Config()
{
  __HAL_RCC_RIFSC_CLK_ENABLE();
  RIMC_MasterConfig_t RIMC_master = {0};
  RIMC_master.MasterCID = RIF_CID_1;
  RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DMA2D, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1 , &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2 , &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_OTG1 , &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_GPU2D , &RIMC_master);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_GFXMMU , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_GPU2D , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_ICACHE , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DMA2D , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI    , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC   , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_OTG1HS , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_SPI5 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
}

/* Reconfigure the ICACHE as 4-way. */
static void GPU_CACHE_Enable(void)
{
    /* DeInit ICACHE */
    HAL_ICACHE_DeInit();

    /* Disable ICACHE */
    HAL_ICACHE_Disable();

    /* Configure ICACHE with selected configuration */
    HAL_ICACHE_ConfigAssociativityMode(ICACHE_4WAYS);

    /* Enable ICACHE */
    HAL_ICACHE_Enable();
}

/* Initialise the GPU2D that rotates the hand crops. */
static void GPU2D_Init(void)
{
    hgpu2d.Instance = (uint32_t)GPU2D;

    if (HAL_GPU2D_Init(&hgpu2d) != HAL_OK)
    {
        assert(0);
    }
}

/* Enable the illegal-access controller. */
static void IAC_Config(void)
{
/* Configure IAC to trap illegal access events */
  __HAL_RCC_IAC_CLK_ENABLE();
  __HAL_RCC_IAC_FORCE_RESET();
  __HAL_RCC_IAC_RELEASE_RESET();
}

/* Illegal access trapped: say so on the console and halt. */
void IAC_IRQHandler(void)
{
  /* The Illegal Access Controller fired. Without this the handler is a silent
   * while(1) and the symptom is indistinguishable from a hang in whatever the
   * task was doing. */
  kdbg("!!! IAC illegal access trapped");
  while (1)
  {
  }
}

/* Final clock tree: CPU at 800 MHz, plus the NPU, bus and peripheral clocks. */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};

  BSP_SMPS_Init(SMPS_VOLTAGE_OVERDRIVE);
  HAL_Delay(1); /* Assuming Voltage Ramp Speed of 1mV/us --> 100mV increase takes 100us */

  // Oscillator config already done in bootrom
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;

  /* PLL1 = 64 x 25 / 2 = 800MHz */
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 2;
  RCC_OscInitStruct.PLL1.PLLN = 25;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1 = 1;
  RCC_OscInitStruct.PLL1.PLLP2 = 1;

  /* PLL2 = 64 x 125 / 8 = 1000MHz */
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL2.PLLM = 8;
  RCC_OscInitStruct.PLL2.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLN = 125;
  RCC_OscInitStruct.PLL2.PLLP1 = 1;
  RCC_OscInitStruct.PLL2.PLLP2 = 1;

  /* PLL3 = (64 x 225 / 8) / (1 * 2) = 900MHz */
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL3.PLLM = 8;
  RCC_OscInitStruct.PLL3.PLLN = 225;
  RCC_OscInitStruct.PLL3.PLLFractional = 0;
  RCC_OscInitStruct.PLL3.PLLP1 = 1;
  RCC_OscInitStruct.PLL3.PLLP2 = 2;

  /* PLL4 = (64 x 225 / 8) / (6 * 6) = 50 MHz */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL4.PLLM = 8;
  RCC_OscInitStruct.PLL4.PLLFractional = 0;
  RCC_OscInitStruct.PLL4.PLLN = 225;
  RCC_OscInitStruct.PLL4.PLLP1 = 6;
  RCC_OscInitStruct.PLL4.PLLP2 = 6;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                                 RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4 |
                                 RCC_CLOCKTYPE_PCLK5);

  /* CPU CLock (sysa_ck) = ic1_ck = PLL1 output/ic1_divider = 800 MHz */
  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider = 1;

  /* AXI Clock (sysb_ck) = ic2_ck = PLL1 output/ic2_divider = 400 MHz */
  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider = 2;

  /* NPU Clock (sysc_ck) = ic6_ck = PLL2 output/ic6_divider = 1000 MHz */
  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL2;
  RCC_ClkInitStruct.IC6Selection.ClockDivider = 1;

  /* AXISRAM3/4/5/6 Clock (sysd_ck) = ic11_ck = PLL3 output/ic11_divider = 900 MHz */
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL3;
  RCC_ClkInitStruct.IC11Selection.ClockDivider = 1;

  /* HCLK = sysb_ck / HCLK divider = 200 MHz */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;

  /* PCLKx = HCLK / PCLKx divider = 200 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    while(1);
  }

  RCC_PeriphCLKInitStruct.PeriphClockSelection = 0;

  /* XSPI1 kernel clock (ck_ker_xspi1) = HCLK = 200MHz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI1;
  RCC_PeriphCLKInitStruct.Xspi1ClockSelection = RCC_XSPI1CLKSOURCE_HCLK;

  /* XSPI2 kernel clock (ck_ker_xspi1) = HCLK =  200MHz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI2;
  RCC_PeriphCLKInitStruct.Xspi2ClockSelection = RCC_XSPI2CLKSOURCE_HCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {
    while (1);
  }
}

/* USART1 console on the ST-LINK virtual COM port, 115200 8N1. */
static void CONSOLE_Config()
{
  GPIO_InitTypeDef gpio_init;

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

 /* DISCO & NUCLEO USART1 (PE5/PE6) */
  gpio_init.Mode      = GPIO_MODE_AF_PP;
  gpio_init.Pull      = GPIO_PULLUP;
  gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
  gpio_init.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOE, &gpio_init);

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    while (1);
  }
}

/*
 * Entry point for the user application, called by micro T-Kernel's initial
 * task once the kernel is up. The contract is that usermain() only creates
 * tasks and returns -- it must not block, because the initial task still has
 * work to do after it.
 */
EXPORT INT usermain(void)
{
  int ret;

  utk_bsp_kernel_started();
  utk_libc_init();

  ret = os_task_create(&main_thread, main_thread_fct, "main", main_thread_stack,
                       sizeof(main_thread_stack), NULL, OS_PRIORITY(0));
  assert(ret == OS_OK);

  /* Lowest priority of anything in the system: it only runs when nothing else
   * can, which is what makes it a valid idle-time measurement. */
  ret = os_task_create(&idle_thread, utk_idle_task_fct, "idle", idle_thread_stack,
                       sizeof(idle_thread_stack), NULL, OS_PRIORITY(-5));
  assert(ret == OS_OK);
  (void)ret;

  /* Must not return -- see os_park_forever(). Returning from usermain() is a
   * shutdown request and halts the CPU before any task above ever runs. */
  os_park_forever();

  return 0;   /* unreachable */
}

/* Start micro T-Kernel, which calls usermain(). Never returns. */
static int main_utkernel(void)
{
  /* Starts the kernel, which calls usermain() above. Never returns. */
  os_start();
  assert(0);

  return -1;
}

/* First task: board bring-up (clocks, memories, NPU, security), then app_run(). */
static void main_thread_fct(void *arg)
{
  IRQn_Type i;

  /* Copy SysTick_IRQn priority set by RTOS and use it as default priorities for IRQs. We are now sure that all irqs
   * have default priority below or equal to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
   */
  /* micro T-Kernel BSP2's sys_start.c reprograms AIRCR PRIGROUP to 0. The
   * STM32 HAL only recognises 3..7 (NVIC_PRIORITYGROUP_4..0), so with
   * USE_FULL_ASSERT every HAL_NVIC_* call trips
   *     assert_param(IS_NVIC_PRIORITY_GROUP(...)) -> assert_failed() -> while(1)
   * and the application hangs on its first NVIC call. Under FreeRTOS this
   * never showed, because HAL_Init() left the grouping at NVIC_PRIORITYGROUP_4.
   *
   * Restoring it is behaviourally free: this device implements 4 priority bits,
   * so PRIGROUP 0 and NVIC_PRIORITYGROUP_4 both mean "all bits preemption, no
   * sub-priority". Only the HAL's idea of what is valid differs. */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Put every peripheral IRQ at a level the kernel can mask. disint() sets
   * BASEPRI to INTPRI_MAX_EXTINT_PRI (1), so anything numerically >= 1 is held
   * off inside kernel critical sections. Copying SysTick's own priority -- what
   * the FreeRTOS version did -- is wrong here: BSP2 puts SysTick at 1, the
   * highest level the kernel permits, which would leave no headroom. */
  for (i = PVD_PVM_IRQn; i <= LTDC_UP_ERR_IRQn; i++)
    HAL_NVIC_SetPriority(i, MTK_MANAGED_IRQ_PRIORITY, 0);


  /* Call SystemClock_Config() after vTaskStartScheduler() since it call HAL_Delay() which call vTaskDelay(). Drawback
   * is that we must call vPortSetupTimerInterrupt() since SystemCoreClock value has been modified by SystemClock_Config()
   */
  SystemClock_Config();
  /* SystemCoreClock just changed, so the kernel's SysTick reload is stale. */
  utk_restart_systick();

  /* FreeRTOS started this via portCONFIGURE_TIMER_FOR_RUN_TIME_STATS(); with
   * FreeRTOS gone nothing did, and the CPU-load counters read a NULL handle.
   * Must come after SystemClock_Config() -- the prescaler is derived from
   * PCLK1. */
  TIM4_Config();

  CONSOLE_Config();


  NPURam_enable();
  Fuse_Programming();

  GPU2D_Init();
  GPU_CACHE_Enable();

  NPUCache_config();

#ifdef STM32N6570_DK_REV
  /*** External RAM and NOR Flash *********************************************/
  BSP_XSPI_RAM_Init(0);
  BSP_XSPI_RAM_EnableMemoryMappedMode(0);
#endif

  BSP_XSPI_NOR_Init_t NOR_Init;
  NOR_Init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
  NOR_Init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;
  BSP_XSPI_NOR_Init(0, &NOR_Init);
  BSP_XSPI_NOR_EnableMemoryMappedMode(0);

  /* Set all required IPs as secure privileged */
  Security_Config();

  IAC_Config();

  /* Keep all IP's enabled during WFE so they can wake up CPU. Fine tune
   * this if you want to save maximum power
   */
  LL_BUS_EnableClockLowPower(~0);
  LL_MEM_EnableClockLowPower(~0);
  LL_AHB1_GRP1_EnableClockLowPower(~0);
  LL_AHB2_GRP1_EnableClockLowPower(~0);
  LL_AHB3_GRP1_EnableClockLowPower(~0);
  LL_AHB4_GRP1_EnableClockLowPower(~0);
  LL_AHB5_GRP1_EnableClockLowPower(~0);
  LL_APB1_GRP1_EnableClockLowPower(~0);
  LL_APB1_GRP2_EnableClockLowPower(~0);
  LL_APB2_GRP1_EnableClockLowPower(~0);
  LL_APB4_GRP1_EnableClockLowPower(~0);
  LL_APB4_GRP2_EnableClockLowPower(~0);
  LL_APB5_GRP1_EnableClockLowPower(~0);
  LL_MISC_EnableClockLowPower(~0);

  app_run();

  os_task_exit();
}

/* Camera interface clock; called by the camera middleware. */
HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp)
{
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};
  HAL_StatusTypeDef ret;

  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DCMIPP;
  RCC_PeriphCLKInitStruct.DcmippClockSelection = RCC_DCMIPPCLKSOURCE_IC17;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL2;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockDivider = 3;
  ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  if (ret)
    return ret;

  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CSI;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockDivider = 40;
  ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  if (ret)
    return ret;

  return HAL_OK;
}

void HAL_CACHEAXI_MspInit(CACHEAXI_HandleTypeDef *hcacheaxi)
{
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_CLK_ENABLE();
  __HAL_RCC_CACHEAXI_FORCE_RESET();
  __HAL_RCC_CACHEAXI_RELEASE_RESET();
}

void HAL_CACHEAXI_MspDeInit(CACHEAXI_HandleTypeDef *hcacheaxi)
{
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_DISABLE();
  __HAL_RCC_CACHEAXI_CLK_DISABLE();
  __HAL_RCC_CACHEAXI_FORCE_RESET();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  UNUSED(file);
  UNUSED(line);
  __BKPT(0);
  while (1)
  {
  }
}
#endif

/* Allow to debug with cache enable */
__attribute__ ((section (".keep_me"))) void app_clean_invalidate_dbg()
{
  SCB_CleanInvalidateDCache();
}

void HAL_GPU2D_MspInit(GPU2D_HandleTypeDef* hgpu2d)
{
  if(hgpu2d->Instance==(GPU2D_TypeDef)GPU2D)
  {
    __HAL_RCC_GPU2D_FORCE_RESET();
    __HAL_RCC_GPU2D_RELEASE_RESET();
    __HAL_RCC_GPU2D_CLK_ENABLE();

    NVIC_SetPriority(GPU2D_IRQn, 6);
    NVIC_EnableIRQ(GPU2D_IRQn);

    NVIC_SetPriority(GPU2D_ER_IRQn, 5);
    NVIC_EnableIRQ(GPU2D_ER_IRQn);
  }
}

void HAL_GPU2D_MspDeInit(GPU2D_HandleTypeDef* hgpu2d)
{
  if(hgpu2d->Instance==(GPU2D_TypeDef)GPU2D)
  {
    __HAL_RCC_GPU2D_CLK_DISABLE();
  }
}

void HAL_GFXMMU_MspInit(GFXMMU_HandleTypeDef *hgfxmmu)
{
  /* GFXMMU clock enable */
  __HAL_RCC_GFXMMU_CLK_ENABLE();

  /* Enable GFXMMU interrupt */
  NVIC_SetPriority(GFXMMU_IRQn, 5);
  NVIC_EnableIRQ(GFXMMU_IRQn);
}

void HAL_GFXMMU_MspDeInit(GFXMMU_HandleTypeDef *hgfxmmu)
{
  /* GFXMMU clock enable */
  __HAL_RCC_GFXMMU_CLK_DISABLE();

  /* Enable GFXMMU interrupt */
  NVIC_DisableIRQ(GFXMMU_IRQn);
}
