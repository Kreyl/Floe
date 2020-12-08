/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32G0xx+/hal_lld.c
 * @brief   STM32G0xx+ HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

void hal_lld_init(void) {

  /* Reset of all peripherals.*/
  rccResetAHB(~0);
  rccResetAPBR1(~RCC_APBRSTR1_PWRRST);
  rccResetAPBR2(~0);

  /* PWR clock enabled.*/
  rccEnablePWRInterface(true);

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* IRQ subsystem initialization.*/
//  irqInit();
  nvicEnableVector(STM32_TIM15_NUMBER, STM32_ST_IRQ_PRIORITY);

}

OSAL_IRQ_HANDLER(STM32_TIM15_HANDLER) {
  OSAL_IRQ_PROLOGUE();
#if STM32_ST_USE_TIM15
  st_lld_serve_interrupt();
#endif
  OSAL_IRQ_EPILOGUE();
}
