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
 * @file    STM32G0xx/hal_lld.h
 * @brief   STM32G0xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSEDRV.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32G070xx.
 *          - STM32G071xx, STM32G081xx.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "stm32_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#if defined(STM32G070xx) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "STM32G0 Entry-level Value Line"

#elif defined(STM32G071xx)
#define PLATFORM_NAME           "STM32G0 Entry-level"

#elif defined(STM32G081xx)
#define PLATFORM_NAME           "STM32G0 Entry-level with Crypto"

#else
#error "STM32G0 device not specified"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(STM32G0XX) || defined(__DOXYGEN__)
#define STM32G0XX
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSI16CLK          16000000U   /**< 16MHz internal clock.      */
#define STM32_LSICLK            32000U      /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PWR_CR1 register bits definitions
 * @{
 */
#define STM32_VOS_MASK          (3U << 9U)  /**< Core voltage mask.         */
#define STM32_VOS_RANGE1        (1U << 9U)  /**< Core voltage 1.2 Volts.    */
#define STM32_VOS_RANGE2        (2U << 9U)  /**< Core voltage 1.0 Volts.    */
/** @} */

/**
 * @name    PWR_CR2 register bits definitions
 * @{
 */
#define STM32_PVDE_DISABLED     (0U << 1U)      /**< PVD enable bit off.    */
#define STM32_PVDE_ENABLED      (1U << 1U)      /**< PVD enable bit on.     */

#define STM32_PVDFT_MASK        (7U << 1U)      /**< PVDFT bits mask.       */
#define STM32_PVDFT(n)          ((n) << 1U)     /**< PVDFT level.           */
#define STM32_PVDFT_LEV0        STM32_PVDFT(0U) /**< PVDFT level 0.         */
#define STM32_PVDFT_LEV1        STM32_PVDFT(1U) /**< PVDFT level 1.         */
#define STM32_PVDFT_LEV2        STM32_PVDFT(2U) /**< PVDFT level 2.         */
#define STM32_PVDFT_LEV3        STM32_PVDFT(3U) /**< PVDFT level 3.         */
#define STM32_PVDFT_LEV4        STM32_PVDFT(4U) /**< PVDFT level 4.         */
#define STM32_PVDFT_LEV5        STM32_PVDFT(5U) /**< PVDFT level 5.         */
#define STM32_PVDFT_LEV6        STM32_PVDFT(6U) /**< PVDFT level 6.         */
#define STM32_PVDFT_LEV7        STM32_PVDFT(7U) /**< PVDFT level 7.         */

#define STM32_PVDRT_MASK        (7U << 4U)      /**< PVDRT bits mask.       */
#define STM32_PVDRT(n)          ((n) << 4U)     /**< PVDRT level.           */
#define STM32_PVDRT_LEV0        STM32_PVDRT(0U) /**< PVDRT level 0.         */
#define STM32_PVDRT_LEV1        STM32_PVDRT(1U) /**< PVDRT level 1.         */
#define STM32_PVDRT_LEV2        STM32_PVDRT(2U) /**< PVDRT level 2.         */
#define STM32_PVDRT_LEV3        STM32_PVDRT(3U) /**< PVDRT level 3.         */
#define STM32_PVDRT_LEV4        STM32_PVDRT(4U) /**< PVDRT level 4.         */
#define STM32_PVDRT_LEV5        STM32_PVDRT(5U) /**< PVDRT level 5.         */
#define STM32_PVDRT_LEV6        STM32_PVDRT(6U) /**< PVDRT level 6.         */
#define STM32_PVDRT_LEV7        STM32_PVDRT(7U) /**< PVDRT level 7.         */
/** @} */

/**
 * @name    RCC_CR register bits definitions
 * @{
 */
#define STM32_HSIDIV_MASK       (7U << 11U)     /**< HSIDIV field mask.     */
#define STM32_HSIDIV_FIELD(n)   ((n) << 11U)    /**< HSIDIV field value.    */
#define STM32_HSIDIV_1          STM32_HSIDIV_FIELD(0U)
#define STM32_HSIDIV_2          STM32_HSIDIV_FIELD(1U)
#define STM32_HSIDIV_4          STM32_HSIDIV_FIELD(2U)
#define STM32_HSIDIV_8          STM32_HSIDIV_FIELD(3U)
#define STM32_HSIDIV_16         STM32_HSIDIV_FIELD(4U)
#define STM32_HSIDIV_32         STM32_HSIDIV_FIELD(5U)
#define STM32_HSIDIV_64         STM32_HSIDIV_FIELD(6U)
#define STM32_HSIDIV_128        STM32_HSIDIV_FIELD(7U)
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (7U << 0U)  /**< SW field mask.             */
#define STM32_SW_HSISYS         (0U << 0U)  /**< SYSCLK source is HSISYS.   */
#define STM32_SW_HSE            (1U << 0U)  /**< SYSCLK source is HSE.      */
#define STM32_SW_PLLRCLK        (2U << 0U)  /**< SYSCLK source is PLL.      */
#define STM32_SW_LSI            (3U << 0U)  /**< SYSCLK source is LSI.      */
#define STM32_SW_LSE            (4U << 0U)  /**< SYSCLK source is LSE.      */

#define STM32_HPRE_MASK         (15U << 8U) /**< HPRE field mask.           */
#define STM32_HPRE_FIELD(n)     ((n) << 8U) /**< HPRE field value.          */
#define STM32_HPRE_DIV1         STM32_HPRE_FIELD(0U)
#define STM32_HPRE_DIV2         STM32_HPRE_FIELD(8U)
#define STM32_HPRE_DIV4         STM32_HPRE_FIELD(9U)
#define STM32_HPRE_DIV8         STM32_HPRE_FIELD(10U)
#define STM32_HPRE_DIV16        STM32_HPRE_FIELD(11U)
#define STM32_HPRE_DIV64        STM32_HPRE_FIELD(12U)
#define STM32_HPRE_DIV128       STM32_HPRE_FIELD(13U)
#define STM32_HPRE_DIV256       STM32_HPRE_FIELD(14U)
#define STM32_HPRE_DIV512       STM32_HPRE_FIELD(15U)

#define STM32_PPRE_MASK         (7U << 12U)  /**< PPRE field mask.          */
#define STM32_PPRE_FIELD(n)     ((n) << 12U) /**< PPRE field value.         */
#define STM32_PPRE_DIV1         STM32_PPRE_FIELD(0U)
#define STM32_PPRE_DIV2         STM32_PPRE_FIELD(4U)
#define STM32_PPRE_DIV4         STM32_PPRE_FIELD(5U)
#define STM32_PPRE_DIV8         STM32_PPRE_FIELD(6U)
#define STM32_PPRE_DIV16        STM32_PPRE_FIELD(7U)

#define STM32_MCOSEL_MASK       (7U << 24U) /**< MCOSEL field mask.         */
#define STM32_MCOSEL_NOCLOCK    (0U << 24U) /**< No clock on MCO pin.       */
#define STM32_MCOSEL_SYSCLK     (1U << 24U) /**< SYSCLK on MCO pin.         */
#define STM32_MCOSEL_HSI16      (3U << 24U) /**< HSI16 clock on MCO pin.    */
#define STM32_MCOSEL_HSE        (4U << 24U) /**< HSE clock on MCO pin.      */
#define STM32_MCOSEL_PLLRCLK    (5U << 24U) /**< PLLR clock on MCO pin.     */
#define STM32_MCOSEL_LSI        (6U << 24U) /**< LSI clock on MCO pin.      */
#define STM32_MCOSEL_LSE        (7U << 24U) /**< LSE clock on MCO pin.      */

#define STM32_MCOPRE_MASK       (7U << 28U) /**< MCOPRE field mask.         */
#define STM32_MCOPRE_FIELD(n)   ((n) << 28U)/**< MCOPRE field value         */
#define STM32_MCOPRE_DIV1       STM32_MCOPRE_FIELD(0U)
#define STM32_MCOPRE_DIV2       STM32_MCOPRE_FIELD(1U)
#define STM32_MCOPRE_DIV4       STM32_MCOPRE_FIELD(2U)
#define STM32_MCOPRE_DIV8       STM32_MCOPRE_FIELD(3U)
#define STM32_MCOPRE_DIV16      STM32_MCOPRE_FIELD(4U)
#define STM32_MCOPRE_DIV32      STM32_MCOPRE_FIELD(5U)
#define STM32_MCOPRE_DIV64      STM32_MCOPRE_FIELD(6U)
#define STM32_MCOPRE_DIV128     STM32_MCOPRE_FIELD(7U)
/** @} */

/**
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLSRC_MASK       (3 << 0)    /**< PLL clock source mask.     */
#define STM32_PLLSRC_NOCLOCK    (0 << 0)    /**< PLL clock source disabled. */
#define STM32_PLLSRC_HSI16      (2 << 0)    /**< PLL clock source is HSI16. */
#define STM32_PLLSRC_HSE        (3 << 0)    /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CCIPR register bits definitions
 * @{
 */
#define STM32_USART1SEL_MASK    (3U << 0U)  /**< USART1SEL mask.            */
#define STM32_USART1SEL_PCLK    (0U << 0U)  /**< USART1 source is PCLK.     */
#define STM32_USART1SEL_SYSCLK  (1U << 0U)  /**< USART1 source is SYSCLK.   */
#define STM32_USART1SEL_HSI16   (2U << 0U)  /**< USART1 source is HSI16.    */
#define STM32_USART1SEL_LSE     (3U << 0U)  /**< USART1 source is LSE.      */

#define STM32_USART2SEL_MASK    (3U << 2U)  /**< USART2 mask.               */
#define STM32_USART2SEL_PCLK    (0U << 2U)  /**< USART2 source is PCLK.     */
#define STM32_USART2SEL_SYSCLK  (1U << 2U)  /**< USART2 source is SYSCLK.   */
#define STM32_USART2SEL_HSI16   (2U << 2U)  /**< USART2 source is HSI16.    */
#define STM32_USART2SEL_LSE     (3U << 2U)  /**< USART2 source is LSE.      */

#define STM32_CECSEL_MASK       (1U << 6U)  /**< CEC mask.                  */
#define STM32_CECSEL_HSI16DIV   (0U << 6U)  /**< CEC source is HSI16/448.   */
#define STM32_CECSEL_LSE        (1U << 6U)  /**< CEC source is LSE.         */

#define STM32_LPUART1SEL_MASK   (3U << 10U) /**< LPUART1 mask.              */
#define STM32_LPUART1SEL_PCLK   (0U << 10U) /**< LPUART1 source is PCLK.    */
#define STM32_LPUART1SEL_SYSCLK (1U << 10U) /**< LPUART1 source is SYSCLK.  */
#define STM32_LPUART1SEL_HSI16  (2U << 10U) /**< LPUART1 source is HSI16.   */
#define STM32_LPUART1SEL_LSE    (3U << 10U) /**< LPUART1 source is LSE.     */

#define STM32_I2C1SEL_MASK      (3U << 12U) /**< I2C1SEL mask.              */
#define STM32_I2C1SEL_PCLK      (0U << 12U) /**< I2C1 source is PCLK.       */
#define STM32_I2C1SEL_SYSCLK    (1U << 12U) /**< I2C1 source is SYSCLK.     */
#define STM32_I2C1SEL_HSI16     (2U << 12U) /**< I2C1 source is HSI16.      */

#define STM32_I2S1SEL_MASK      (3U << 14U) /**< I2S1SEL mask.              */
#define STM32_I2S1SEL_SYSCLK    (0U << 14U) /**< I2S1 source is SYSCLK.     */
#define STM32_I2S1SEL_PLLPCLK   (1U << 14U) /**< I2S1 source is PLLPCLK.    */
#define STM32_I2S1SEL_HSI16     (2U << 14U) /**< I2S1 source is HSI16.      */
#define STM32_I2S1SEL_CKIN      (3U << 14U) /**< I2S1 source is CKIN.       */

#define STM32_LPTIM1SEL_MASK    (3U << 18U) /**< LPTIM1SEL mask.            */
#define STM32_LPTIM1SEL_PCLK    (0U << 18U) /**< LPTIM1 source is PCLK.     */
#define STM32_LPTIM1SEL_LSI     (1U << 18U) /**< LPTIM1 source is LSI.      */
#define STM32_LPTIM1SEL_HSI16   (2U << 18U) /**< LPTIM1 source is HSI16.    */
#define STM32_LPTIM1SEL_LSE     (3U << 18U) /**< LPTIM1 source is LSE.      */

#define STM32_LPTIM2SEL_MASK    (3U << 20U) /**< LPTIM2SEL mask.            */
#define STM32_LPTIM2SEL_PCLK    (0U << 20U) /**< LPTIM2 source is PCLK.     */
#define STM32_LPTIM2SEL_LSI     (1U << 20U) /**< LPTIM2 source is LSI.      */
#define STM32_LPTIM2SEL_HSI16   (2U << 20U) /**< LPTIM2 source is HSI16.    */
#define STM32_LPTIM2SEL_LSE     (3U << 20U) /**< LPTIM2 source is LSE.      */

#define STM32_TIM1SEL_MASK      (1U << 22U) /**< TIM1SEL mask.              */
#define STM32_TIM1SEL_TIMPCLK   (0U << 22U) /**< TIM1SEL source is TIMPCLK. */
#define STM32_TIM1SEL_PLLQCLK   (1U << 22U) /**< TIM1SEL source is PLLQCLK. */

#define STM32_TIM15SEL_MASK     (1U << 24U) /**< TIM15SEL mask.             */
#define STM32_TIM15SEL_TIMPCLK  (0U << 24U) /**< TIM15SEL source is TIMPCLK.*/
#define STM32_TIM15SEL_PLLQCLK  (1U << 24U) /**< TIM15SEL source is PLLQCLK.*/

#define STM32_RNGSEL_MASK       (3U << 26U) /**< RNGSEL mask.               */
#define STM32_RNGSEL_NOCLOCK    (0U << 26U) /**< RNG source is disabled.    */
#define STM32_RNGSEL_HSI16      (1U << 26U) /**< RNG source is HSI16.       */
#define STM32_RNGSEL_SYSCLK     (2U << 26U) /**< RNG source is SYSCLK.      */
#define STM32_RNGSEL_PLLQCLK    (3U << 26U) /**< RNG source is PLLQCLK.     */

#define STM32_RNGDIV_MASK       (3U << 28U) /**< RNGDIV field mask.         */
#define STM32_RNGDIV_FIELD(n)   ((n) << 28U)/**< RNGDIV field value         */
#define STM32_RNGDIV_1          STM32_RNGDIV_FIELD(0U)
#define STM32_RNGDIV_2          STM32_RNGDIV_FIELD(1U)
#define STM32_RNGDIV_4          STM32_RNGDIV_FIELD(2U)
#define STM32_RNGDIV_8          STM32_RNGDIV_FIELD(3U)

#define STM32_ADCSEL_MASK       (3U << 30U) /**< ADCSEL mask.               */
#define STM32_ADCSEL_SYSCLK     (0U << 30U) /**< ADC source is SYSCLK.      */
#define STM32_ADCSEL_PLLPCLK    (1U << 30U) /**< ADC source is PLLPCLK.     */
#define STM32_ADCSEL_HSI16      (2U << 30U) /**< ADC source is HSI16.       */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3U << 8U)  /**< RTC source mask.           */
#define STM32_RTCSEL_NOCLOCK    (0U << 8U)  /**< No RTC source.             */
#define STM32_RTCSEL_LSE        (1U << 8U)  /**< RTC source is LSE.         */
#define STM32_RTCSEL_LSI        (2U << 8U)  /**< RTC source is LSI.         */
#define STM32_RTCSEL_HSEDIV     (3U << 8U)  /**< RTC source is HSE divided. */

#define STM32_LSCOSEL_MASK      (3U << 24U) /**< LSCO pin clock source.     */
#define STM32_LSCOSEL_NOCLOCK   (0U << 24U) /**< No clock on LSCO pin.      */
#define STM32_LSCOSEL_LSI       (1U << 24U) /**< LSI on LSCO pin.           */
#define STM32_LSCOSEL_LSE       (3U << 24U) /**< LSE on LSCO pin.           */
/** @} */


/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "stm32_isr.h"
//#include "stm32_dma.h"
//#include "stm32_exti.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
