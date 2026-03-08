// bsp/bsp_config.h

#pragma once

#define BSP_HSE_HZ 8000000U // 8 MHz

/** Target SYSCLK after PLL configuration (Hz).                       */
#define BSP_SYSCLK_HZ 72000000U // 72 MHz

/** AHB bus clock (HCLK) — equal to SYSCLK with prescaler = 1.       */
#define BSP_HCLK_HZ BSP_SYSCLK_HZ

/** APB1 bus clock — max 36 MHz, so prescaler = 2.                    */
#define BSP_PCLK1_HZ (BSP_SYSCLK_HZ / 2U) /* 36 MHz */

/** APB2 bus clock — can run at full SYSCLK speed.                    */
#define BSP_PCLK2_HZ BSP_SYSCLK_HZ /* 72 MHz */

/** SysTick interval (Hz) — typically 1 ms tick.                      */
#define BSP_SYSTICK_HZ 1000U

/** APB2 peripherals to enable at startup.
 *  GPIOA, GPIOB, GPIOC for pins; AFIO for alternate-function remap;
 *  USART1 lives on APB2.                                             */
#define BSP_APB2_PERIPH_EN                                                                         \
    (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN             \
     | RCC_APB2ENR_USART1EN)

#define BSP_APB1_PERIPH_EN (RCC_APB1ENR_USART2EN)

#define BSP_IRQ_PRIO_SYSTICK 0U // SysTick
#define BSP_IRQ_PRIO_USART1 1U // LiDAR (USART2 is TX-only, no IRQ)

#define BSP_USART_LIDAR_BAUD 921600U // LiDAR (preconfigured)  
#define BSP_USART_DEBUG_BAUD 115200U // Debug TX           
