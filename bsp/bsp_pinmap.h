// bsp/bsp_pinmap.h

#pragma once
#include "stm32f103xb.h"

#define BSP_PIN_MASK(pin) (1U << (pin)) // Convert a pin number (0-15) to the corresponding bit mask

#define BSP_PIN_CR_POS(pin) (((pin)&0x07U) * 4U) // Convert a pin number to its CRL/CRH bit position (4 bits per pin)


// USART1 — LiDAR (full duplex)
#define BSP_USART_LIDAR USART1
#define BSP_USART_LIDAR_TX_PORT GPIOB
#define BSP_USART_LIDAR_TX_PIN 6U // PB6  → AF Push-Pull  (remapped)
#define BSP_USART_LIDAR_RX_PORT GPIOB
#define BSP_USART_LIDAR_RX_PIN 7U // PB7  → Input Floating (remapped)

// USART2 — Debug/logging to computer (TX only)
#define BSP_USART_DEBUG USART2
#define BSP_USART_DEBUG_TX_PORT GPIOA
#define BSP_USART_DEBUG_TX_PIN 2U // PA2 → AF Push-Pull

// Convenience: RCC enable masks for GPIO ports in use
// OR these together in BSP_Init() to enable only the clocks needed.
#define BSP_RCC_GPIOA_EN RCC_APB2ENR_IOPAEN
#define BSP_RCC_GPIOB_EN RCC_APB2ENR_IOPBEN
#define BSP_RCC_GPIOC_EN RCC_APB2ENR_IOPCEN
