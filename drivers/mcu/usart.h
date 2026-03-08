// drivers/mcu/usart.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx.h"

typedef struct {
    USART_TypeDef *instance; // USART1, USART2
    uint32_t       baud_rate;
    uint8_t        stop_bits; // 1 or 2  (0 defaults to 1)
} usart_config_t;

void usart_init(const usart_config_t *config);
void usart_write(USART_TypeDef *usart, const uint8_t *data, uint32_t len);
void usart_write_byte(USART_TypeDef *usart, uint8_t byte);

/**
 * Check if a received byte is available in the RX ring buffer.
 * @return true if at least one byte is waiting.
 */
bool usart_rx_ready(USART_TypeDef *usart);

/**
 * Read one byte from the RX ring buffer.
 * Blocks (spins) if the buffer is empty.
 */
uint8_t usart_read_byte(USART_TypeDef *usart);
