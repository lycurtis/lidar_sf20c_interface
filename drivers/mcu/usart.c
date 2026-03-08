// drivers/mcu/usart.c


#include "usart.h"

#include "bsp_config.h"

/*
 * USART1 (LiDAR)                       APB2 @ 72MHz
 *   TX = PB6, RX = PB7
 *
 * USART2 (Debug TX)        APB1 @ 36MHz
 *   TX = PA2  (debug log -> ST-Link)
 */

// RX ring buffer (interrupt-driven)

#define RX_BUF_SIZE 1024u
#define RX_BUF_MASK (RX_BUF_SIZE - 1u)

typedef struct {
    uint8_t buf[RX_BUF_SIZE];
    volatile uint16_t head; // ISR writes here  
    volatile uint16_t tail; // main loop reads  
} rx_ring_t;

static rx_ring_t s_rx_lidar; // USART1 (LiDAR) only; USART2 is TX-only for debug

static inline rx_ring_t* rx_ring_for(USART_TypeDef* usart) {
    return (usart == USART1) ? &s_rx_lidar : (rx_ring_t*)0;
}

static inline void rx_isr(USART_TypeDef* usart, rx_ring_t* r) {
    // Read SR then DR - clears RXNE, ORE, and other error flags
    uint32_t sr = usart->SR;
    uint8_t dr = (uint8_t)(usart->DR & 0xFFu);

    if (sr & USART_SR_RXNE) { // RXNE is set when a new byte is received
        uint16_t nxt = (r->head + 1u) & RX_BUF_MASK;
        if (nxt != r->tail) {
            r->buf[r->head] = dr;
            r->head = nxt;
        }
    }
}

// IRQ handlers (override weak defaults in startup .s)

void USART1_IRQHandler(void) {
    rx_isr(USART1, &s_rx_lidar);
}

void usart_init(const usart_config_t* config) {
    if (config == (void*)0 || config->instance == (void*)0) {
        return;
    }

    // USART1 is on APB2; USART2/3 are on APB1
    uint32_t pclk;
    if (config->instance == USART1) {
        pclk = BSP_PCLK2_HZ;
    } else {
        pclk = BSP_PCLK1_HZ;
    }

    // Disable USART while configuring
    config->instance->CR1 &= ~USART_CR1_UE;

    // Baud rate: BRR = fPCLK / baud
    // (pclk + (baud/2)) / baud
    config->instance->BRR = (pclk + (config->baud_rate / 2U)) / config->baud_rate;

    // Stop bits: CR2 STOP[13:12]  00 = 1, 10 = 2
    if (config->stop_bits == 2U) {
        config->instance->CR2 = (config->instance->CR2 & ~USART_CR2_STOP) | USART_CR2_STOP_1;
    } else {
        config->instance->CR2 &= ~USART_CR2_STOP; // 1 stop bit (default)
    }

    // USART1 (LiDAR): full duplex + RX interrupt. USART2 (debug): TX only.
    if (config->instance == USART1) {
        config->instance->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
        NVIC_SetPriority(USART1_IRQn, BSP_IRQ_PRIO_USART1);
        NVIC_EnableIRQ(USART1_IRQn);
    } else if (config->instance == USART2) {
        config->instance->CR1 |= USART_CR1_TE | USART_CR1_UE;
    } else {
        return;
    }
}

void usart_write_byte(USART_TypeDef* usart, uint8_t byte) {
    while (!(usart->SR & USART_SR_TXE))
        ;
    usart->DR = byte;
}

void usart_write(USART_TypeDef* usart, const uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        usart_write_byte(usart, data[i]);
    }
    while (!(usart->SR & USART_SR_TC))
        ;
}

bool usart_rx_ready(USART_TypeDef* usart) {
    rx_ring_t* r = rx_ring_for(usart);
    return r && (r->head != r->tail);
}

uint8_t usart_read_byte(USART_TypeDef* usart) {
    rx_ring_t* r = rx_ring_for(usart);
    if (!r) return 0;
    while (r->head == r->tail) { /* spin until data (USART1 LiDAR only) */ }
    uint8_t b = r->buf[r->tail];
    r->tail = (r->tail + 1u) & RX_BUF_MASK;
    return b;
}