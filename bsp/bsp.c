// bsp/bsp.c

#include "bsp.h"

static volatile uint32_t s_tick_count = 0U;

/**
 * @brief  Configure the system clock to 72 MHz via HSE + PLL.
 *
 * Clock tree:
 *   HSE (8 MHz) → PLL ×9 → SYSCLK 72 MHz
 *   AHB  prescaler = 1  → HCLK  = 72 MHz
 *   APB1 prescaler = 2  → PCLK1 = 36 MHz  (max for APB1)
 *   APB2 prescaler = 1  → PCLK2 = 72 MHz
 *   Flash latency  = 2 WS (required for 48 < SYSCLK ≤ 72 MHz)
 */
static void BSP_ClockConfig(void) {
    /* --- Enable HSE and wait for it to stabilise -------------------- */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        /* TODO: add timeout / error handling */
    }

    /* --- Flash latency: 2 wait-states for 72 MHz -------------------- */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* --- AHB, APB1, APB2 prescalers --------------------------------- */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; /* HCLK  = SYSCLK / 1      */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; /* PCLK1 = HCLK / 2        */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; /* PCLK2 = HCLK / 1        */

    /* --- PLL: HSE × 9 = 72 MHz -------------------------------------- */
    RCC->CFGR |= RCC_CFGR_PLLSRC; /* PLL source = HSE         */
    RCC->CFGR |= RCC_CFGR_PLLMULL9; /* PLL multiplier = 9       */

    /* --- Enable PLL and wait ---------------------------------------- */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        /* TODO: add timeout / error handling */
    }

    /* --- Switch SYSCLK to PLL --------------------------------------- */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        /* TODO: add timeout / error handling */
    }
}

/**
 * @brief  Configure the mode of a single pin in CRL (pins 0-7) or
 *         CRH (pins 8-15).
 *
 * @param  port     GPIO port (e.g. GPIOA)
 * @param  pin      Pin number 0-15
 * @param  mode_cnf Combined MODE[1:0] | CNF[1:0]<<2  (4-bit value)
 *
 * Common 4-bit values (reference table):
 *   0x0  Input  Analog
 *   0x4  Input  Floating
 *   0x8  Input  Pull-up / Pull-down (set ODR for up or down)
 *   0x1  Output 10 MHz Push-Pull
 *   0x2  Output  2 MHz Push-Pull
 *   0x3  Output 50 MHz Push-Pull
 *   0x9  AF     10 MHz Push-Pull
 *   0xB  AF     50 MHz Push-Pull
 *   0xF  AF     50 MHz Open-Drain
 */
static void BSP_GPIO_SetMode(GPIO_TypeDef* port, uint32_t pin, uint32_t mode_cnf) {
    volatile uint32_t* cr = (pin < 8U) ? &port->CRL : &port->CRH;
    uint32_t pos = BSP_PIN_CR_POS(pin);

    uint32_t tmp = *cr;
    tmp &= ~(0xFU << pos);
    tmp |= (mode_cnf & 0xFU) << pos;
    *cr = tmp;
}

/**
 * @brief  Set all GPIO pin directions/modes according to bsp_pinmap.h.
 *
 * This is the **only place** where pin hardware configuration happens.
 * If you add a new pin to bsp_pinmap.h, add its mode setup here.
 */
static void BSP_GPIO_Init(void) {
    /* --- Enable GPIO port clocks, AFIO, and USART1 (APB2) ---------- */
    RCC->APB2ENR |= BSP_APB2_PERIPH_EN;

    /* --- Enable USART2 (APB1, debug TX) ---------------------------- */
    RCC->APB1ENR |= BSP_APB1_PERIPH_EN;

    /* Small delay after enabling clocks (silicon errata workaround) */
    __NOP();
    __NOP();

    /* -------------------------------------------------------------- */
    /*  USART1 — LiDAR (PB6 TX, PB7 RX)  REMAPPED from PA9/PA10       */
    /* -------------------------------------------------------------- */

    /* Enable USART1 remap: TX→PB6, RX→PB7 */
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

    /* USART1 TX — PB6   AF 50 MHz Push-Pull (0xB) */
    BSP_GPIO_SetMode(BSP_USART_LIDAR_TX_PORT, BSP_USART_LIDAR_TX_PIN, 0xB);

    /* USART1 RX — PB7   Input Floating (0x4) */
    BSP_GPIO_SetMode(BSP_USART_LIDAR_RX_PORT, BSP_USART_LIDAR_RX_PIN, 0x4);

    /* -------------------------------------------------------------- */
    /*  USART2 — Debug TX to computer (PA2 TX only; no RX used)        */
    /* -------------------------------------------------------------- */
    BSP_GPIO_SetMode(BSP_USART_DEBUG_TX_PORT, BSP_USART_DEBUG_TX_PIN, 0xB);
}

static void BSP_SysTick_Init(void) {
    // Reload value for 1 ms tick: HCLK / 1000 - 1
    SysTick->LOAD = (BSP_HCLK_HZ / BSP_SYSTICK_HZ) - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk // AHB clock      
                    | SysTick_CTRL_TICKINT_Msk // Enable interrupt
                    | SysTick_CTRL_ENABLE_Msk; // Start counter  
}

void BSP_Init(void) {
    BSP_ClockConfig();
    BSP_GPIO_Init();
    BSP_SysTick_Init();
}

void BSP_Delay_ms(uint32_t ms) {
    uint32_t start = s_tick_count;
    while ((s_tick_count - start) < ms) {
        __WFI(); // Sleep until next interrupt - saves power
    }
}

uint32_t BSP_GetTick(void) { // Get the current tick count
    return s_tick_count;
}


void SysTick_Handler(void) { // SysTick ISR - called every 1 ms
    s_tick_count++;
}