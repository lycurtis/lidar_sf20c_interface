/**
 * @file    interrupts.c
 * @brief   Cortex-M3 fault handler stubs
 *
 * NOTE: SysTick_Handler lives in bsp/bsp.c (BSP owns the tick counter).
 *       Do NOT define it here.
 */

#include "stm32f1xx.h"

void NMI_Handler(void)        { while (1); }
void HardFault_Handler(void)  { while (1); }
void MemManage_Handler(void)  { while (1); }
void BusFault_Handler(void)   { while (1); }
void UsageFault_Handler(void) { while (1); }

void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

/* SysTick_Handler — defined in bsp/bsp.c, do NOT add here */