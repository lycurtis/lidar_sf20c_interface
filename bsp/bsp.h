// bsp/bsp.h
#pragma once

#include "bsp_config.h"
#include "bsp_pinmap.h"

void BSP_Init(void);

void BSP_Delay_ms(uint32_t ms);
uint32_t BSP_GetTick(void); // Get the current tick count
