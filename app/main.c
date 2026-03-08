// app/main.c

#include <stdio.h>
#include "bsp.h"
#include "bsp_config.h"
#include "bsp_pinmap.h"
#include "lidar.h"
#include "usart.h"

int main(void) {
    BSP_Init();

    usart_config_t lidar = {
        .instance = BSP_USART_LIDAR,
        .baud_rate = BSP_USART_LIDAR_BAUD,
        .stop_bits = 1,
    };
    usart_init(&lidar);

    usart_config_t debug = {
        .instance = BSP_USART_DEBUG,
        .baud_rate = BSP_USART_DEBUG_BAUD,
        .stop_bits = 1,
    };
    usart_init(&debug);

    printf("LiDAR interface\r\n");

    for (;;) {
    }
}