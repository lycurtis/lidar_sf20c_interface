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

    static sf_parse_ctx_t lidar_ctx;
    sf_parser_init(&lidar_ctx);

    uint8_t tx_buf[16];
    uint16_t len = sf_build_read_request(0, tx_buf, sizeof(tx_buf));

    printf("TX packet (%u bytes): ", len);
    for (uint16_t i = 0; i < len; i++) {
        printf("%02X ", tx_buf[i]);
    }
    printf("\r\n");

    BSP_Delay_ms(1000);
    usart_write(BSP_USART_LIDAR, tx_buf, len);
    BSP_Delay_ms(100);
    usart_write(BSP_USART_LIDAR, tx_buf, len);

    uint32_t rx_count = 0;

    for (;;) {
        while (usart_rx_ready(BSP_USART_LIDAR)) {
            uint8_t b = usart_read_byte(BSP_USART_LIDAR);
            printf("%02X ", b);
            rx_count++;
            if (sf_parser_feed(&lidar_ctx, b)) {
                if (lidar_ctx.last_packet.cmd_id == 0) {
                    printf("\r\nProduct: %.*s\r\n",
                           lidar_ctx.last_packet.payload_len,
                           lidar_ctx.last_packet.payload);
                }
            }
        }

        printf("\r\n[RX=%lu] Sending request...\r\n", rx_count);
        usart_write(BSP_USART_LIDAR, tx_buf, len);
        BSP_Delay_ms(500);
    }
}