// app/main.c

#include <stdio.h>
#include <string.h>
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


    static sf_parse_ctx_t lidar_ctx;
    sf_parser_init(&lidar_ctx);

    uint8_t tx_buf[16];
    uint16_t len;

    len = sf_build_read_request(0, tx_buf, sizeof(tx_buf));
    usart_write(BSP_USART_LIDAR, tx_buf, len);
    BSP_Delay_ms(100);
    usart_write(BSP_USART_LIDAR, tx_buf, len);

    bool handshake = false;
    while (!handshake) {
        if (usart_rx_ready(BSP_USART_LIDAR)) {
            uint8_t b = usart_read_byte(BSP_USART_LIDAR);
            if (sf_parser_feed(&lidar_ctx, b) && lidar_ctx.last_packet.cmd_id == 0) {
                printf("Connected: %.*s\r\n",
                       lidar_ctx.last_packet.payload_len,
                       lidar_ctx.last_packet.payload);
                handshake = true;
            }
        }
    }

    uint8_t stream_val[] = {5, 0, 0, 0};
    len = sf_build_write_request(30, stream_val, 4, tx_buf, sizeof(tx_buf));
    usart_write(BSP_USART_LIDAR, tx_buf, len);

    for (;;) {
        while (usart_rx_ready(BSP_USART_LIDAR)) {
            uint8_t b = usart_read_byte(BSP_USART_LIDAR);
            if (sf_parser_feed(&lidar_ctx, b) && lidar_ctx.last_packet.cmd_id == 44) {
                int16_t dist_cm;
                memcpy(&dist_cm, lidar_ctx.last_packet.payload, sizeof(dist_cm));
                printf("Distance: %d cm\r\n", dist_cm);
            }
        }
    }
}