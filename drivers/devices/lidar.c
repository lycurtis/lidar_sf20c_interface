// drivers/lidar.c
/**
 * LightWare SF20/C LiDAR — binary protocol driver
 * Byte-at-a-time state machine that frames, validates, and parses
 * LightWare binary packets arriving on UART.
 */

#include "lidar.h"
#include <string.h> // for memset function

static uint16_t sf_crc16_continue(uint16_t crc, const uint8_t* data, uint16_t size)
{
    for (uint32_t i = 0; i < size; ++i)
    {
        uint16_t code = crc >> 8;
        code ^= data[i];
        code ^= code >> 4;
        crc = crc << 8;
        crc ^= code;
        code = code << 5;
        crc ^= code;
        code = code << 7;
        crc ^= code;
    }
    return crc;
}

static inline uint16_t sf_crc16(const uint8_t* data, uint16_t size)
{
    return sf_crc16_continue(0, data, size);
} 

void sf_parser_init(sf_parse_ctx_t* p){
    memset(p, 0, sizeof(sf_parse_ctx_t));
    p->state = SF_PARSE_SYNC;
}

bool sf_parser_feed(sf_parse_ctx_t* p, uint8_t byte){
    switch(p->state){
        case SF_PARSE_SYNC:
            if(byte == SF_START_BYTE){
                p->state = SF_PARSE_FLAGS_LOW;
            }
            return false;

        case SF_PARSE_FLAGS_LOW:
            p->flags = byte;
            p->state = SF_PARSE_FLAGS_HIGH;
            return false;

        case SF_PARSE_FLAGS_HIGH:
            p->flags |= ((uint16_t)byte << 8);
            p->payload_len = p->flags >> 6;
            if(p->payload_len == 0 || p->payload_len > SF_PAYLOAD_MAX){
                p->state = SF_PARSE_SYNC;
                return false;
            }
            p->payload_idx = 0u;
            p->state = SF_PARSE_PAYLOAD;
            return false;

        case SF_PARSE_PAYLOAD:
            p->payload[p->payload_idx++] = byte;
            if(p->payload_idx == p->payload_len){
                p->state = SF_PARSE_CRC_LOW;
            }
            return false;

        case SF_PARSE_CRC_LOW:
            p->rx_crc = byte;
            p->state = SF_PARSE_CRC_HIGH;
            return false;

        case SF_PARSE_CRC_HIGH: {
            p->rx_crc |= ((uint16_t)byte << 8);

            uint8_t hdr[3] = { 0xAA, p->flags & 0xFF, p->flags >> 8 };
            uint16_t crc = sf_crc16(hdr, 3);
            crc = sf_crc16_continue(crc, p->payload, p->payload_len);

            if(crc != p->rx_crc){
                p->state = SF_PARSE_SYNC;
                return false;
            }

            p->last_packet.cmd_id = p->payload[0];
            p->last_packet.write = p->flags & 0x01;
            p->last_packet.payload_len = p->payload_len - 1u;
            memcpy(p->last_packet.payload, &p->payload[1], p->last_packet.payload_len);

            p->state = SF_PARSE_SYNC;
            return true;
        }
    }

    return false;
}

static uint16_t sf_build_packet(uint8_t cmd_id, bool write, const uint8_t* data, uint16_t data_len, uint8_t* buf, uint16_t buf_size){
    uint16_t payload_len = 1u + data_len; // ID + data
    uint16_t total = 3u + payload_len + 2u; // start + flags(2) + payload + CRC(2)

    if(total > buf_size){
        return 0u;
    }

    buf[0] = SF_START_BYTE;

    uint16_t flags = (payload_len << 6) | (write ? 1u : 0u);
    buf[1] = (uint8_t)(flags & 0xFFu); // flags low
    buf[2] = ((uint8_t)(flags >> 8) & 0xFFu); // flags high

    // Payload
    buf[3] = cmd_id;
    if(data_len > 0u && data != (void*)0){
        memcpy(&buf[4], data, data_len);
    }

    // CRC over everything except the CRC itself
    uint16_t crc = sf_crc16(buf, 3u + payload_len);
    buf[3u + payload_len] = (uint8_t)(crc & 0xFFu); // CRC low
    buf[3u + payload_len + 1u] = (uint8_t)((crc >> 8) & 0xFFu); // CRC high

    return total;
}

uint16_t sf_build_read_request(uint8_t cmd_id, uint8_t* buf, uint16_t buf_size) {
    // Read request: payload = [ID] only, write bit = 0
    return sf_build_packet(cmd_id, false, (void*)0, 0u, buf, buf_size);
}

uint16_t sf_build_write_request(uint8_t cmd_id, const uint8_t* data, uint16_t data_len, uint8_t* buf, uint16_t buf_size) {
    // Write request: payload = [ID] + data, write bit = 1
    return sf_build_packet(cmd_id, true, data, data_len, buf, buf_size);
}