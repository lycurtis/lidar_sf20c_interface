// drivers/lidar.h

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Max payload per SF20/C binary protocol (bits 15-6 of flags: 0–1023 bytes). */
#define SF_PAYLOAD_MAX  1024u

#define SF_START_BYTE 0xAA

typedef enum {
    SF_PARSE_SYNC,
    SF_PARSE_FLAGS_LOW,
    SF_PARSE_FLAGS_HIGH,
    SF_PARSE_PAYLOAD,
    SF_PARSE_CRC_LOW,
    SF_PARSE_CRC_HIGH
} sf_parse_state_t;

typedef struct {
    uint8_t cmd_id; // command ID (first byte of the payload)
    bool write; // bit 0 of flags: read (0) or write (1)
    uint16_t payload_len; // payload length (bits 15-6 of flags)
    uint8_t payload[SF_PAYLOAD_MAX - 1]; // data bytes
} sf_packet_t; // result after parsing a packet

typedef struct {
    sf_parse_state_t state;
    uint16_t flags;
    uint16_t payload_len;
    uint16_t payload_idx;
    uint8_t payload[SF_PAYLOAD_MAX];
    uint16_t crc_accum;
    uint16_t rx_crc;
    sf_packet_t last_packet;
} sf_parse_ctx_t;

void sf_parser_init(sf_parse_ctx_t* p);
bool sf_parser_feed(sf_parse_ctx_t* p, uint8_t byte);

uint16_t sf_build_read_request(uint8_t cmd_id, uint8_t* buf, uint16_t buf_size);
uint16_t sf_build_write_request(uint8_t cmd_id, const uint8_t* data, uint16_t data_len, uint8_t* buf, uint16_t buf_size);

