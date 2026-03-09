# SF20/C Binary Protocol Driver — Implementation Notes

Low-level details for `lidar.c` / `lidar.h`. For the full SF20/C register map,
command descriptions, and product specifications see the project root `README.md`.

## LiDAR preconfiguration (via LightWare Studio)

These settings are flashed to the sensor's onboard storage before use and persist
across power cycles. They must match the MCU-side configuration in `bsp_config.h`.

| Parameter | Value | Why |
|-----------|-------|-----|
| Baud rate (ID 90) | 7 → 921600 bps | Maximum throughput for 5 kHz streaming |
| Update rate preselect (ID 93) | 12 → 5000 Hz | Maximum measurement rate |
| Communication mode (ID 28) | 4 → Serial UART, no preamble | Eliminates legacy startup banner; binary packets accepted immediately |
| Startup mode | No preamble | Sensor does not emit MMI text on power-up |

If these are ever changed back (e.g. via LightWare Studio), the MCU baud rate in
`bsp_config.h` (`BSP_USART_LIDAR_BAUD`) must be updated to match.

## Packet format (binary protocol)

Every packet on the wire (both request and response) has the same structure:

```
Byte:  [0]    [1]        [2]         [3]      [3+N]    [3+N+1]
       start  flags_low  flags_high  payload  CRC_low  CRC_high
       0xAA   ────────────────────── ──────── ─────────────────
              16-bit flags            N bytes  16-bit CRC
```

### Flags (16-bit, little-endian)

| Bits | Field |
|------|-------|
| 0 | Write bit: 0 = read, 1 = write |
| 1–5 | Reserved (0) |
| 15–6 | Payload length (0–1023). This counts the ID byte + data bytes |

To encode: `flags = (payload_len << 6) | (write ? 1 : 0)`
To decode payload length from flags: `payload_len = flags >> 6`

### Payload

The first byte of the payload is always the **command ID**. The remaining bytes
(if any) are the command data. For a read request the payload is just the ID
(1 byte, length = 1). For a write request the payload is the ID followed by the
data bytes.

### CRC-16-CCITT (0x1021)

The checksum covers **every byte except the CRC itself**: start byte + flags_low +
flags_high + all payload bytes. CRC is stored little-endian (low byte first).

**Critical implementation note:** The reference algorithm from the product guide
uses sequential shift-and-XOR steps. When refactoring into fewer lines, do NOT
combine `code = code << N` with `^ code` on the same line — this changes the
intermediate value of `code` and produces a wrong checksum. Keep each step
separate:

```c
crc = crc << 8;
crc ^= code;
code = code << 5;   // NOT (code << 5) ^ code
crc ^= code;
code = code << 7;   // NOT (code << 7) ^ code
crc ^= code;
```

## Receiver state machine (`sf_parser_feed`)

The parser is fed one byte at a time from the UART RX ring buffer. It returns
`true` when a complete, CRC-validated packet has been assembled in
`ctx.last_packet`.

```
  ┌──────────┐  byte == 0xAA   ┌────────────┐
  │ SYNC     │────────────────→│ FLAGS_LOW  │
  └──────────┘                  └────────────┘
       ↑  any failure                │
       │  (bad length,               ↓
       │   bad CRC)            ┌────────────┐
       │                       │ FLAGS_HIGH │── extract payload_len
       │                       └────────────┘   if invalid → SYNC
       │                             │
       │                             ↓
       │                       ┌────────────┐
       │                       │ PAYLOAD    │── collect payload_len bytes
       │                       └────────────┘
       │                             │
       │                             ↓
       │                       ┌────────────┐
       │                       │ CRC_LOW    │
       │                       └────────────┘
       │                             │
       │                             ↓
       │                       ┌────────────┐
       └───────────────────────│ CRC_HIGH   │── verify CRC
                               └────────────┘   if valid → populate
                                                 last_packet, return true
```

On any failure the machine resets to `SYNC` and scans for the next `0xAA`. This
makes the parser self-recovering on corrupted or partial data.

### CRC verification in `CRC_HIGH`

Rather than buffering the entire raw packet, the CRC is computed in two calls:
1. `sf_crc16(hdr, 3)` over the 3-byte header `[0xAA, flags_low, flags_high]`
2. `sf_crc16_continue(crc, payload, payload_len)` continues the running CRC over
   the payload bytes

This avoids needing a separate contiguous buffer for header + payload.

## Parsed packet (`sf_packet_t`)

After a successful parse, `ctx.last_packet` contains:

| Field | Source | Description |
|-------|--------|-------------|
| `cmd_id` | `payload[0]` | Command register ID (0–255) |
| `write` | `flags & 0x01` | Whether this was a write response |
| `payload_len` | `raw_payload_len - 1` | Number of **data** bytes (excludes the ID byte) |
| `payload[]` | `raw_payload[1..]` | Data bytes only (ID byte stripped) |

## Transmitter (`sf_build_packet`)

`sf_build_read_request` and `sf_build_write_request` are thin wrappers around
`sf_build_packet`, which fills a caller-provided buffer:

```
buf[0]           = 0xAA
buf[1..2]        = flags (little-endian)
buf[3]           = command ID
buf[4..3+dataN]  = data bytes (write only)
buf[N-2..N-1]    = CRC-16 (little-endian)
```

Returns the total packet length, or 0 if the buffer is too small.

A read request is always **6 bytes**: 1 start + 2 flags + 1 ID + 2 CRC.

## Handshake sequence

Per the product guide, the first command after power-up is consumed for interface
detection (serial vs I2C) and does **not** produce a response. The recommended
startup is:

1. Send Read Product Name (ID 0)  — consumed silently
2. Wait ~100 ms
3. Send Read Product Name (ID 0)  — response expected
4. Parse until `cmd_id == 0` with payload `"SF20\0..."` → sensor confirmed alive

With Communication Mode set to 4 (no preamble), there is no startup banner to
drain and no initial delay is needed before sending the first command.

## Streaming mode

Once the handshake succeeds, streaming is enabled by writing to the Stream
register (ID 30):

| Write value (uint32_t LE) | Effect |
|---------------------------|--------|
| `{5, 0, 0, 0}` | Stream distance data in cm (cmd ID 44 packets) |
| `{6, 0, 0, 0}` | Stream distance data in mm (cmd ID 45 packets) |
| `{0, 0, 0, 0}` | Disable streaming |

Once enabled, the sensor pushes distance packets at the configured update rate
without the host sending further requests. The main loop only needs to feed
incoming bytes into `sf_parser_feed` and check for `cmd_id == 44` (or 45).

### Distance data layout (cmd 44, cm mode)

The payload size depends on which outputs are enabled in the Distance Output
register (ID 27). Each enabled bit adds one `int16_t` to the payload, packed in
bit order. With only bit 0 set (first return raw), the payload is a single
`int16_t` representing the distance in centimeters.

## Memory considerations

`sf_parse_ctx_t` is approximately 2 KB (two 1024-byte buffers for raw payload and
parsed packet data). On the STM32F103RB (20 KB SRAM), this should be declared
`static` to place it in `.bss` rather than on the stack, which is typically 1–4 KB.

## UART configuration

| Parameter | USART1 (LiDAR) | USART2 (Debug) |
|-----------|----------------|----------------|
| Baud rate | 921600 | 115200 |
| Data bits | 8 | 8 |
| Parity | None | None |
| Stop bits | 1 | 1 |
| Direction | TX + RX | TX only |
| Pins | PB6 (TX), PB7 (RX) — AFIO remap | PA2 (TX) |
| RX method | Interrupt-driven ring buffer (1024 bytes) | N/A |
| IRQ priority | 1 (below SysTick at 0) | N/A |

USART1 uses the AFIO remap (`AFIO->MAPR |= AFIO_MAPR_USART1_REMAP`) to move
TX/RX from the default PA9/PA10 to PB6/PB7.

## Lessons learned

- **CRC refactoring trap:** Combining `code = code << N; crc ^= code;` into
  `code = (code << N) ^ code; crc ^= code;` silently produces wrong checksums.
  The sensor rejects packets with bad CRC without any error response — it falls
  back to legacy text mode and replies with `$: ` prompts instead.

- **Blocking reads:** `usart_read_byte` spins while the RX buffer is empty. In
  the main loop always guard with `usart_rx_ready()` first, otherwise the loop
  hangs and no other work (debug prints, state machines) can execute.

- **Stack overflow:** `sf_parse_ctx_t` on the stack can overflow the default
  1–2 KB stack on Cortex-M3. Use `static` storage.

- **Startup banner:** In Communication Mode 3 (legacy header), the sensor emits
  ASCII text (`p:SF20,...` and `$: ` prompts) before accepting binary commands.
  Mode 4 (no preamble) avoids this entirely.

- **Baud rate mismatch:** The sensor's baud rate persists across power cycles.
  If the MCU and sensor baud rates don't match, received bytes appear as
  repeating garbage (e.g. `F1 F1 F1...`). Always verify both sides agree.

- **Legacy text protocol:** If the sensor is in legacy mode, it uses a different
  command format: `?<cmd>\r` for reads, `#<cmd>,<value>\r` for writes. Command
  names are short letter codes (e.g. `?ldf` for laser distance first, `?cb` for
  baud rate). This is the older MMI protocol and is not used by this driver.
