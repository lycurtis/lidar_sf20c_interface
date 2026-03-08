# LiDAR SF20/C Driver Interface with Nucleo-STM32F103RB

## Protocol
- UART 921600 Baud
- Binary Protocol

## Spec Sheet
- Update Rate: 48 to 5000 readings per second (customizable, I want 5000 Hz)

## Adjustable parameters
### Communication
- Serial UART baud rate: 9600 to 921600 (I am going with 921600)
- Startup mode: Select the communication behavior when powered on
    - Wait for interface: Wait for either an I2C or serial [UART] communication before enabling the appropriate interface. 
    - Serial mode: Immediately start in serial UART mode.
    - I2C mode: Immediately start in I2C mode. Note: If this option is selected, you will no longer be able to use a serial UART adapter to communicate with the sensor.
    - Serial mode with legacy banner: Immediately start in serial UART mode and output legacy diagnostics text on startup.
    - Serial mode with no preamble: Immediately start in serial UART mode, but do not output the initial MMI message on startup.
### Measurement
- Update rate: 48 to 5000 (Select the number of measurements taken per second (Hz).)
- Update rate override: Specify a custom update rate which will take priority over the Update rate parameter. The value will be adjusted to the nearest rate allowed by the sensor. (Whole number, Hz.). 0 to 5000
- Zero distance offset: -10 to 10 (The offset applied to the measured distance value. (In meters, up to three decimal places.) 
- Lost signal threshold: 1 to 250
- Lost signal type: -1.00 to 230.0 (Select the output value to be reported when signal is lost. )
### Filtering
- Median Filter: Used to disregard short unwanted readings
- Median filter size: 3 to 32 (The response time of the median filter. (Whole number, in seconds.)
- Rolling average: Used to average out a specified number of last-distance results. 
- Rolling average size: 2 to 32 (The number of distance results to use for the rolling average filter, (whole number).)
- Smoothing filter enabled: Used to remove noise from the readings. 
- Smoothing filter strength: 0 to 100 (The stronger the smoothing, the slower the response to change, (whole number).)

## Communication interface
Once a sensor is connected to a host controller, the first command from the host controller
will inform it which of the two communication interfaces is being used. Subsequent
commands sent from the host controller to the sensor will request values, change settings, or
alter the sensor’s performance. The sensor will reply to a single command with a single reply,
although the streaming command allows the sensor to continuously update the reply without
the host resending the command. Note that streaming data is only available through the
serial UART interface.

## Serial UART Interface
For serial UART communication, the sensor uses encapsulated packets to send and receive
data. A packet sent to the sensor is a request. A correctly formatted request will always be
replied to with a response. Streaming is available through the serial UART interface. In this
case, the sensor sends request streaming packets without a direct request from the host, and
they do not require a response from the host.
Requests are made using one of the sensor commands. The complete command list is
contained in this product guide. Commands are flagged as either read or write. When a read
request is issued, the response will contain the requested data. When a write request is
issued, the contents of the response will vary depending on the command.

### Default serial UART interface properties:
• Baud rate: 115200 (configurable)
• Data: 8 bit
• Parity: none
• Stop: 1 bit
• Flow control: none

# Commands
The first command sent by the host to the sensor after powerup will be used to detect
whether serial UART or I2C mode is in use. The sensor will not return a response to the first
command. Subsequently, for each command sent by the host controller, a single reply will be
returned by the sensor. 
To initialize the communication with the sensor, send the command to request the Product
name. It is advisable to send the command to query the Product name twice in succession
shortly after powerup. As described above the first request will not return a response,
however the second request will return the product name, indicating that the sensor has
indeed initialized successfully, and a handshake has been successfully established with the
sensor.
The streaming ($) command can be used to command the sensor to continuously update the
reply without waiting for the host controller to resend the command

## Binary Protocol - Command Structure
Both request and response packets are composed of the following bytes:

### Packet composition
- Header: [start] [flags_low] [flags_high]
- Payload: [ID] [data]
- Checksum: [CRC_low] [CRC_high]

### Header Flag byte explanation
- flags_high [bits 15-8]
- flags_low [bits 7-0]. Bit0: Write, Bits[1-5]: Reserved,
Note the payload length (0 to 1023) and is from Bits[15-6]

- The start byte is always 0xAA and indicates the beginning of a packet. 
- The flags bytes form a 16-bit integer representing the packet's payload length and read/write status.
- The payload includes the ID byte, the data bytes, and the write bit. Its length is between 1 and 1023 bytes, inclusive depending on the command type.
- The ID byte indicates which command the request/response relates to. 
- The write bit is 1 to indicate write mode, or 0 to indicate read mode. 
- The CRC bytes form a 16-bit/2-byte checksum value used to validate the integrity of the packet data. The sensor will not accept and process a packet if the CRC is not correctly formed. Every byte in the packet except for the CRC itself is included in the checksum calculation.

## Binary protocol - Checksum algorithm
The checksum algorithm is CRC-16-CCITT 0x1021. Below is a CRC calculation example written in C/C++:
```
uint16_t createCRC(uint8_t* Data, uint16_t Size)
{
 uint16_t crc = 0;
 for (uint32_t i = 0; i < Size; ++i)
 {
 uint16_t code = crc >> 8;
 code ^= Data[i];
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
```

## Binary protocol - Reading bytes
Once a packet is successfully read it can be processed based on its command ID. It is vital to
verify the payload length and checksum before processing.

If either of the following errors are received, “invalid packet length” or “checksum is invalid”,
please roll the incoming stream back to one byte after where the start byte was detected.

Below is the process for reading the raw serial byte stream and identifying packets:

### Packet reception (state machine)

The receiver processes bytes in order and restarts on any failure:

1. **Read one byte** from the UART stream.
2. **Start byte** — If the byte is not `0xAA`, treat it as an invalid start; discard it and go back to step 1. If it is `0xAA`, continue.
3. **Flags** — Read the next 2 bytes and interpret them as a 16-bit value (flags). From this, derive the **payload length** (e.g. bits 15–6 per the header flag explanation).
4. **Payload length** — If the payload length is not in the range 0–1023, the packet is invalid; go back to step 1 and try to find the next `0xAA`.
5. **Payload** — Read exactly that many payload bytes.
6. **CRC** — Read the next two bytes as the 16-bit CRC (low byte, then high byte).
7. **Checksum** — Compute the CRC over the packet (e.g. start byte + flags + payload) and compare it to the received CRC. If it does not match, the packet is invalid; go back to step 1. If it matches, the packet is **valid** and can be handled (e.g. by command ID).

On any failure (invalid start byte, invalid payload length, or invalid checksum), the process discards the bad packet and resumes by looking for the next `0xAA` and repeating from step 1. Only when all checks pass does a packet reach “success” and get processed.


## Binary protocol - Sending commands
Every request sent to the sensor will receive a response. The response also confirms that the request was received and processed. The timeout value and number of retries should be optimized for the specific application.

Below is the process for sending a command request and reading the response:

### Process flow for sending commands

1. **Send request packet** — Transmit the command packet (start byte, flags, payload, CRC) to the sensor.
2. **Wait for response** — Wait up to **100 ms** for a valid response packet to arrive.
3. **Response received?**
   - **No** — Check if **retries have expired**. If yes, the process ends with **Error: No response received**. If retries remain, go back to step 1 and send the request again.
   - **Yes** — Continue to step 4.
4. **Command ID match?** — Check whether the response packet’s command ID matches the command ID of the request you sent.
   - **No** — Go back to step 2 and Wait up to 100 ms for response.
   - **Yes** — **Success.** The response is for your command and can be processed.

The flow uses a 100 ms timeout and retry limit: if no response arrives in time, or if the response’s command ID does not match the request, the host re-sends the request. After the maximum number of retries with no valid response, the host reports an error.

## Binary protocol - Saving
Parameters listed in the command list below, and indicated to persists across power cycles, must be saved to onboard flash once changed.
To save the parameters, the Token (ID 10) must be read from the unit by sending a read command. The value received must then be sent as the data in the Save Parameters command (ID 12) to the unit.
The Token expires every time after use and consecutive save commands will require the request of a new token prior to the save commands sent.

## Binary protocol - Command list

Product register map. Columns: **ID**, **Name**, **RW** (Read/Write), **Read bytes**, **Write bytes**, **Persists**, **Description**.

### Product register map (summary)

| ID | Name | RW | Read bytes | Write bytes | Persists | Description |
|----|------|-----|------------|-------------|----------|-------------|
| 0 | Product name | R | 16 | - | - | See below |
| 1 | Hardware version | R | 4/uint32 | - | - | See below |
| 2 | Firmware version | R | 4 | - | - | See below |
| 3 | Serial number | R | 16 | - | - | See below |
| 9 | User data | RW | 16 | 16 | Yes | See below |
| 10 | Token | R | 2/uint16 | - | - | See below |
| 12 | Save parameters | W | - | 2/uint16 | - | See below |
| 14 | Reset | W | - | 2/uint16 | - | See below |
| 27 | Distance output | RW | 4/uint32 | 4/uint32 | No | See below |
| 28 | Communication Mode | RW | 1/uint8 | 1/uint8 | Yes | See below |
| 30 | Stream | RW | 4/uint32 | 4/uint32 | No | See below |
| 44 | Distance data in cm | R | varies | - | - | See below |
| 45 | Distance data in mm | R | varies | - | - | See below |
| 50 | Laser firing | RW | 1/uint8 | 1/uint8 | No | See below |
| 55 | Temperature | R | 4/uint32 | - | - | See below |
| 85 | Noise | R | 4/uint32 | - | - | See below |
| 90 | Baud rate | RW | 1/uint8 | 1/uint8 | Yes | See below |
| 91 | I²C address | RW | 1/uint8 | 1/uint8 | Yes | See below |
| 92 | Update rate | RW | 4/uint32 | 4/uint32 | Yes | See below |
| 93 | Update rate preselect list | RW | 1/uint8 | 1/uint8 | Yes | See below |
| 94 | Zero offset | RW | 4/int32 | 4/int32 | Yes | See below |
| 95 | Lost signal counter | RW | 4/int32 | 4/int32 | Yes | See below |
| 137 | Median filter enable | RW | 1/uint8 | 1/uint8 | Yes | See below |
| 138 | Median filter size | RW | 4/int32 | 4/int32 | Yes | See below |
| 139 | Smoothing filter enable | RW | 1/uint8 | 1/uint8 | Yes | See below |
| 140 | Smoothing factor | RW | 4/uint32 | 4/uint32 | Yes | See below |
| 141 | Rolling average enable | RW | 1/uint8 | 1/uint8 | Yes | See below |
| 142 | Rolling average size | RW | 4/uint32 | 4/uint32 | Yes | See below |

---

### Command descriptions (word-for-word)

**0 — Product name** (R, 16 bytes read, - write, - persists)  
A 16-byte string indicating product model name. Always SF20 followed by a null terminator. Use to verify the SF20 is connected and operational over the selected interface.

**1 — Hardware version** (R, 4/uint32 read, - write, - persists)  
The hardware revision number as a uint32.

**2 — Firmware version** (R, 4 bytes read, - write, - persists)  
The currently installed firmware version as 4 bytes. Used to identify the product for API compatibility.

| Byte | 1 | 2 | 3 | 4 |
|------|---|---|---|---|
| Meaning | Patch | Minor | Major | Reserved |

**3 — Serial number** (R, 16 bytes read, - write, - persists)  
A 16-byte string (null-terminated) of the serial identifier assigned during production.

**9 — User data** (RW, 16 read, 16 write, Yes persists)  
16 bytes of user data stored and read for any purpose.

**10 — Token** (R, 2/uint16 read, - write, - persists)  
Next usable safety token / Current safety token. Once used, it will expire, and a new token will be created.

**12 — Save parameters** (W, - read, 2/uint16 write, - persists)  
Commands written to, that must be stored and persist across power cycles will be saved to flash memory on the receipt of the latest Token (ID 10) value sent to this command. The safety token prevents unintentional writes. The token expires once a successful save has completed.

**14 — Reset** (W, - read, 2/uint16 write, - persists)  
Writing the safety token to this command will restart the sensor.

**27 — Distance output** (RW, 4/uint32 read, 4/uint32 write, No persists)  
Configuration for (44) distance data command data output. Each bit toggles the output of specified data.

| Bit | Output |
|-----|--------|
| 0 | First return raw |
| 1 | First return closest |
| 2 | First return median |
| 3 | First return furthest |
| 4 | First return strength |
| 5 | Last return raw |
| 6 | Last return closest |
| 7 | Last return median |
| 8 | Last return furthest |
| 9 | Last return strength |
| 10 | Noise |

**28 — Communication Mode** (RW, 1/uint8 read, 1/uint8 write, Yes persists)  
The Communication mode sets the startup state. The following options are available:

| Bit | Mode |
|-----|------|
| 0 | Auto Detect Interface (Default) |
| 1 | Serial UART |
| 2 | I²C |
| 3 | Serial UART (Legacy Header) |
| 4 | Serial UART (no startup message) |

**30 — Stream** (RW, 4/uint32 read, 4/uint32 write, No persists)  
Serial and USB interface only. (If used on I²C, the data will not be retrievable.) Reading from the stream command will indicate what type of data is currently being streamed. Writing to the stream command will set the type of data to be streamed.

| Value | Streamed data |
|-------|----------------|
| 0 | disabled |
| 5 | (44) stream distance data cm |
| 6 | (45) stream distance data mm |

**44 — Distance data in cm** (R, varies read, - write, - persists)  
Distance data in cm as measured by the SF20. This command can be read any time, but if (30) stream is set to 5, this command will automatically output at the measurement update rate. The data included will vary and be packed in order based on the configuration of the (27) distance output command.

| Data output bit | Description | Size |
|-----------------|-------------|------|
| 0 | First return raw (cm) | int16 |
| 1 | First return closest (cm) | int16 |
| 2 | First return median (cm) | int16 |
| 3 | First return furthest (cm) | int16 |
| 4 | First return strength (%) | int16 |
| 5 | Last return raw (cm) | int16 |
| 6 | Last return closest (cm) | int16 |
| 7 | Last return median (cm) | int16 |
| 8 | Last return furthest (cm) | int16 |
| 9 | Last return strength (%) | int16 |
| 10 | Background noise | int16 |

**45 — Distance data in mm** (R, varies read, - write, - persists)  
Distance data in mm as measured by the SF20. This command can be read any time, but if (30) stream is set to 6, this command will automatically output at the measurement update rate. The data included will vary and be packed in order based on the configuration of the (27) distance output command.

| Data output bit | Description | Size |
|-----------------|-------------|------|
| 0 | First return raw (mm) | Int32 |
| 1 | First return closest (mm) | Int32 |
| 2 | First return median (mm) | Int32 |
| 3 | First return furthest (mm) | Int32 |
| 4 | First return strength (%) | Int32 |
| 5 | Last return raw (mm) | Int32 |
| 6 | Last return closest (mm) | Int32 |
| 7 | Last return median (mm) | Int32 |
| 8 | Last return furthest (mm) | Int32 |
| 9 | Last return strength (%) | Int32 |
| 10 | Background noise | Int32 |

**50 — Laser firing** (RW, 1/uint8 read, 1/uint8 write, No persists)  
Reading this command will indicate the current laser firing state. Writing to this command will enable or disable laser firing.

| Value | Description |
|-------|-------------|
| 0 | Disabled |
| 1 | Enabled |

**55 — Temperature** (R, 4/uint32 read, - write, - persists)  
Reading this command will return the measured temperature in 0.01 of a degree.

**85 — Noise** (R, 4/uint32 read, - write, - persists)  
Reading this command will return the level of measured background noise.

**90 — Baud rate** (RW, 1/uint8 read, 1/uint8 write, Yes persists)  
The serial baud rate used by the serial interface. This parameter only takes effect when the serial interface is first enabled after power-up or restart. Reading this command will return the baud rate. Writing to this command will set the baud rate.

| Value | Baud rate (bps) |
|-------|-----------------|
| 0 | 9600 |
| 1 | 19200 |
| 2 | 38400 |
| 3 | 57600 |
| 4 | 115200 |
| 5 | 230400 |
| 6 | 460800 |
| 7 | 921600 |

**91 — I²C address** (RW, 1/uint8 read, 1/uint8 write, Yes persists)  
The I²C address value is in decimal. Reading this command will return the I²C address. Writing this command will set the I²C address.

**92 — Update rate** (RW, 4/uint32 read, 4/uint32 write, Yes persists)  
This variable when read will indicate the current update rate in Hz. When writing to this variable, the update rate can be set in Hz and the closest possible update rate to the request will be used and then reported back.

**93 — Update rate preselect list** (RW, 1/uint8 read, 1/uint8 write, Yes persists)  
Controls the SF20's sampling update rate. Reading this command will return the current update rate. Writing this command will set the update rate.

| Command value | Update rate (samples/second) |
|---------------|------------------------------|
| 1 | 48 |
| 2 | 55 |
| 3 | 64 |
| 4 | 77 |
| 5 | 97 |
| 6 | 129 |
| 7 | 194 |
| 8 | 388 |
| 9 | 625 |
| 10 | 1250 |
| 11 | 2500 |
| 12 | 5000 |

**94 — Zero offset** (RW, 4/int32 read, 4/int32 write, Yes persists)  
Changing this offset value will change the zero-distance position for the output, written and read in mm.

**95 — Lost signal counter** (RW, 4/int32 read, 4/int32 write, Yes persists)  
Sets the number of lost signal returns before a lost signal indication is output on the distance value. The distance output lost signal indication -1000.

**137 — Median filter enable** (RW, 1/uint8 read, 1/uint8 write, Yes persists)  
Reading this command will return the status of the median filter. Writing this command will set the status of the median filter.

| Value | Description |
|-------|-------------|
| 0 | Disabled |
| 1 | Enabled |

**138 — Median filter size** (RW, 4/int32 read, 4/int32 write, Yes persists)  
Reading this command will return the size of the median filter. Writing this command will set the size of the median filter. The valid range is 3 to 32.

**139 — Smoothing filter enable** (RW, 1/uint8 read, 1/uint8 write, Yes persists)  
Reading this command will return the status of the smoothing filter. Writing this command will set the status of the smoothing filter.

| Value | Description |
|-------|-------------|
| 0 | Disabled |
| 1 | Enabled |

**140 — Smoothing factor** (RW, 4/uint32 read, 4/uint32 write, Yes persists)  
Reading this command will return the strength of the smoothing filter. Writing this command will set the strength of the smoothing filter. The valid range is 1 to 99.

**141 — Rolling average enable** (RW, 1/uint8 read, 1/uint8 write, Yes persists)  
Reading this command will return the status of the rolling average filter. Writing this command will set the status of the rolling average filter.

| Value | Description |
|-------|-------------|
| 0 | Disabled |
| 1 | Enabled |

**142 — Rolling average size** (RW, 4/uint32 read, 4/uint32 write, Yes persists)  
Reading this command will return the size of the rolling average filter. Writing this command will set the size of the rolling average filter. The valid range is 2 to 32.

