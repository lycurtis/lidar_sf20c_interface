// drivers/lidar.c
/**
 * LightWare SF20/C LiDAR — binary protocol driver
 * Byte-at-a-time state machine that frames, validates, and parses
 * LightWare binary packets arriving on UART.
 */

#include "lidar.h"
