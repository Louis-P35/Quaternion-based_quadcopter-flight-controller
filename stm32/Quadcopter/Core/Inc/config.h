/*
 * config.h
 *
 *  Created on: Jan 5, 2026
 *      Author: louis
 */

#pragma once

/*
 * ============================================================================
 * Sensor Configuration
 * ============================================================================
 * Enable/disable sensors at compile-time
 */

// MTF-01 Optical Flow Sensor (UART4)
#define SENSOR_MTF01_ENABLED    0   // 1 = enabled, 0 = disabled

// Radio source: 1 = read SBUS from ESP32 via SPI, 0 = STM32 UART6 SBUS
#define RADIO_SOURCE_SPI        0

