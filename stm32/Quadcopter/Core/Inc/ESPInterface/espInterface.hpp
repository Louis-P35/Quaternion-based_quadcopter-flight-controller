/*
 * espInterface.hpp
 *
 *  Created on: May 24, 2026
 *      Author: louis
 */
#pragma once

#include <cstdint>
#include <cstdarg>
#include "stm32h7xx_hal.h"

namespace EspSpi {

static constexpr uint16_t FRAME_SIZE    = 256;
static constexpr uint16_t MAGIC_FC_ESP  = 0xBEEF;

static constexpr uint8_t  FRAME_TYPE_ATTITUDE = 0x01;
static constexpr uint8_t  FRAME_TYPE_STATUS   = 0x02;
static constexpr uint8_t  FRAME_TYPE_LOG      = 0x05;
static constexpr uint8_t  FRAME_TYPE_RC       = 0x06;
static constexpr uint8_t  LOG_TEXT_LEN        = 128;

// SBUS data parsed from the ESP32 MISO frame (extension in the pad area at offset 33)
struct SbusFromMiso
{
    bool     valid;          // true if the ESP32 reported has_sbus=1
    bool     frame_lost;
    bool     failsafe;
    uint16_t channels[16];  // raw 11-bit SBUS values
};

enum class LogLevel : uint8_t
{
    LOG_DEBUG = 0,
    LOG_INFO  = 1,
    LOG_WARN  = 2,
    LOG_ERR   = 3,
};

} // namespace EspSpi


class EspInterface
{
public:
    EspInterface(SPI_HandleTypeDef& hspi, GPIO_TypeDef* csPort, uint16_t csPin);

    bool sendAttitude(float qw, float qx, float qy, float qz,
                      float gx, float gy, float gz,
                      float ax, float ay, float az);

    bool sendStatus(const char* fsmState,
                    float batteryVoltage = 0.0f,
                    float batteryCurrent = 0.0f,
                    uint8_t batteryPercent = 0,
                    const uint8_t* motorPercent = nullptr,
                    uint8_t motorCount = 0);

    bool sendLog(EspSpi::LogLevel level, const char* text);
    bool sendLogf(EspSpi::LogLevel level, const char* fmt, ...);

    // Send 16 radio channel values (µs) to the ESP32 for GCS forwarding
    bool sendRc(const uint16_t* channels_us, uint8_t count);

    // Latest SBUS data parsed from the MISO frame (updated after every SPI transaction)
    const EspSpi::SbusFromMiso& getSbusData() const { return m_sbusData; }

private:
    SPI_HandleTypeDef& m_hspi;
    GPIO_TypeDef*      m_csPort;
    uint16_t           m_csPin;

    uint8_t              m_txBuf[EspSpi::FRAME_SIZE];
    uint8_t              m_rxBuf[EspSpi::FRAME_SIZE];
    EspSpi::SbusFromMiso m_sbusData = {};

    bool     transmitFrame(uint8_t frameType, uint8_t payloadLen);
    void     parseMisoSbus();
    uint16_t crc16(const uint8_t* data, uint16_t len);
};
