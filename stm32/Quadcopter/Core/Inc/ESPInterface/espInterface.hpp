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
static constexpr uint8_t  LOG_TEXT_LEN        = 128;

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

private:
    SPI_HandleTypeDef& m_hspi;
    GPIO_TypeDef*      m_csPort;
    uint16_t           m_csPin;

    uint8_t m_txBuf[EspSpi::FRAME_SIZE];

    bool     transmitFrame(uint8_t frameType, uint8_t payloadLen);
    uint16_t crc16(const uint8_t* data, uint16_t len);
};
