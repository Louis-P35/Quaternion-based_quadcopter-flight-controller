/*
 * espInterface.cpp
 *
 *  Created on: May 24, 2026
 *      Author: louis
 */
#include "ESPInterface/espInterface.hpp"

#include <cstring>
#include <cstdio>

namespace
{

struct __attribute__((packed)) FrameHeader
{
    uint16_t magic;
    uint8_t  type;
    uint8_t  payload_len;
};

struct __attribute__((packed)) PayloadAttitude
{
    float qw, qx, qy, qz;  // quaternion
    float gx, gy, gz;      // angular velocity, deg/s
    float ax, ay, az;      // acceleration, g
};

static constexpr uint8_t ATTITUDE_PAYLOAD_SIZE = sizeof(PayloadAttitude);  // 40

struct __attribute__((packed)) PayloadLog
{
    uint8_t level;
    char    text[EspSpi::LOG_TEXT_LEN];
};

static constexpr uint8_t LOG_PAYLOAD_SIZE = sizeof(PayloadLog);  // 129

} // namespace


EspInterface::EspInterface(SPI_HandleTypeDef& hspi, GPIO_TypeDef* csPort, uint16_t csPin)
    : m_hspi(hspi), m_csPort(csPort), m_csPin(csPin)
{}


bool EspInterface::sendAttitude(float qw, float qx, float qy, float qz,
                                float gx, float gy, float gz,
                                float ax, float ay, float az)
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    auto* p = reinterpret_cast<PayloadAttitude*>(m_txBuf + sizeof(FrameHeader));
    p->qw = qw; p->qx = qx; p->qy = qy; p->qz = qz;
    p->gx = gx; p->gy = gy; p->gz = gz;
    p->ax = ax; p->ay = ay; p->az = az;

    return transmitFrame(EspSpi::FRAME_TYPE_ATTITUDE, ATTITUDE_PAYLOAD_SIZE);
}


bool EspInterface::sendLog(EspSpi::LogLevel level, const char* text)
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    auto* payload  = reinterpret_cast<PayloadLog*>(m_txBuf + sizeof(FrameHeader));
    payload->level = static_cast<uint8_t>(level);
    strncpy(payload->text, text, EspSpi::LOG_TEXT_LEN - 1);

    return transmitFrame(EspSpi::FRAME_TYPE_LOG, LOG_PAYLOAD_SIZE);
}


bool EspInterface::sendLogf(EspSpi::LogLevel level, const char* fmt, ...)
{
    char buf[EspSpi::LOG_TEXT_LEN];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    return sendLog(level, buf);
}


bool EspInterface::transmitFrame(uint8_t frameType, uint8_t payloadLen)
{
    // Write header at start of already-zeroed buffer
    auto* hdr      = reinterpret_cast<FrameHeader*>(m_txBuf);
    hdr->magic       = EspSpi::MAGIC_FC_ESP;
    hdr->type        = frameType;
    hdr->payload_len = payloadLen;

    // CRC placed immediately after the actual payload (not at a fixed offset)
    const uint16_t crcOffset = sizeof(FrameHeader) + payloadLen;
    uint16_t crc = crc16(m_txBuf, crcOffset);
    memcpy(m_txBuf + crcOffset, &crc, sizeof(crc));

    if (m_csPort) HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&m_hspi, m_txBuf, EspSpi::FRAME_SIZE, 10);
    if (m_csPort) HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);

    return status == HAL_OK;
}


uint16_t EspInterface::crc16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}
