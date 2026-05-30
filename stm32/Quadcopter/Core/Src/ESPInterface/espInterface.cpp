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

// ESP32 → FC (MISO): SBUS extension in the pad area starting at byte 33
// Layout: [magic 2B][has_cmd 1B][cmd_type 1B][cmd 27B][crc 2B] = 33B, then this struct
// has_sbus=0 encodes failsafe (no valid frame); no separate frame_lost/failsafe bytes.
struct __attribute__((packed)) MisoSbusExt
{
    uint8_t  has_sbus;
    uint16_t channels[16];
};
static constexpr uint16_t MISO_SBUS_OFFSET    = 33;
static constexpr uint16_t SPI_MAGIC_ESP_TO_FC = 0xCAFE;

struct __attribute__((packed)) PayloadStatus
{
    float   battery_voltage;
    float   battery_current;
    uint8_t battery_percent;
    char    state[32];
    uint8_t motor_percent[8]; // unused by STM32, zeroed
    uint8_t wifi_rssi;        // unused by STM32, ESP32 overwrites with WiFi.RSSI()
};

static constexpr uint8_t STATUS_PAYLOAD_SIZE = sizeof(PayloadStatus);  // 50

struct __attribute__((packed)) PayloadLog
{
    uint8_t level;
    char    text[EspSpi::LOG_TEXT_LEN];
};

static constexpr uint8_t LOG_PAYLOAD_SIZE = sizeof(PayloadLog);  // 129

struct __attribute__((packed)) PayloadRc
{
    uint16_t channels[16];  // µs values [1000, 2000]
    uint8_t  rssi;          // not known by STM32 — ESP32 uses its own RSSI
};
static constexpr uint8_t RC_PAYLOAD_SIZE = sizeof(PayloadRc);  // 33

} // namespace


EspInterface::EspInterface(SPI_HandleTypeDef& hspi, GPIO_TypeDef* csPort, uint16_t csPin)
    : m_hspi(hspi), m_csPort(csPort), m_csPin(csPin)
{
    memset(m_rxBuf, 0, sizeof(m_rxBuf));
}


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


bool EspInterface::sendStatus(const char* fsmState,
                              float batteryVoltage,
                              float batteryCurrent,
                              uint8_t batteryPercent,
                              const uint8_t* motorPercent,
                              uint8_t motorCount)
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    auto* p = reinterpret_cast<PayloadStatus*>(m_txBuf + sizeof(FrameHeader));
    p->battery_voltage = batteryVoltage;
    p->battery_current = batteryCurrent;
    p->battery_percent = batteryPercent;
    strncpy(p->state, fsmState, sizeof(p->state) - 1);

    if (motorPercent && motorCount > 0)
    {
        const uint8_t n = motorCount < sizeof(p->motor_percent) ? motorCount : sizeof(p->motor_percent);
        for (uint8_t i = 0; i < n; ++i)
            p->motor_percent[i] = motorPercent[i];
    }

    return transmitFrame(EspSpi::FRAME_TYPE_STATUS, STATUS_PAYLOAD_SIZE);
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
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&m_hspi, m_txBuf, m_rxBuf, EspSpi::FRAME_SIZE, 10);
    if (m_csPort) HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);

    parseMisoSbus();

    return status == HAL_OK;
}


void EspInterface::parseMisoSbus()
{
    uint16_t magic;
    memcpy(&magic, m_rxBuf, sizeof(magic));
    if (magic != SPI_MAGIC_ESP_TO_FC)
    {
        m_sbusData = {};
        return;
    }
    const auto* ext = reinterpret_cast<const MisoSbusExt*>(m_rxBuf + MISO_SBUS_OFFSET);
    m_sbusData.valid      = ext->has_sbus != 0;
    m_sbusData.frame_lost = !m_sbusData.valid;
    m_sbusData.failsafe   = !m_sbusData.valid;
    if (m_sbusData.valid)
        memcpy(m_sbusData.channels, ext->channels, sizeof(m_sbusData.channels));
}


bool EspInterface::sendRc(const uint16_t* channels_us, uint8_t count)
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    auto* p = reinterpret_cast<PayloadRc*>(m_txBuf + sizeof(FrameHeader));
    const uint8_t n = count < 16 ? count : 16;
    for (uint8_t i = 0; i < n; ++i)
        p->channels[i] = channels_us[i];

    return transmitFrame(EspSpi::FRAME_TYPE_RC, RC_PAYLOAD_SIZE);
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
