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

// MISO GPS section starts at byte 66: has_gps(1B) + SpiPayloadGps(30B)
// SpiPayloadGps: latitude(8B) longitude(8B) altitude_m(4B) speed_ms(4B) heading_deg(4B) satellites(1B) fix_type(1B)
static constexpr uint16_t MISO_GPS_OFFSET   = 66;

// MISO MTF-01 section starts at byte 97: has_mtf01(1B) + SpiPayloadMtf01(9B)
// SpiPayloadMtf01: distance_m(4B) flow_x(2B) flow_y(2B) quality(1B)
static constexpr uint16_t MISO_MTF01_OFFSET = 97;

// MISO GPS-module compass section starts at byte 107: has_mag(1B) + SpiPayloadMag(6B)
// SpiPayloadMag: x(2B) y(2B) z(2B) — raw counts, HMC5883L/QMC5883L on BN-880, updated at 5 Hz
static constexpr uint16_t MISO_GPS_MAG_OFFSET = 107;

// MISO baro section starts at byte 114: has_baro(1B) + SpiPayloadBaro(12B)
// SpiPayloadBaro: pressure_pa(4B) temperature_c(4B) altitude_m(4B)
static constexpr uint16_t MISO_BARO_OFFSET  = 114;

// FC → ESP32 (MOSI) GPS payload — mirrors SpiPayloadGps on the ESP32 side
struct __attribute__((packed)) PayloadGps
{
    double  latitude;
    double  longitude;
    float   altitude_m;
    float   speed_ms;
    float   heading_deg;
    uint8_t satellites;
    uint8_t fix_type;
};
static constexpr uint8_t GPS_PAYLOAD_SIZE = sizeof(PayloadGps);  // 30

// FC → ESP32 (MOSI) MTF-01 payload — mirrors SpiPayloadMtf01 on the ESP32 side
struct __attribute__((packed)) PayloadMtf01
{
    float   distance_m;
    int16_t flow_x;
    int16_t flow_y;
    uint8_t quality;
};
static constexpr uint8_t MTF01_PAYLOAD_SIZE = sizeof(PayloadMtf01);  // 9

// FC → ESP32 (MOSI) barometer payload — mirrors SpiPayloadBaro on the ESP32 side
struct __attribute__((packed)) PayloadBaro
{
    float pressure_pa;
    float temperature_c;
    float altitude_m;
};
static constexpr uint8_t BARO_PAYLOAD_SIZE = sizeof(PayloadBaro);  // 12

// FC → ESP32 (MOSI) magnetometer payload — mirrors SpiPayloadMag on the ESP32 side
// Values are bias-corrected raw counts from the IMU's AK09916, after BiquadLPF at 100 Hz
struct __attribute__((packed)) PayloadMag
{
    int16_t x;
    int16_t y;
    int16_t z;
};
static constexpr uint8_t MAG_PAYLOAD_SIZE = sizeof(PayloadMag);  // 6

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

    parseMisoFrame();

    return status == HAL_OK;
}


void EspInterface::parseMisoFrame()
{
    uint16_t magic;
    memcpy(&magic, m_rxBuf, sizeof(magic));
    if (magic != SPI_MAGIC_ESP_TO_FC)
    {
        m_sbusData   = {};
        m_gpsData    = {};
        m_gpsMagData = {};
        m_mtf01Data  = {};
        m_baroData   = {};
        return;
    }

    // Parse SBUS (offset 33)
    const auto* ext    = reinterpret_cast<const MisoSbusExt*>(m_rxBuf + MISO_SBUS_OFFSET);
    m_sbusData.valid      = ext->has_sbus != 0;
    m_sbusData.frame_lost = !m_sbusData.valid;
    m_sbusData.failsafe   = !m_sbusData.valid;
    if (m_sbusData.valid)
        memcpy(m_sbusData.channels, ext->channels, sizeof(m_sbusData.channels));

    // Parse GPS (offset 66) — use memcpy to avoid unaligned double access on Cortex-M7
    const uint8_t* gpsPtr = m_rxBuf + MISO_GPS_OFFSET;
    m_gpsData.valid = (gpsPtr[0] != 0);
    if (m_gpsData.valid)
    {
        memcpy(&m_gpsData.latitude,    gpsPtr + 1,  sizeof(double));
        memcpy(&m_gpsData.longitude,   gpsPtr + 9,  sizeof(double));
        memcpy(&m_gpsData.altitude_m,  gpsPtr + 17, sizeof(float));
        memcpy(&m_gpsData.speed_ms,    gpsPtr + 21, sizeof(float));
        memcpy(&m_gpsData.heading_deg, gpsPtr + 25, sizeof(float));
        m_gpsData.satellites = gpsPtr[29];
        m_gpsData.fix_type   = gpsPtr[30];
    }

    // Parse MTF-01 (offset 97) — use memcpy to avoid unaligned float access
    const uint8_t* mtfPtr = m_rxBuf + MISO_MTF01_OFFSET;
    m_mtf01Data.valid = (mtfPtr[0] != 0);
    if (m_mtf01Data.valid)
    {
        memcpy(&m_mtf01Data.distance_m, mtfPtr + 1, sizeof(float));
        memcpy(&m_mtf01Data.flow_x,     mtfPtr + 5, sizeof(int16_t));
        memcpy(&m_mtf01Data.flow_y,     mtfPtr + 7, sizeof(int16_t));
        m_mtf01Data.quality = mtfPtr[9];
    }

    // Parse GPS-module compass (offset 107) — int16_t, aligned, direct cast safe
    const uint8_t* gpsMagPtr = m_rxBuf + MISO_GPS_MAG_OFFSET;
    m_gpsMagData.valid = (gpsMagPtr[0] != 0);
    if (m_gpsMagData.valid)
    {
        memcpy(&m_gpsMagData.x, gpsMagPtr + 1, sizeof(int16_t));
        memcpy(&m_gpsMagData.y, gpsMagPtr + 3, sizeof(int16_t));
        memcpy(&m_gpsMagData.z, gpsMagPtr + 5, sizeof(int16_t));
    }

    // Parse baro (offset 114) — use memcpy to avoid unaligned float access
    const uint8_t* baroPtr = m_rxBuf + MISO_BARO_OFFSET;
    m_baroData.valid = (baroPtr[0] != 0);
    if (m_baroData.valid)
    {
        memcpy(&m_baroData.pressure_pa,   baroPtr + 1, sizeof(float));
        memcpy(&m_baroData.temperature_c, baroPtr + 5, sizeof(float));
        memcpy(&m_baroData.altitude_m,    baroPtr + 9, sizeof(float));
    }
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


bool EspInterface::sendGps()
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    // Write through a packed pointer — GCC generates safe unaligned writes for packed structs
    auto* p = reinterpret_cast<PayloadGps*>(m_txBuf + sizeof(FrameHeader));
    p->latitude    = m_gpsData.latitude;
    p->longitude   = m_gpsData.longitude;
    p->altitude_m  = m_gpsData.altitude_m;
    p->speed_ms    = m_gpsData.speed_ms;
    p->heading_deg = m_gpsData.heading_deg;
    p->satellites  = m_gpsData.satellites;
    p->fix_type    = m_gpsData.fix_type;

    return transmitFrame(EspSpi::FRAME_TYPE_GPS, GPS_PAYLOAD_SIZE);
}


bool EspInterface::sendMtf01()
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    auto* p = reinterpret_cast<PayloadMtf01*>(m_txBuf + sizeof(FrameHeader));
    p->distance_m = m_mtf01Data.distance_m;
    p->flow_x     = m_mtf01Data.flow_x;
    p->flow_y     = m_mtf01Data.flow_y;
    p->quality    = m_mtf01Data.quality;

    return transmitFrame(EspSpi::FRAME_TYPE_MTF01, MTF01_PAYLOAD_SIZE);
}


bool EspInterface::sendBaro()
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    auto* p = reinterpret_cast<PayloadBaro*>(m_txBuf + sizeof(FrameHeader));
    p->pressure_pa   = m_baroData.pressure_pa;
    p->temperature_c = m_baroData.temperature_c;
    p->altitude_m    = m_baroData.altitude_m;

    return transmitFrame(EspSpi::FRAME_TYPE_BARO, BARO_PAYLOAD_SIZE);
}


bool EspInterface::sendMag(int16_t x, int16_t y, int16_t z)
{
    memset(m_txBuf, 0, EspSpi::FRAME_SIZE);

    auto* p = reinterpret_cast<PayloadMag*>(m_txBuf + sizeof(FrameHeader));
    p->x = x;
    p->y = y;
    p->z = z;

    return transmitFrame(EspSpi::FRAME_TYPE_MAG, MAG_PAYLOAD_SIZE);
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
