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
static constexpr uint8_t  FRAME_TYPE_GPS      = 0x07;
static constexpr uint8_t  FRAME_TYPE_MTF01    = 0x08;
static constexpr uint8_t  FRAME_TYPE_MAG      = 0x09;
static constexpr uint8_t  LOG_TEXT_LEN        = 128;

// SBUS data parsed from the ESP32 MISO frame (extension in the pad area at offset 33)
struct SbusFromMiso
{
    bool     valid;          // true if the ESP32 reported has_sbus=1
    bool     frame_lost;
    bool     failsafe;
    uint16_t channels[16];  // raw 11-bit SBUS values
};

// GPS data parsed from the ESP32 MISO frame (offset 66)
struct GpsFromMiso
{
    bool    valid;           // true if the ESP32 reported has_gps=1
    double  latitude;        // decimal degrees
    double  longitude;       // decimal degrees
    float   altitude_m;      // meters above sea level
    float   speed_ms;        // ground speed, m/s
    float   heading_deg;     // course over ground, degrees
    uint8_t satellites;
    uint8_t fix_type;        // 0=none, 1=2D, 2=3D
};

// MTF-01 data parsed from the ESP32 MISO frame (offset 97)
struct Mtf01FromMiso
{
    bool    valid;           // true if the ESP32 reported has_mtf01=1
    float   distance_m;      // lidar range, metres
    int16_t flow_x;          // optical flow X, dpix/s
    int16_t flow_y;          // optical flow Y, dpix/s
    uint8_t quality;         // 0–255
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

    // Forward GPS data received via MISO to the GCS
    bool sendGps();

    // Forward MTF-01 data received via MISO to the GCS
    bool sendMtf01();

    // Send filtered IMU magnetometer values to the GCS (int16_t raw counts after bias correction)
    bool sendMag(int16_t x, int16_t y, int16_t z);

    // Latest SBUS data parsed from the MISO frame (updated after every SPI transaction)
    const EspSpi::SbusFromMiso&  getSbusData()  const { return m_sbusData;  }

    // Latest GPS and MTF-01 data parsed from the MISO frame
    const EspSpi::GpsFromMiso&   getGpsData()   const { return m_gpsData;   }
    const EspSpi::Mtf01FromMiso& getMtf01Data() const { return m_mtf01Data; }

private:
    SPI_HandleTypeDef& m_hspi;
    GPIO_TypeDef*      m_csPort;
    uint16_t           m_csPin;

    uint8_t               m_txBuf[EspSpi::FRAME_SIZE];
    uint8_t               m_rxBuf[EspSpi::FRAME_SIZE];
    EspSpi::SbusFromMiso  m_sbusData  = {};
    EspSpi::GpsFromMiso   m_gpsData   = {};
    EspSpi::Mtf01FromMiso m_mtf01Data = {};

    bool     transmitFrame(uint8_t frameType, uint8_t payloadLen);
    void     parseMisoFrame();
    uint16_t crc16(const uint8_t* data, uint16_t len);
};
