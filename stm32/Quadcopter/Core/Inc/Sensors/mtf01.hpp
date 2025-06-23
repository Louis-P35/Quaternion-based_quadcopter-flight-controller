/*
 * mtf01.hpp
 *
 *  Created on: Jun 11, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Filters/lowPassFilter.hpp"
#include "Sensors/mtf01WrapperC.h"

// Includes from STL
#include <stdint.h>
#include <array>


/*struct MavlinkOpticalFlow_t
{
    uint64_t timeUsec;		// Timestamp (microseconds)
    float flowCompMX;		// Flow in meters/sec (X-axis, body frame)
    float flowCompMY;		// Flow in meters/sec (Y-axis, body frame)
    float groundDistance;	// Ground distance in meters
    int16_t flowX;			// Raw flow in pixels (X-axis)
    int16_t flowY;			// Raw flow in pixels (Y-axis)
    uint8_t sensorId;		// Sensor ID
    uint8_t quality;		// Flow quality (0–255)
};*/


/*
 * Optical flow sensor base class.
 * Using a base class so other optical flow sensors can be easily integrated in the future.
 */
class OpticalFlowSensor
{
protected:
	LPF<float> m_lpfLidar;
	LPF<float> m_lpfVelX;
	LPF<float> m_lpfVelY;

public:
	float m_lidarDist = 0.0f;
	float m_xVelocity = 0.0f;
	float m_yVelocity = 0.0f;

public:
	OpticalFlowSensor() = default;

	virtual bool init() = 0;
	virtual void readSensor() noexcept = 0;

	virtual float getLidarDist() const noexcept = 0;
	virtual float getXVelocity() const noexcept = 0;
	virtual float getYVelocity() const noexcept = 0;
};



class OpticalFlowProtocole
{
public:
	OpticalFlowProtocole() = default;

	virtual bool decodeBuffer() = 0;
};


/*
 * Handle the rx frame with a state machine, as it is received byte per byte.
 */
class MavlinkProtocole : public OpticalFlowProtocole
{
private:
	// Sensor data
	float m_flowX;    	// X optical flow (m/s)
	float m_flowY;    	// Y optical flow (m/s)
	float m_height;   	// Height (meters)
	uint8_t m_quality; 	// Flow quality
	bool m_dataValid; 	// True if latest data is valid

	// Receive buffer
	static constexpr size_t RX_BUFFER_SIZE = 64; // Large enough for MAVLink packets
	std::array<uint8_t, RX_BUFFER_SIZE> m_rxBuffer;
	uint8_t m_rxByte; // Single byte for interrupt reception

	// MAVLink parsing state
	enum class ParseState
	{
		WAITING_FOR_STX,
		LEN,
		SEQ,
		SYSID,
		COMPID,
		MSGID,
		PAYLOAD,
		CRC_L,
		CRC_H
	};
	ParseState m_parseState;

	size_t m_rxIndex;
	size_t m_payloadLength;
	static constexpr uint8_t MAVLINK_STX_V1 = 0xFE; // MAVLink v1 start byte
	static constexpr uint8_t OPTICAL_FLOW_MSG_ID = 100; // MAVLink OPTICAL_FLOW message ID
	static constexpr uint8_t OPTICAL_FLOW_CRC_EXTRA = 175; // CRC seed for OPTICAL_FLOW

	// MAVLink packet structure
	struct MavlinkPacket_t
	{
		uint8_t len;
		uint8_t seq;
		uint8_t sysid;
		uint8_t compid;
		uint8_t msgid;
		std::array<uint8_t, 255> payload; // Max payload size
		uint16_t crc;
	};
	MavlinkPacket_t m_packet;

public:
	MavlinkProtocole() = default;

	void handleByte(const uint8_t& byte);
	virtual bool decodeBuffer() override;

private:
	uint16_t calculateCrc(const uint8_t* pBuffer, const size_t& len, const uint8_t& crcExtra) const; // MAVLink CRC

};


/*
 * MTF-01 optical flow and lidar sensor.
 */
class Mtf01 : public OpticalFlowSensor, public MavlinkProtocole
{
public:
	static constexpr int m_outputFrequency = 100;
	static constexpr float m_lidarRangeMeterMax = 8.0f;
	static constexpr float m_lidarRangeMeterMin = 0.02f;
	static constexpr float m_opticalFlowMinWorkingDistance = 0.08f;

	float m_flowX = 0.0f;
	float m_flowY = 0.0f;
	float m_height = 0.0f;

public:
	Mtf01() = default;

	virtual bool init() override;
	virtual void readSensor() noexcept override;

	virtual float getLidarDist() const noexcept override;
	virtual float getXVelocity() const noexcept override;
	virtual float getYVelocity() const noexcept override;
};



