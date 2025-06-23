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

	virtual bool decodeBuffer(const std::array<uint8_t, MTF01_FRAME_SIZE>& pRxBuffer, float& flowX, float& flowY, float& height) = 0;
};


class MavlinkProtocole : public OpticalFlowProtocole
{
public:
	MavlinkProtocole() = default;

	virtual bool decodeBuffer(const std::array<uint8_t, MTF01_FRAME_SIZE>& pRxBuffer, float& flowX, float& flowY, float& height) override;
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

public:
	Mtf01() = default;

	virtual bool init() override;
	virtual void readSensor() noexcept override;

	virtual float getLidarDist() const noexcept override;
	virtual float getXVelocity() const noexcept override;
	virtual float getYVelocity() const noexcept override;
};



