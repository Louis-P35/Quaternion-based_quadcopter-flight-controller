/*
 * mtf01.hpp
 *
 *  Created on: Jun 11, 2025
 *      Author: louis
 */

#pragma once


/*
 * Optical flow sensor base class.
 * Using a base class so other optical flow sensors can be easily integrated in the future.
 */
class OpticalFlowSensor
{
public:
	OpticalFlowSensor() = default;

	virtual bool init() = 0;
	virtual void readSensor() noexcept override = 0;

	virtual float getLidarDist() const noexcept = 0;
	virtual float getXVelocity() const noexcept = 0;
	virtual float getYVelocity() const noexcept = 0;
};


/*
 * MTF-01 optical flow and lidar sensor.
 */
class Mtf01 : public OpticalFlowSensor
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



