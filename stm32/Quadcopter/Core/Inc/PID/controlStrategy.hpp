/*
 * controlStrategy.hpp
 *
 *  Created on: Jun 17, 2024
 *      Author: Louis
 */

#pragma once

// Project
#include "PID/pid.hpp"
#include "Radio.hpp"
#include "Utils/Vector.hpp"

// External lib
#include "Utils/Eigen/Dense"


enum StabilizationMode { ACRO, STAB, HORIZON, POSHOLD };


class ControlStrategy
{
public:
	StabilizationMode m_flightMode = StabilizationMode::ACRO;

public:
	PIDBlock m_rateLoop[3]; // Roll, pitch, yaw
	PIDBlock m_angleLoop[3]; // Roll, pitch, yaw
	PIDBlock m_posLoop[3]; // Roll (x), pitch (y), height (z)

public:
	ControlStrategy() {};
	void angleControlLoop(const float dt);
	void rateControlLoop(const float dt, const Vector3<float>& gyro, Radio& radio);
	void posControlLoop(const float dt);

	void getTorqueVector(float& tx, float& ty, float& tz);

	// PIDs coeff setters
	void setPIDsatMinMax(const float sat, const float minOut, const float maxOut);
	void setRatePIDcoefsRoll(const float kp, const float ki, const float kd);
	void setRatePIDcoefsPitch(const float kp, const float ki, const float kd);
	void setRatePIDcoefsYaw(const float kp, const float ki, const float kd);
	void setRatePIDderivativeMode(const DerivativeMode& derivativeMode);
	void setAnglePIDcoefsRoll(const float kp, const float ki, const float kd);
	void setAnglePIDcoefsPitch(const float kp, const float ki, const float kd);
	void setAnglePIDcoefsYaw(const float kp, const float ki, const float kd);
	void setAnglePIDderivativeMode(const DerivativeMode& derivativeMode);
	void setPosPIDcoefsRoll(const float kp, const float ki, const float kd);
	void setPosPIDcoefsPitch(const float kp, const float ki, const float kd);
	void setPosPIDcoefsYaw(const float kp, const float ki, const float kd);
	void setPosPIDderivativeMode(const DerivativeMode& derivativeMode);

private:
	void setAngleTarget();
	void setRateTarget(Radio& radio);
};





