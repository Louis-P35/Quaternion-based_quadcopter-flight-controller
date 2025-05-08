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
	void angleControlLoop(const double dt);
	void rateControlLoop(const double dt, const Vector3<double>& gyro, Radio& radio);
	void posControlLoop(const double dt);

	void getTorqueVector(double& tx, double& ty, double& tz);

	// PIDs coeff setters
	void setPIDsatMinMax(const double sat, const double minOut, const double maxOut);
	void setRatePIDcoefsRoll(const double kp, const double ki, const double kd);
	void setRatePIDcoefsPitch(const double kp, const double ki, const double kd);
	void setRatePIDcoefsYaw(const double kp, const double ki, const double kd);
	void setAnglePIDcoefsRoll(const double kp, const double ki, const double kd);
	void setAnglePIDcoefsPitch(const double kp, const double ki, const double kd);
	void setAnglePIDcoefsYaw(const double kp, const double ki, const double kd);
	void setPosPIDcoefsRoll(const double kp, const double ki, const double kd);
	void setPosPIDcoefsPitch(const double kp, const double ki, const double kd);
	void setPosPIDcoefsYaw(const double kp, const double ki, const double kd);

private:
	void setAngleTarget();
	void setRateTarget(Radio& radio);
};





