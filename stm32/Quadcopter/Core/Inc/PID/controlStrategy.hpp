/*
 * controlStrategy.hpp
 *
 *  Created on: Jun 17, 2024
 *      Author: Louis
 */

#pragma once

// Project
#include "PID/pid.hpp"

// External lib
#include "Utils/Eigen/Dense"


enum StabilizationMode { ACRO, STAB, HORIZON, POSHOLD };


class ControlStrategy
{
public:
	StabilizationMode m_flightMode = StabilizationMode::STAB;

private:
	PIDBlock m_rateLoop[3]; // Roll, pitch, yaw
	PIDBlock m_angleLoop[3]; // Roll, pitch, yaw
	PIDBlock m_posLoop[3]; // Roll (x), pitch (y), height (z)

public:
	ControlStrategy() {};
	void angleControlLoop(const double dt);
	void rateControlLoop(const double dt);
	void posControlLoop(const double dt);

	void getTorqueVector(double& tx, double& ty, double& tz);

private:
	void setAngleTarget();
	void setRateTarget();
};





