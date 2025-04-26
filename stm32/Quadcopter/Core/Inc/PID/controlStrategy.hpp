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


enum StabilizationMode { ACRO, STAB, HORIZON };
enum ControlMode { POSHOLD, RADIO };


/*
 * Telemetry class.
 * Store the state of the drone.
 * Two instances of this class must be made:
 * 		- One that measure the actual state of the drone
 * 		- One that store the target state of the drone
 */
class Telemetry
{
public:
	// Attitude (angle) of the drone
	Eigen::Quaternionf m_attitude;

	// Angular velocity of the drone
	Eigen::Vector3f m_angularRate;

	// Absolute (X, Y, Z) position of the drone
	Eigen::Vector3f m_position;
};


struct PIDs_out
{
	float roll;
	float pitch;
	float yaw;
};

class ControlStrategy
{
public:
	StabilizationMode m_flightMode = StabilizationMode::STAB;
	ControlMode m_controlMode = ControlMode::RADIO;

	PIDBlock m_rateLoop[3]; // Roll, pitch, yaw
	PIDBlock m_angleLoop[3]; // Roll, pitch, yaw
	PIDBlock m_linearVelocityLoop[3]; // Roll (x), pitch (y), height (z)


	ControlStrategy() {};
	PIDs_out controlLoop(const float dt, const bool ctrlLoop);
};





