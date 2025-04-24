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

// Strategy design pattern



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



/*
 * This class is the base abstract class for the control strategy.
 */
class ControlStrategy
{
public:
	virtual ~ControlStrategy() = default;

	// This method execute the control task
	virtual void execute(const Telemetry& currentState, const Telemetry& targetState) = 0;
};


/*
 * This class implement the attitude control
 * strategy with a single PID loop.
 * Attitude error -> PID -> motor power
 */
class AttitudeControlStrategy : public ControlStrategy
{
public:
	// PID instances for each axis
	PID m_pidRoll;
	PID m_pidPitch;
	PID m_pidYaw;

public:
	AttitudeControlStrategy(
			const float kp,
			const float ki,
			const float kd,
			const float kpYaw,
			const float kiYaw,
			const float kdYaw
			) :
				m_pidRoll(kp, ki, kd, 100.0f),
				m_pidPitch(kp, ki, kd, 100.0f),
				m_pidYaw(kpYaw, kiYaw, kdYaw, 100.0f) {};

	// This method execute the control task
	void execute(const Telemetry& currentState, const Telemetry& targetState) override;
};


/*
 * This class implement the angular rate control
 * strategy with a single PID loop.
 * Angular velocity error -> PID -> motor power
 */
class RateControlStrategy : public ControlStrategy
{
public:
	// PID instances for each axis
	PID m_pidRoll;
	PID m_pidPitch;
	PID m_pidYaw;

public:
	RateControlStrategy(
			const float kp,
			const float ki,
			const float kd,
			const float kpYaw,
			const float kiYaw,
			const float kdYaw
			) :
				m_pidRoll(kp, ki, kd, 100.0f),
				m_pidPitch(kp, ki, kd, 100.0f),
				m_pidYaw(kpYaw, kiYaw, kdYaw, 100.0f) {};

	// This method execute the control task
	void execute(const Telemetry& currentState, const Telemetry& targetState) override;
};


/*
 * This class implement the attitude control
 * strategy with a dual PID loop.
 * Position error -> PID -> Attitude (get error) -> PID -> motor power
 */
class PositionControlStrategy : public ControlStrategy
{
public:
	// PID instances for each axis
	PID m_pidPosToAttRoll;
	PID m_pidPosToAttPitch;
	PID m_pidPosToAttYaw;
	PID m_pidAttToMotorRoll;
	PID m_pidAttToMotorPitch;
	PID m_pidAttToMotorYaw;

public:
	PositionControlStrategy(
			const float kp1,
			const float ki1,
			const float kd1,
			const float kp1Yaw,
			const float ki1Yaw,
			const float kd1Yaw,
			const float kp2,
			const float ki2,
			const float kd2,
			const float kp2Yaw,
			const float ki2Yaw,
			const float kd2Yaw
			) :
			m_pidPosToAttRoll(kp1, ki1, kd1, 100.0f),
			m_pidPosToAttPitch(kp1, ki1, kd1, 100.0f),
			m_pidPosToAttYaw(kp1Yaw, ki1Yaw, kd1Yaw, 100.0f),
			m_pidAttToMotorRoll(kp2, ki2, kd2, 100.0f),
			m_pidAttToMotorPitch(kp2, ki2, kd2, 100.0f),
			m_pidAttToMotorYaw(kp2Yaw, ki2Yaw, kd2Yaw, 100.0f){};

	// This method execute the control task
	void execute(const Telemetry& currentState, const Telemetry& targetState) override;
};






