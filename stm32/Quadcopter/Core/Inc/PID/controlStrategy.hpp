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
	PID m_pid[3];

public:
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
	PID m_pid[3];

public:
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
	PID m_pidPosToAtt[3];
	PID m_pidAttToMotor[3];

public:
	// This method execute the control task
	void execute(const Telemetry& currentState, const Telemetry& targetState) override;
};






