/*
 * controlStrategy.hpp
 *
 *  Created on: Jun 17, 2024
 *      Author: Louis
 */

#pragma once

#include "PID/pid.hpp"
#include "Utils/VectorNd.hpp"
#include "Utils/Quaternion.hpp"

// Composite design pattern



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
	Quaternion m_attitude;

	// Angular velocity of the drone
	Vector<double, 3> m_angularRate;

	// Absolute (X, Y, Z) position of the drone
	Vector<double, 3> m_position;

	// Power of the 4 motors, scaled between [0, 1]
	double m_motorPower[4];
};



/*
 * This class is the base abstract class for the
 * composite design used to combine flight control
 * strategy.
 */
class Control
{
public:
	// This method execute the control task
	virtual void execute(const double errors[3]) = 0;

	// This method compute the error to be corrected
	virtual void getError(const Telemetry& actualState, Telemetry& targetState) = 0;
};


/*
 *	This class handle the control strategy.
 *	It handle chained PID loops.
 */
class CompositeControl : public Control
{
private:
	// Up to 3 PID can be chained
	// Max being position -> PID -> attitude -> PID -> angular rate -> PID -> motor power
	Control* m_pControls[3] = {nullptr};

	/*
	 * This method add a PID to the chained control strategy
	 */
	bool addControl(Control* pControl)
	{
		for (auto& ctrl : m_pControls)
		{
			if (ctrl == nullptr)
			{
				ctrl = pControl;
				return true;
			}
		}

		return false;
	}
};


/*
 * This class implement the attitude control
 * strategy with a single PID loop.
 * Attitude error -> PID -> motor power
 */
class AttitudeToMotors : public Control
{
public:
	// PID instances for each axis
	PID m_pid[3];

public:
	// This method execute the control task
	void execute(const double errors[3]) override;

	// This method compute the error to be corrected
	void getError(const Telemetry& actualState, Telemetry& targetState) override;
};


/*
 * This class implement the angular rate control
 * strategy with a single PID loop.
 * Angular velocity error -> PID -> motor power
 */
class AngularRateToMotors : public Control
{
public:
	// PID instances for each axis
	PID m_pid[3];

public:
	// This method execute the control task
	void execute(const double errors[3]) override;

	// This method compute the error to be corrected
	void getError(const Telemetry& actualState, Telemetry& targetState) override;
};


/*
 * This class implement the attitude control
 * strategy with a dual PID loop.
 * Attitude error -> PID -> angular rate (get error) -> PID -> motor power
 */
class AttitudeToRate : public Control
{
public:
	// PID instances for each axis
	PID m_pid[3];

public:
	// This method execute the control task
	void execute(const double errors[3]) override;

	// This method compute the error to be corrected
	void getError(const Telemetry& actualState, Telemetry& targetState) override;
};


class PositionToRate : public Control
{
public:
	// PID instances for each axis
	PID m_pid[3];

public:
	// This method execute the control task
	void execute(const double errors[3]) override;

	// This method compute the error to be corrected
	void getError(const Telemetry& actualState, Telemetry& targetState) override;
};


class PositionToAttitude : public Control
{
public:
	// PID instances for each axis
	PID m_pid[3];

public:
	// This method execute the control task
	void execute(const double errors[3]) override;

	// This method compute the error to be corrected
	void getError(const Telemetry& actualState, Telemetry& targetState) override;
};




