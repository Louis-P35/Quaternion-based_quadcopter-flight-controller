/*
 * stateMachine.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

// Includes from project
#include "stateMachine.hpp"
#include "logManager.hpp"
#include "PID/pid.hpp"


void StartupSequenceState::handleState(const double dt, DroneController& dc)
{
	m_time += dt;

	// All motors at 0% power
	dc.setMotorPower(Motor::eMotor1, 0.0);
	dc.setMotorPower(Motor::eMotor2, 0.0);
	dc.setMotorPower(Motor::eMotor3, 0.0);
	dc.setMotorPower(Motor::eMotor3, 0.0);

	// Wait 1s
	if (m_time > 1.0)
	{
		m_time = 0.0; // Reset time

		// Goto idle state
		StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
	}
}


void IdleState::handleState(const double dt, DroneController& dc)
{
	// All motors at 0% power
	dc.setMotorPower(Motor::eMotor1, 0.0);
	dc.setMotorPower(Motor::eMotor2, 0.0);
	dc.setMotorPower(Motor::eMotor3, 0.0);
	dc.setMotorPower(Motor::eMotor3, 0.0);

	// Wait for throttle all the way down from the controller
	if (dc.m_radio.m_targetThrust < 0.05)
	{
		// Goto ready to take off state
		StateMachine::getInstance().setState(StateMachine::getInstance().getReadyToTakeOffState());
	}
}


void ReadyToTakeOffState::handleState(const double dt, DroneController& dc)
{
	// Wait for throttle little increase from the controller
	if (dc.m_radio.m_targetThrust > 0.06)
	{
		// Goto ready to flying state
		StateMachine::getInstance().setState(StateMachine::getInstance().getTakeOffState());
	}
}


void TakeOffState::handleState(const double dt, DroneController& dc)
{
	// TODO: Handle take off autonomously

	// Goto to flying state
	StateMachine::getInstance().setState(StateMachine::getInstance().getFlyingState());
}


/*
 * Handle flying
 */
void FlyingState::handleState(const double dt, DroneController& dc)
{
	Quaternion qEst = Quaternion::canonical(dc.m_madgwickFilter.m_qEst);
	Quaternion qTarget = Quaternion::canonical(dc.m_targetAttitude);

	// Get attitude error
	Quaternion qError = PID::getError(qEst, qTarget);

	//Quaternion qTest = qError * qEst;
	//qTest.normalize();

	// Get the angle and axis of rotation
	Vector3 rotAxis;
	double angleRad = 0.0;
	qError.toAxisAngle(rotAxis, angleRad);

	// Projection of the rotation axis onto the 3 axis of the drone
	// It is NOT Euler angles here, so no singularity
	double rollError = rotAxis.m_x * angleRad;
	double pitchError = rotAxis.m_y * angleRad;
	double yawError = rotAxis.m_z * angleRad;

	//LogManager::getInstance().serialPrint(rollError, pitchError, yawError, 0.0);



	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(qEst, qTarget);
	//LogManager::getInstance().serialPrint(qTest, qTarget);
	//LogManager::getInstance().serialPrint(dc.m_madgwickFilter.m_qEst, dc.m_targetAttitude);

	//StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
	//StateMachine::getInstance().setState(StateMachine::getInstance().getLandingState());
}


void LandingtState::handleState(const double dt, DroneController& dc)
{
	// TODO: Handle autonomous landing

	// Go back to idle state
	StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
}


