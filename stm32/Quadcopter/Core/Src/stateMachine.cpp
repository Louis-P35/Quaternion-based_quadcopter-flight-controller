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
#include "PID/controlStrategy.hpp"


void StartupSequenceState::handleState(const double dt, Scheduler& dc)
{
	m_time += dt;

	// All motors at 0% power
	dc.m_thrust = 0.0;
	dc.m_torqueX = 0.0;
	dc.m_torqueY = 0.0;
	dc.m_torqueZ = 0.0;

	//LogManager::getInstance().serialPrint("StartupSequenceState\n\r");
	//LogManager::getInstance().serialPrint(m_time);

	// Wait 2s
	if (m_time > 2.0)
	{
		m_time = 0.0; // Reset time

		// Goto idle state
		StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
	}
}


void IdleState::handleState(const double dt, Scheduler& dc)
{
	// All motors at 0% power
	dc.m_thrust = 0.0;
	dc.m_torqueX = 0.0;
	dc.m_torqueY = 0.0;
	dc.m_torqueZ = 0.0;

	//LogManager::getInstance().serialPrint("IdleState\n\r");
	//LogManager::getInstance().serialPrint(dc.m_radio.m_targetThrust);

	// Wait for throttle all the way down from the controller
	if (!dc.m_radio.m_signalLost && dc.m_radio.m_targetThrust < (dc.m_radio.m_throttleHoverOffset + 0.01))
	{
		// Goto ready to take off state
		StateMachine::getInstance().setState(StateMachine::getInstance().getReadyToTakeOffState());
	}
}


void ReadyToTakeOffState::handleState(const double dt, Scheduler& dc)
{
	//LogManager::getInstance().serialPrint("ReadyToTakeOffState\n\r");
	//LogManager::getInstance().serialPrint(dc.m_radio.m_targetThrust);

	dc.m_thrust = dc.m_radio.m_targetThrust;
	dc.m_torqueX = 0.0;
	dc.m_torqueY = 0.0;
	dc.m_torqueZ = 0.0;

	// Wait for throttle little increase from the controller
	if (!dc.m_radio.m_signalLost && dc.m_radio.m_targetThrust > (dc.m_radio.m_throttleHoverOffset + 0.01))
	{
		// Goto ready to flying state
		StateMachine::getInstance().setState(StateMachine::getInstance().getTakeOffState());
	}
}


void TakeOffState::handleState(const double dt, Scheduler& dc)
{
	// TODO: Handle take off autonomously

	//LogManager::getInstance().serialPrint("TakeOffState\n\r");

	// Goto to flying state
	StateMachine::getInstance().setState(StateMachine::getInstance().getFlyingState());
}


/*
 * Handle flying
 */
void FlyingState::handleState(const double dt, Scheduler& dc)
{
	//static unsigned long int pos = 0;
	//static unsigned long int angle = 0;
	//static unsigned long int rate = 0;

	// Angle loop
	if (dc.m_angleLoop)
	{
		Quaternion qEst = Quaternion::canonical(dc.m_madgwickFilter.m_qEst);
		Quaternion qTarget = Quaternion::canonical(dc.m_targetAttitude);

		// Get attitude error
		Quaternion qError = PID::getError(qEst, qTarget);

		//Quaternion qTest = qError * qEst;
		//qTest.normalize();

		// Get the angle and axis of rotation
		Vector3<double> rotAxis;
		double angleRad = 0.0;
		qError.toAxisAngle(rotAxis, angleRad);

		// Projection of the rotation axis onto the 3 axis of the drone
		// It is NOT Euler angles here, so no singularity
		double rollError = rotAxis.m_x * angleRad;
		double pitchError = rotAxis.m_y * angleRad;
		double yawError = rotAxis.m_z * angleRad;

		// Run angle PID
		dc.m_ctrlStrat.angleControlLoop(dc.m_pidAngleLoopDt);

		//angle++;
	}

	// Position hold loop
	if (dc.m_posLoop)
	{
		//pos++;
	}

	//rate++;
	//LogManager::getInstance().serialPrint(pos, angle, rate, 0L);


	// Run rate PID
	dc.m_ctrlStrat.rateControlLoop(dt);

	dc.m_thrust = dc.m_radio.m_targetThrust * 4.0;
	dc.m_torqueX = 0.0;
	dc.m_torqueY = 0.0;
	dc.m_torqueZ = 0.0;

	//LogManager::getInstance().serialPrint(rollError, pitchError, yawError, 0.0);

	//LogManager::getInstance().serialPrint("FlyingState\n\r");

	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(qEst, qTarget);
	//LogManager::getInstance().serialPrint(qTest, qTarget);
	//LogManager::getInstance().serialPrint(dc.m_madgwickFilter.m_qEst, dc.m_targetAttitude);

	//StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
	//StateMachine::getInstance().setState(StateMachine::getInstance().getLandingState());
}


void LandingtState::handleState(const double dt, Scheduler& dc)
{
	// TODO: Handle autonomous landing

	// Go back to idle state
	StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
}


