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


void StartupSequenceState::handleState(Scheduler& dc)
{
	m_time += dc.m_rateDt;

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


void IdleState::handleState(Scheduler& dc)
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


void ReadyToTakeOffState::handleState(Scheduler& dc)
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


void TakeOffState::handleState(Scheduler& dc)
{
	// TODO: Handle take off autonomously

	//LogManager::getInstance().serialPrint("TakeOffState\n\r");

	// Goto to flying state
	StateMachine::getInstance().setState(StateMachine::getInstance().getFlyingState());
}


/*
 * Handle flying
 */
void FlyingState::handleState(Scheduler& dc)
{
	//static unsigned long int pos = 0;
	//static unsigned long int angle = 0;
	//static unsigned long int rate = 0;

	// Angle loop
	if (dc.m_angleLoop)
	{
		// Correct the physical offset IMU -> drone
		dc.m_qAttitudeCorrected = dc.m_qHoverOffset * dc.m_madgwickFilter.m_qEst;
		dc.m_qAttitudeCorrected.normalize();

		// A quaternion q and -q represent the same rotation.
		// Here, canonical() make a sign choice (q.w >= 0).
		Quaternion<float> qEst = Quaternion<float>::canonical(dc.m_qAttitudeCorrected);
		Quaternion<float> qTarget = Quaternion<float>::canonical(dc.m_targetAttitude);

		// Get attitude error
		Quaternion<float> qError = PID::getError(qEst, qTarget);

		//Quaternion qTest = qError * qEst;
		//qTest.normalize();

		// Get the angle and axis of rotation
		Vector3<float> rotAxis;
		float angleRad = 0.0f;
		qError.toAxisAngle(rotAxis, angleRad);

		// Projection of the rotation axis onto the 3 axis of the drone
		// It is NOT Euler angles here, so no singularity
		float rollError = rotAxis.m_x * angleRad;
		float pitchError = rotAxis.m_y * angleRad;
		float yawError = rotAxis.m_z * angleRad;

		// Run angle PID
		dc.m_ctrlStrat.angleControlLoop(dc.m_angleDt);

		//angle++;

		//LogManager::getInstance().serialPrint(qEst, qTarget);
	}

	// Position hold loop
	if (dc.m_posLoop)
	{
		//pos++;
	}

	//rate++;
	//LogManager::getInstance().serialPrint(pos, angle, rate, 0L);

	// Run rate PID
	dc.m_ctrlStrat.rateControlLoop(dc.m_rateDt, dc.m_gyroCopy, dc.m_radio);

	dc.m_thrust = dc.m_radio.m_targetThrust * 4.0f;
	dc.m_torqueX = dc.m_ctrlStrat.m_rateLoop[0].m_output;
	dc.m_torqueY = dc.m_ctrlStrat.m_rateLoop[1].m_output;
	dc.m_torqueZ = dc.m_ctrlStrat.m_rateLoop[2].m_output;

	//LogManager::getInstance().serialPrint(dc.m_torqueX, dc.m_torqueY, dc.m_torqueZ, 0.0);

	//LogManager::getInstance().serialPrint(dc.m_averagedGyro.m_x, dc.m_averagedGyro.m_y, dc.m_averagedGyro.m_z, 0.0);
	//LogManager::getInstance().serialPrint(dc.m_radio.m_targetRateRoll, dc.m_radio.m_targetRatePitch, dc.m_radio.m_targetRateYaw, 0.0);
	//LogManager::getInstance().serialPrint("\n\r");

	//LogManager::getInstance().serialPrint("FlyingState\n\r");

	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(qEst, qTarget);
	//LogManager::getInstance().serialPrint(qTest, qTarget);


	//StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
	//StateMachine::getInstance().setState(StateMachine::getInstance().getLandingState());
}


void LandingtState::handleState(Scheduler& dc)
{
	// TODO: Handle autonomous landing

	// Go back to idle state
	StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
}


