/*
 * stateMachine.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

// Includes from project
#include "FSM/stateMachine.hpp"
#include "logManager.hpp"
#include "PID/pid.hpp"
#include "PID/controlStrategy.hpp"


#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)


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
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getIdleState());
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
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getReadyToTakeOffState());
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
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getFlyingState());
	}
}


/*
 * Handle flying
 */
void FlyingState::handleState(Scheduler& dc)
{
	// TODO: Set setPoint must be replace by subStateMachine output

	// Angle loop
	if (dc.m_angleLoop)
	{
		// Set setPoint (from radio)
		if (!dc.m_radio.m_signalLost)
		{
			// Compute target quaternion
			dc.m_setPoint.m_targetQuaternion = Quaternion<float>::fromEuler(dc.m_radio.m_targetRoll * DEG_TO_RAD, dc.m_radio.m_targetPitch * DEG_TO_RAD, dc.m_radio.m_targetYaw * DEG_TO_RAD);
		}
		else
		{
			// TODO: Signal lost, target quaternion is horizon
		}

		// Correct the physical offset IMU -> drone
		dc.m_qAttitudeCorrected = dc.m_qHoverOffset * dc.m_madgwickFilter.m_qEst;
		dc.m_qAttitudeCorrected.normalize();

		// A quaternion q and -q represent the same rotation.
		// Here, canonical() make a sign choice (q.w >= 0).
		Quaternion<float> qEst = Quaternion<float>::canonical(dc.m_qAttitudeCorrected);
		Quaternion<float> qTarget = Quaternion<float>::canonical(dc.m_setPoint.m_targetQuaternion);

		// Get attitude error
		Quaternion<float> qError = PID::getError(qEst, qTarget);

		// Test
		//Quaternion qTest = qError * qEst;
		//qTest.normalize();

		// Get the angle and axis of rotation
		Vector3<float> rotAxis;
		float angleRad = 0.0f;
		qError.toAxisAngle(rotAxis, angleRad);

		// Projection of the rotation axis onto the 3 axis of the drone
		// It is NOT Euler angles here, so no singularity
		std::array<float, 3> error;
		error[0] = rotAxis.m_x * angleRad * RAD_TO_DEG;
		error[1] = rotAxis.m_y * angleRad * RAD_TO_DEG;
		error[2] = rotAxis.m_z * angleRad * RAD_TO_DEG;

		// Run angle PID
		dc.m_ctrlStrat.angleControlLoop(dc.m_angleDt, dc.m_imu.m_gyroFilterRates, error, dc.m_isFlying);
	}

	// Position hold loop
	if (dc.m_posLoop)
	{

	}

	// Set setPoint
	if (dc.m_ctrlStrat.m_flightMode == StabilizationMode::ACRO)
	{
		dc.m_setPoint.m_targetRateRoll = dc.m_radio.m_targetRateRoll;
		dc.m_setPoint.m_targetRatePitch = dc.m_radio.m_targetRatePitch;
		dc.m_setPoint.m_targetRateYaw = dc.m_radio.m_targetRateYaw;
	}
	else if (dc.m_ctrlStrat.m_flightMode == StabilizationMode::STAB)
	{
		dc.m_setPoint.m_targetRateRoll = dc.m_ctrlStrat.m_angleLoop[0].m_output;
		dc.m_setPoint.m_targetRatePitch = dc.m_ctrlStrat.m_angleLoop[1].m_output;
		dc.m_setPoint.m_targetRateYaw = dc.m_ctrlStrat.m_angleLoop[2].m_output;
	}

	// Run rate PID
	dc.m_ctrlStrat.rateControlLoop(dc.m_rateDt, dc.m_imu.m_gyroFilterRates, dc.m_setPoint);

	dc.m_thrust = dc.m_radio.m_targetThrust * 4.0f;
	dc.m_torqueX = dc.m_ctrlStrat.m_rateLoop[0].m_output;
	dc.m_torqueY = dc.m_ctrlStrat.m_rateLoop[1].m_output;
	dc.m_torqueZ = dc.m_ctrlStrat.m_rateLoop[2].m_output;
}


