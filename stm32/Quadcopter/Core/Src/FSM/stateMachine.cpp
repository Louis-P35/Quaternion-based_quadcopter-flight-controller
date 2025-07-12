/*
 * stateMachine.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

// Includes from project
#include "flightCore.hpp"
#include "FSM/stateMachine.hpp"
#include "logManager.hpp"
#include "PID/pid.hpp"
#include "PID/controlStrategy.hpp"


#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)


extern FlightCore* g_pFlightCore;

void StartupSequenceState::handleState(const float& dt)
{
	m_time += dt;

	// All motors at 0% power
	g_pFlightCore->m_thrust = 0.0;
	g_pFlightCore->m_torqueX = 0.0;
	g_pFlightCore->m_torqueY = 0.0;
	g_pFlightCore->m_torqueZ = 0.0;

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


void IdleState::handleState(const float& dt)
{
	// All motors at 0% power
	g_pFlightCore->m_thrust = 0.0;
	g_pFlightCore->m_torqueX = 0.0;
	g_pFlightCore->m_torqueY = 0.0;
	g_pFlightCore->m_torqueZ = 0.0;

	//LogManager::getInstance().serialPrint("IdleState\n\r");
	//LogManager::getInstance().serialPrint(g_flightCore.m_radio.m_targetThrust);

	// Wait for throttle all the way down from the controller
	if (!g_pFlightCore->m_radio.m_signalLost &&
			g_pFlightCore->m_radio.m_targetThrust < (g_pFlightCore->m_radio.m_throttleHoverOffset + 0.01))
	{
		// Goto ready to take off state
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getReadyToTakeOffState());
	}
}


void ReadyToTakeOffState::handleState(const float& dt)
{
	//LogManager::getInstance().serialPrint("ReadyToTakeOffState\n\r");
	//LogManager::getInstance().serialPrint(g_flightCore.m_radio.m_targetThrust);

	g_pFlightCore->m_thrust = g_pFlightCore->m_radio.m_targetThrust;
	g_pFlightCore->m_torqueX = 0.0;
	g_pFlightCore->m_torqueY = 0.0;
	g_pFlightCore->m_torqueZ = 0.0;

	// Wait for throttle little increase from the controller
	if (!g_pFlightCore->m_radio.m_signalLost &&
			g_pFlightCore->m_radio.m_targetThrust > (g_pFlightCore->m_radio.m_throttleHoverOffset + 0.01))
	{
		// Goto ready to flying state
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getFlyingState());
	}
}


/*
 * Handle flying
 */
void FlyingState::handleState(const float& dt)
{
	// TODO: Set setPoint must be replace by subStateMachine output

	// Angle loop
	if (g_pFlightCore->m_angleLoop)
	{
		// Set setPoint (from radio)
		if (!g_pFlightCore->m_radio.m_signalLost)
		{
			// Compute target quaternion
			g_pFlightCore->m_setPoint.m_targetQuaternion = Quaternion<float>::fromEuler(
					g_pFlightCore->m_radio.m_targetRoll * DEG_TO_RAD,
					g_pFlightCore->m_radio.m_targetPitch * DEG_TO_RAD,
					g_pFlightCore->m_radio.m_targetYaw * DEG_TO_RAD
					);
		}
		else
		{
			// TODO: Signal lost, target quaternion is horizon
		}

		// Correct the physical offset IMU -> drone
		g_pFlightCore->m_qAttitudeCorrected = g_pFlightCore->m_qHoverOffset * g_pFlightCore->m_madgwickFilter.m_qEst;
		g_pFlightCore->m_qAttitudeCorrected.normalize();

		// A quaternion q and -q represent the same rotation.
		// Here, canonical() make a sign choice (q.w >= 0).
		Quaternion<float> qEst = Quaternion<float>::canonical(g_pFlightCore->m_qAttitudeCorrected);
		Quaternion<float> qTarget = Quaternion<float>::canonical(g_pFlightCore->m_setPoint.m_targetQuaternion);

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
		g_pFlightCore->m_ctrlStrat.angleControlLoop(
				//g_flightCore.m_angleDt,
				0.01, // TODO dt !!!
				g_pFlightCore->m_imu.m_gyroFilterRates,
				error,
				g_pFlightCore->m_isFlying
				);
	}

	// Position hold loop
	if (g_pFlightCore->m_posLoop)
	{

	}

	// Set setPoint
	if (g_pFlightCore->m_ctrlStrat.m_flightMode == StabilizationMode::ACRO)
	{
		g_pFlightCore->m_setPoint.m_targetRateRoll = g_pFlightCore->m_radio.m_targetRateRoll;
		g_pFlightCore->m_setPoint.m_targetRatePitch = g_pFlightCore->m_radio.m_targetRatePitch;
		g_pFlightCore->m_setPoint.m_targetRateYaw = g_pFlightCore->m_radio.m_targetRateYaw;
	}
	else if (g_pFlightCore->m_ctrlStrat.m_flightMode == StabilizationMode::STAB)
	{
		g_pFlightCore->m_setPoint.m_targetRateRoll = g_pFlightCore->m_ctrlStrat.m_angleLoop[0].m_output;
		g_pFlightCore->m_setPoint.m_targetRatePitch = g_pFlightCore->m_ctrlStrat.m_angleLoop[1].m_output;
		g_pFlightCore->m_setPoint.m_targetRateYaw = g_pFlightCore->m_ctrlStrat.m_angleLoop[2].m_output;
	}

	// Run rate PID
	g_pFlightCore->m_ctrlStrat.rateControlLoop(
			//g_flightCore.m_rateDt,
			0.01, // TODO dt !!!
			g_pFlightCore->m_imu.m_gyroFilterRates,
			g_pFlightCore->m_setPoint
			);

	g_pFlightCore->m_thrust = g_pFlightCore->m_radio.m_targetThrust * 4.0f;
	g_pFlightCore->m_torqueX = g_pFlightCore->m_ctrlStrat.m_rateLoop[0].m_output;
	g_pFlightCore->m_torqueY = g_pFlightCore->m_ctrlStrat.m_rateLoop[1].m_output;
	g_pFlightCore->m_torqueZ = g_pFlightCore->m_ctrlStrat.m_rateLoop[2].m_output;
}


