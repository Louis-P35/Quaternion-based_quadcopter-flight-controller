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


/*
 * Handle startup sequence state.
 */
void StartupSequenceState::enterState()
{
	// All motors at 0% power
	g_pFlightCore->m_thrust = 0.0f;
	g_pFlightCore->m_torqueX = 0.0f;
	g_pFlightCore->m_torqueY = 0.0f;
	g_pFlightCore->m_torqueZ = 0.0f;

	// Reset time
	m_time = 0.0f;
}
void StartupSequenceState::handleState(const float& dt)
{
	m_time += dt;

	LogManager::getInstance().serialPrint("StartupSequenceState\n\r");
	//LogManager::getInstance().serialPrint(m_time);

	// Wait 2s
	if (m_time > 2.0f)
	{
		// Goto idle state
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getIdleState());
	}
}
void StartupSequenceState::exitState()
{

}


/*
 * Handle idle state.
 */
void IdleState::enterState()
{
	// All motors at 0% power
	g_pFlightCore->m_thrust = 0.0f;
	g_pFlightCore->m_torqueX = 0.0f;
	g_pFlightCore->m_torqueY = 0.0f;
	g_pFlightCore->m_torqueZ = 0.0f;
}
void IdleState::handleState(const float& dt)
{
	LogManager::getInstance().serialPrint("IdleState\n\r");
	//LogManager::getInstance().serialPrint(g_flightCore.m_radio.m_targetThrust);

	// Wait for throttle all the way down from the controller
	if (!g_pFlightCore->m_radio.m_signalLost &&
			g_pFlightCore->m_radio.m_targetThrust < (g_pFlightCore->m_radio.m_throttleHoverOffset + 0.01f))
	{
		// Goto ready to take off state
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getReadyToTakeOffState());
	}
}
void IdleState::exitState()
{

}



/*
 * Handle ready to take off state.
 */
void ReadyToTakeOffState::enterState()
{
	g_pFlightCore->m_torqueX = 0.0f;
	g_pFlightCore->m_torqueY = 0.0f;
	g_pFlightCore->m_torqueZ = 0.0f;
}
void ReadyToTakeOffState::handleState(const float& dt)
{
	LogManager::getInstance().serialPrint("ReadyToTakeOffState\n\r");
	//LogManager::getInstance().serialPrint(g_flightCore.m_radio.m_targetThrust);

	g_pFlightCore->m_thrust = g_pFlightCore->m_radio.m_targetThrust;

	// Wait for throttle little increase from the controller
	if (!g_pFlightCore->m_radio.m_signalLost &&
			g_pFlightCore->m_radio.m_targetThrust > (g_pFlightCore->m_radio.m_throttleHoverOffset + 0.01f))
	{
		// Goto ready to flying state
		MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getFlyingState());
	}
}
void ReadyToTakeOffState::exitState()
{

}


/*
 * Handle flying
 */
void FlyingState::enterState()
{
	g_pFlightCore->m_rateLoopEnable = true;
}
void FlyingState::handleState(const float& dt)
{
	// TODO: Set setPoint must be replace by subStateMachine output

	// Compute setpoints for the angle loop
	if (g_pFlightCore->m_angleLoopEnable)
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
	}

	// Compute setpoints for the position hold loop
	if (g_pFlightCore->m_posLoopEnable)
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
}
void FlyingState::exitState()
{
	g_pFlightCore->m_rateLoopEnable = false;
}


