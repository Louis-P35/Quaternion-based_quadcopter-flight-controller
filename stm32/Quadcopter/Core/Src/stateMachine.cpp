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


void IdleState::handleState(const double dt, DroneController& dc)
{
	// TODO: Wait for throttle all the way down from the controller

	// Goto ready to take off state
	StateMachine::getInstance().setState(StateMachine::getInstance().getReadyToTakeOffState());
}


void ReadyToTakeOffState::handleState(const double dt, DroneController& dc)
{
	// TODO: Wait for throttle little increase from the controller

	// Goto ready to flying state
	StateMachine::getInstance().setState(StateMachine::getInstance().getTakeOffState());
}


void TakeOffState::handleState(const double dt, DroneController& dc)
{
	// TODO: Handle take off autonomously

	// Goto to flying state
	StateMachine::getInstance().setState(StateMachine::getInstance().getFlyingState());
}


void FlyingState::handleState(const double dt, DroneController& dc)
{
	// TODO: Handle flying

	Quaternion qEst = Quaternion::canonical(dc.m_madgwickFilter.m_qEst);
	Quaternion qTarget = Quaternion::canonical(dc.m_targetAttitude);

	// Get attitude error
	Quaternion qError = PID::getError(qEst, qTarget);

	Quaternion qTest = qError * qEst;
	qTest.normalize();

	/*double angle = 2.0 * acos(fabs(qTest.dotProduct(qTarget)));
	if (angle < 0.01)
	{
	    LogManager::getInstance().serialPrint("✓ Quaternions math OK\n\r");
	}
	else
	{
	    LogManager::getInstance().serialPrint("✗ Quaternion mismatch\n\r");
	    LogManager::getInstance().serialPrint(angle);
	    LogManager::getInstance().serialPrint(qEst);
	    LogManager::getInstance().serialPrint(qEstInv);
	    LogManager::getInstance().serialPrint(qTarget);
	    LogManager::getInstance().serialPrint(qError);
	    LogManager::getInstance().serialPrint(qTest);
	}*/

	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(qEst, qTarget);
	LogManager::getInstance().serialPrint(qTest, qTarget);
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


