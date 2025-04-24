/*
 * stateMachine.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

#include "stateMachine.hpp"


void IdleState::handleState()
{
	// TODO: Wait for throttle all the way down from the controller

	// Goto ready to take off state
	StateMachine::getInstance().setState(StateMachine::getInstance().getReadyToTakeOffState());
}


void ReadyToTakeOffState::handleState()
{
	// TODO: Wait for throttle little increase from the controller

	// Goto ready to flying state
	StateMachine::getInstance().setState(StateMachine::getInstance().getTakeOffState());
}


void TakeOffState::handleState()
{
	// TODO: Handle take off autonomously

	// Goto to flying state
	StateMachine::getInstance().setState(StateMachine::getInstance().getFlyingState());
}


void FlyingState::handleState()
{
	// TODO: Handle flying

	//StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
	//StateMachine::getInstance().setState(StateMachine::getInstance().getLandingState());
}


void LandingtState::handleState()
{
	// TODO: Handle autonomous landing

	// Go back to idle state
	StateMachine::getInstance().setState(StateMachine::getInstance().getIdleState());
}


