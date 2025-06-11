/*
 * stateMachine.hpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

#pragma once

#include "scheduler.hpp"


/*
 * Abstract base class for FSM states
 */
class FSMState
{
public:
	virtual ~FSMState() = default;
	virtual void handleState(Scheduler& dc) = 0;
};




class StateMachineBase
{
protected:
	FSMState* m_pState = nullptr;

public:
	StateMachineBase() = default;
	virtual ~StateMachineBase() = default;

	void setState(FSMState& nextState) noexcept
	{
		m_pState = &nextState;
	};

	void run(Scheduler& dc) noexcept
	{
		if (m_pState)
		{
			m_pState->handleState(dc);
		}
	};
};



class FullRadioControlState : public FSMState
{
public:
	~FullRadioControlState() override = default;
	virtual void handleState(Scheduler& dc) override;
};



class SetPointFSM : public StateMachineBase
{
public:
	SetPointFSM() = default;
};




/*
 * In this state the drone is performing the ESCs' startup sequence.
 */
class StartupSequenceState : public FSMState
{
private:
	double m_time = 0.0;

public:
	~StartupSequenceState() override = default;
	virtual void handleState(Scheduler& dc) override;
};




/*
 * In this state the drone is in safe mode, propellers are not spinning.
 */
class IdleState : public FSMState
{
public:
	~IdleState() override = default;
	virtual void handleState(Scheduler& dc) override;
};




/*
 * In this state the drone is ready to take off,
 * Security is off, propellers are spinning.
 * PID's integral term is blocked to zero until flying.
 */
class ReadyToTakeOffState : public FSMState
{
public:
	~ReadyToTakeOffState() override = default;
	virtual void handleState(Scheduler& dc) override;
};



/*
 * Flying state
 */
class FlyingState : public FSMState
{
private:
	SetPointFSM m_setPointStateMachine;

public:
	~FlyingState() override = default;
	virtual void handleState(Scheduler& dc) override;
};



class MainStateMachine : public StateMachineBase
{
private:
	// States static instances (this class is a singleton)
	StartupSequenceState m_startupSequenceState;
	IdleState m_idleState;
	ReadyToTakeOffState m_readyToTakeOffState;
	FlyingState m_flyingState;

public:
	MainStateMachine()
	{
		m_pState = &m_startupSequenceState;
	};

public:
	// Delete copy constructor and copy assignment operator
	MainStateMachine(const MainStateMachine& sm) = delete;
	MainStateMachine& operator=(const MainStateMachine& sm) = delete;
	// Delete move constructor and move assignment operator
	MainStateMachine(MainStateMachine&& sm) = delete;
	MainStateMachine& operator=(MainStateMachine&& sm) = delete;

	// Singleton instance getter
	static MainStateMachine& getInstance() {
		static MainStateMachine instance;
		return instance;
	};

	// Getters
	StartupSequenceState& getStartupSequenceState() {return m_startupSequenceState;};
	IdleState& getIdleState() {return m_idleState;};
	ReadyToTakeOffState& getReadyToTakeOffState() {return m_readyToTakeOffState;};
	FlyingState& getFlyingState() {return m_flyingState;};
};




