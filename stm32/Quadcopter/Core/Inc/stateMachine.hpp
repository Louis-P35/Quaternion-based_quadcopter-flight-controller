/*
 * stateMachine.hpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

#pragma once

#include "scheduler.hpp"

class StateMachine;

/*
 * Abstract base class for the drone flying states
 */
class State
{
public:
	virtual ~State() = default;
	virtual void handleState(Scheduler& dc) = 0;
};


/*
 * In this state the drone is performing the ESCs' startup sequence.
 */
class StartupSequenceState : public State
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
class IdleState : public State
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
class ReadyToTakeOffState : public State
{
public:
	~ReadyToTakeOffState() override = default;
	virtual void handleState(Scheduler& dc) override;
};


/*
 * Flying state
 */
class FlyingState : public State
{
private:
	State* m_pSubStateMachine = nullptr;

public:
	~FlyingState() override = default;
	virtual void handleState(Scheduler& dc) override;
};




class MainStateMachine
{
private:
	// States static instances (this class is a singleton)
	StartupSequenceState m_startupSequenceState;
	IdleState m_idleState;
	ReadyToTakeOffState m_readyToTakeOffState;
	FlyingState m_flyingState;

	State* m_pState = &m_startupSequenceState;

private:
	MainStateMachine() = default;

public:
	// Delete copy constructor and copy assignment operator
	MainStateMachine(const MainStateMachine& sm) = delete;
	MainStateMachine operator=(const MainStateMachine& sm) = delete;
	// Delete move constructor and move assignment operator
	MainStateMachine(MainStateMachine&& sm) = delete;
	MainStateMachine& operator=(MainStateMachine&& sm) = delete;

	// Singleton instance getter
	static MainStateMachine& getInstance() {
		static MainStateMachine instance;
		return instance;
	};

	void setState(State& nextState)
	{
		m_pState = &nextState;
	};

	void run(Scheduler& dc)
	{
		if (m_pState)
		{
			m_pState->handleState(dc);
		}
	};

	// Getters
	StartupSequenceState& getStartupSequenceState() {return m_startupSequenceState;};
	IdleState& getIdleState() {return m_idleState;};
	ReadyToTakeOffState& getReadyToTakeOffState() {return m_readyToTakeOffState;};
	FlyingState& getFlyingState() {return m_flyingState;};
};




