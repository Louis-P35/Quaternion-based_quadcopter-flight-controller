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
 * This state handle the take off autonomously
 */
class TakeOffState : public State
{
public:
	~TakeOffState() override = default;
	virtual void handleState(Scheduler& dc) override;
};


/*
 * This state handle the "take off / recovery" during free fall detection
 */
class TakeOffOnFreeFallState : public State
{
public:
	~TakeOffOnFreeFallState() override = default;
	virtual void handleState(Scheduler& dc) override;
};


/*
 * Flying state
 */
class FlyingState : public State
{
public:
	~FlyingState() override = default;
	virtual void handleState(Scheduler& dc) override;
};


/*
 * This state handle the landing autonomousely
 */
class LandingtState : public State
{
public:
	~LandingtState() override = default;
	virtual void handleState(Scheduler& dc) override;
};



class StateMachine
{
private:
	// States static instances (this class is a singleton)
	StartupSequenceState m_startupSequenceState;
	IdleState m_idleState;
	ReadyToTakeOffState m_readyToTakeOffState;
	TakeOffState m_takeOffState;
	FlyingState m_flyingState;
	LandingtState m_landingState;

	State* m_pState = &m_startupSequenceState;

private:
	StateMachine() = default;

public:
	// Delete copy constructor and copy assignment operator
	StateMachine(const StateMachine& sm) = delete;
	StateMachine operator=(const StateMachine& sm) = delete;
	// Delete move constructor and move assignment operator
	StateMachine(StateMachine&& sm) = delete;
	StateMachine& operator=(StateMachine&& sm) = delete;

	// Singleton instance getter
	static StateMachine& getInstance() {
		static StateMachine instance;
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
	TakeOffState& getTakeOffState() {return m_takeOffState;};
	FlyingState& getFlyingState() {return m_flyingState;};
	LandingtState& getLandingState() {return m_landingState;};
};


