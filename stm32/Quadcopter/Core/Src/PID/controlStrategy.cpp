/*
 * controlStrategy.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

#include "PID/controlStrategy.hpp"


/*
 * Set the angles target according to the flight mode
 */
void ControlStrategy::setAngleTarget()
{
	switch (m_flightMode)
	{
	case StabilizationMode::STAB:
		m_angleLoop[0].m_target = 0.0; // Radio roll
		m_angleLoop[1].m_target = 0.0; // Radio pitch
		m_angleLoop[2].m_target = 0.0; // Radio yaw
		break;

	case StabilizationMode::ACRO:
		// None
		break;

	case StabilizationMode::HORIZON:
		// TODO
		break;

	case StabilizationMode::POSHOLD:
		m_angleLoop[0].m_target = m_posLoop[0].m_output;
		m_angleLoop[1].m_target = m_posLoop[1].m_output;
		m_angleLoop[2].m_target = m_posLoop[2].m_output;
		break;

	default:
		break;
	}
}

/*
 * Set the rates target according to the flight mode
 */
void ControlStrategy::setRateTarget()
{
	switch (m_flightMode)
	{
	case StabilizationMode::STAB:
	case StabilizationMode::POSHOLD:
		m_rateLoop[0].m_target = m_angleLoop[0].m_output;
		m_rateLoop[1].m_target = m_angleLoop[1].m_output;
		m_rateLoop[2].m_target = m_angleLoop[2].m_output;
		break;

	case StabilizationMode::ACRO:
		m_rateLoop[0].m_target = 0.0; // Radio
		m_rateLoop[1].m_target = 0.0; // Radio
		m_rateLoop[2].m_target = 0.0; // Radio
		break;

	case StabilizationMode::HORIZON:
		// TODO
		break;

	default:
		break;
	}
}


/*
 *
 * runPosLoop: Run the position hold loop each time it is true
 * runAngleLoop: Run the attitude hold loop each time it is true
 *
 * return the rotation axis. It's norm is the rotation magnitude
 */
void ControlStrategy::rateControlLoop(const double dt)
{
	m_rateLoop[0].m_measure = 0.0; // TODO: From gyro
	m_rateLoop[1].m_measure = 0.0; // TODO: From gyro
	m_rateLoop[2].m_measure = 0.0; // TODO: From gyro

	// Set the rates target according to the flight mode
	setRateTarget();

	for (int i = 0; i < 3; ++i)
	{
		m_rateLoop[i].run(dt);
	}
}

void ControlStrategy::angleControlLoop(const double dt)
{
	m_rateLoop[0].m_measure = 0.0; // TODO: From gyro
	m_rateLoop[1].m_measure = 0.0; // TODO: From gyro
	m_rateLoop[2].m_measure = 0.0; // TODO: From gyro

	// Set the angles target according to the flight mode
	setAngleTarget();

	for (int i = 0; i < 3; ++i)
	{
		m_rateLoop[i].run(dt);
	}
}

void ControlStrategy::posControlLoop(const double dt)
{
	m_posLoop[0].m_measure = 0.0; // TODO: px
	m_posLoop[1].m_measure = 0.0; // TODO: py
	m_posLoop[2].m_measure = 0.0; // TODO: pz

	m_posLoop[0].m_target = 0.0;
	m_posLoop[1].m_target = 0.0;
	m_posLoop[2].m_target = 0.0;

	for (int i = 0; i < 3; ++i)
	{
		m_posLoop[i].run(dt);
		m_angleLoop[i].m_target = m_posLoop[i].m_output; // Chain
	}
}

/*
 * Return the torque vector.
 * It is the rotation torque (around the rotation axis) computed by the last PID of the chain
 */
void ControlStrategy::getTorqueVector(double& tx, double& ty, double& tz)
{
	tx = m_angleLoop[0].m_output;
	ty = m_angleLoop[1].m_output;
	tz = m_angleLoop[2].m_output;
}
