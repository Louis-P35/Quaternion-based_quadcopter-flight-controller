/*
 * controlStrategy.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

#include "PID/controlStrategy.hpp"
#include "radio.hpp"


/*
 * Set the angles target according to the flight mode
 */
void ControlStrategy::setAngleTarget()
{
	switch (m_flightMode)
	{
	case StabilizationMode::STAB:
		m_angleLoop[0].m_target = 0.0f; // Radio roll
		m_angleLoop[1].m_target = 0.0f; // Radio pitch
		m_angleLoop[2].m_target = 0.0f; // Radio yaw
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
void ControlStrategy::setRateTarget(Radio& radio)
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
		m_rateLoop[0].m_target = radio.m_targetRateRoll;	// Radio
		m_rateLoop[1].m_target = radio.m_targetRatePitch;	// Radio
		m_rateLoop[2].m_target = radio.m_targetRateYaw;		// Radio
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
void ControlStrategy::rateControlLoop(const float dt, const Vector3<float>& gyro, Radio& radio)
{
	m_rateLoop[0].m_measure = gyro.m_x;
	m_rateLoop[1].m_measure = gyro.m_y;
	m_rateLoop[2].m_measure = gyro.m_z;

	// Set the rates target according to the flight mode
	setRateTarget(radio);

	for (int i = 0; i < 3; ++i)
	{
		m_rateLoop[i].run(dt);
	}
}

void ControlStrategy::angleControlLoop(const float dt)
{
	m_rateLoop[0].m_measure = 0.0f; // TODO: From gyro
	m_rateLoop[1].m_measure = 0.0f; // TODO: From gyro
	m_rateLoop[2].m_measure = 0.0f; // TODO: From gyro

	// Set the angles target according to the flight mode
	setAngleTarget();

	for (int i = 0; i < 3; ++i)
	{
		// TODO: fix error calculation
		//m_angleLoop[i].run(dt);
	}
}

void ControlStrategy::posControlLoop(const float dt)
{
	m_posLoop[0].m_measure = 0.0f; // TODO: px
	m_posLoop[1].m_measure = 0.0f; // TODO: py
	m_posLoop[2].m_measure = 0.0f; // TODO: pz

	m_posLoop[0].m_target = 0.0f;
	m_posLoop[1].m_target = 0.0f;
	m_posLoop[2].m_target = 0.0f;

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
void ControlStrategy::getTorqueVector(float& tx, float& ty, float& tz)
{
	tx = m_angleLoop[0].m_output;
	ty = m_angleLoop[1].m_output;
	tz = m_angleLoop[2].m_output;
}





/*
 * PIDs coeff setters
 */

void ControlStrategy::setPIDsatMinMax(const float sat, const float minOut, const float maxOut)
{
	for (size_t i = 0; i < 3; ++i)
	{
		m_rateLoop[i].setBoundcoeffs(sat, minOut, maxOut);
		m_angleLoop[i].setBoundcoeffs(sat, minOut, maxOut);
		m_posLoop[i].setBoundcoeffs(sat, minOut, maxOut);
	}
}


void ControlStrategy::setRatePIDcoefsRoll(const float kp, const float ki, const float kd)
{
	m_rateLoop[0].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setRatePIDcoefsPitch(const float kp, const float ki, const float kd)
{
	m_rateLoop[1].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setRatePIDcoefsYaw(const float kp, const float ki, const float kd)
{
	m_rateLoop[2].setPIDcoeffs(kp, ki, kd);
}

void ControlStrategy::setRatePIDderivativeMode(const DerivativeMode& derivativeMode)
{
	for (size_t i = 0; i < 3; ++i)
	{
		m_rateLoop[i].setDerivativeMode(derivativeMode);
	}
}


void ControlStrategy::setAnglePIDcoefsRoll(const float kp, const float ki, const float kd)
{
	m_angleLoop[0].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setAnglePIDcoefsPitch(const float kp, const float ki, const float kd)
{
	m_angleLoop[1].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setAnglePIDcoefsYaw(const float kp, const float ki, const float kd)
{
	m_angleLoop[2].setPIDcoeffs(kp, ki, kd);
}

void ControlStrategy::setAnglePIDderivativeMode(const DerivativeMode& derivativeMode)
{
	for (size_t i = 0; i < 3; ++i)
	{
		m_angleLoop[i].setDerivativeMode(derivativeMode);
	}
}


void ControlStrategy::setPosPIDcoefsRoll(const float kp, const float ki, const float kd)
{
	m_posLoop[0].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setPosPIDcoefsPitch(const float kp, const float ki, const float kd)
{
	m_posLoop[1].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setPosPIDcoefsYaw(const float kp, const float ki, const float kd)
{
	m_posLoop[2].setPIDcoeffs(kp, ki, kd);
}

void ControlStrategy::setPosPIDderivativeMode(const DerivativeMode& derivativeMode)
{
	for (size_t i = 0; i < 3; ++i)
	{
		m_posLoop[i].setDerivativeMode(derivativeMode);
	}
}

