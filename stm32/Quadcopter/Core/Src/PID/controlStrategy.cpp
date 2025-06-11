/*
 * controlStrategy.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

// Includes from project
#include "PID/controlStrategy.hpp"


/*
 *
 * return the rotation axis. It's norm is the rotation magnitude
 */
void ControlStrategy::rateControlLoop(const float& dt, const Vector3<float>& gyro, const SetPoint<float>& setPoint)
{
	m_rateLoop[0].m_measure = gyro.m_x;
	m_rateLoop[1].m_measure = gyro.m_y;
	m_rateLoop[2].m_measure = gyro.m_z;

	// Set the rates target according to the setPoint
	m_rateLoop[0].m_target = setPoint.m_targetRateRoll;
	m_rateLoop[1].m_target = setPoint.m_targetRatePitch;
	m_rateLoop[2].m_target = setPoint.m_targetRateYaw;

	for (int i = 0; i < 3; ++i)
	{
		const float error = m_rateLoop[i].m_target - m_rateLoop[i].m_measure;
		m_rateLoop[i].m_output = m_rateLoop[i].computePID(error, m_rateLoop[i].m_measure, m_rateLoop[i].m_target, dt, true);
	}
}

void ControlStrategy::angleControlLoop(
		const float& dt,
		const Vector3<float>& gyro,
		const std::array<float, 3>& error,
		const bool& integrate
		)
{
	m_rateLoop[0].m_measure = gyro.m_x;
	m_rateLoop[1].m_measure = gyro.m_y;
	m_rateLoop[2].m_measure = gyro.m_z;

	for (int i = 0; i < 3; ++i)
	{
		// measure = 0.0f because the derivative is on the error here
		// target = 0.0f because no feedforward here
		float pidOut = m_angleLoop[i].computePID(error[i], 0.0f, 0.0f, dt, integrate);
		m_angleLoop[i].m_output = m_angleLoop[i].m_filteredOutput.apply(pidOut);
	}
}

void ControlStrategy::posControlLoop(const float& dt)
{
	m_posLoop[0].m_measure = 0.0f; // TODO: px
	m_posLoop[1].m_measure = 0.0f; // TODO: py
	m_posLoop[2].m_measure = 0.0f; // TODO: pz

	m_posLoop[0].m_target = 0.0f;
	m_posLoop[1].m_target = 0.0f;
	m_posLoop[2].m_target = 0.0f;

	for (int i = 0; i < 3; ++i)
	{
		//m_posLoop[i].run(dt);
		//m_angleLoop[i].m_target = m_posLoop[i].m_output; // Chain
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

void ControlStrategy::setPIDsatMinMaxRate(const float& sat, const float& minOut, const float& maxOut, const float maxDpercent)
{
	for (size_t i = 0; i < 3; ++i)
	{
		m_rateLoop[i].setBoundcoeffs(sat, minOut, maxOut, maxDpercent);
	}
}

void ControlStrategy::setPIDsatMinMaxAngle(const float& sat, const float& minOut, const float& maxOut, const float maxDpercent)
{
	for (size_t i = 0; i < 3; ++i)
	{
		m_angleLoop[i].setBoundcoeffs(sat, minOut, maxOut, maxDpercent);
	}
}

void ControlStrategy::setPIDsatMinMaxPos(const float& sat, const float& minOut, const float& maxOut, const float maxDpercent)
{
	for (size_t i = 0; i < 3; ++i)
	{
		m_posLoop[i].setBoundcoeffs(sat, minOut, maxOut, maxDpercent);
	}
}

void ControlStrategy::setRatePIDcoefsRoll(const float& kp, const float& ki, const float& kd)
{
	m_rateLoop[0].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setRatePIDcoefsPitch(const float& kp, const float& ki, const float& kd)
{
	m_rateLoop[1].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setRatePIDcoefsYaw(const float& kp, const float& ki, const float& kd)
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


void ControlStrategy::setAnglePIDcoefsRoll(const float& kp, const float& ki, const float& kd)
{
	m_angleLoop[0].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setAnglePIDcoefsPitch(const float& kp, const float& ki, const float& kd)
{
	m_angleLoop[1].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setAnglePIDcoefsYaw(const float& kp, const float& ki, const float& kd)
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


void ControlStrategy::setPosPIDcoefsRoll(const float& kp, const float& ki, const float& kd)
{
	m_posLoop[0].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setPosPIDcoefsPitch(const float& kp, const float& ki, const float& kd)
{
	m_posLoop[1].setPIDcoeffs(kp, ki, kd);
}


void ControlStrategy::setPosPIDcoefsYaw(const float& kp, const float& ki, const float& kd)
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

