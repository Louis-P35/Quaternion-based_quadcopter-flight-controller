/*
 * pid.cpp
 *
 *  Created on: Jun 17, 2024
 *      Author: Louis
 */

// Project
#include "PID/pid.hpp"


/*
 * Compute error
 * For Euler-based PID
 */
float PID::getError(const float& current, const float& target)
{
	return target - current;
}

/*
 * Compute error
 * For Quaternion-based PID
 */
Quaternion<float> PID::getError(const Quaternion<float>& current, const Quaternion<float>& target)
{
	/*
	Computes the rotation necessary to go from the current orientation to the target orientation.
	This operation is from the perspective of the current orientation being the reference frame, and
	it calculates the relative rotation needed to align exactly with the target.
	*/
	Quaternion<float> qError = target * current.inverse();

	// Ensure it is normalized
	qError.normalize();

	// q and -q represent the same rotation
	// But q1 * q2 != (-q1) * q2

	/*
	A quaternion q and its negative -q represent the same orientation. This can result in a sudden
	flip in the error direction when the quaternion passes through these equivalent representations.
	If the dot product is negative, it implies that the quaternion error might cross the antipodal
	point, leading to a sign reversal. If so, we adjust the error quaternion by negating it to ensure
	it represents the shortest path.
	*/
	/*if (qError.m_w < 0.0)//current.dotProduct(target) < 0.0)
	{
		return Quaternion(
				-qError.m_w,
				-qError.m_x,
				-qError.m_y,
				-qError.m_z
				);
	}*/

	return qError;
}


/*
 * PID control.
 * Compute the response to an error.
 */
float PID::computePID(
		const float& error,
		const float& measure,
		const float& target,
		const float& dt,
		const bool& integrate
		)
{
	// Feed forward term
	if (m_kff > 0.0f) // Do not compute it if the gain is 0
	{
		// Feed forward with low pass filter
		m_ffTerm = m_ffTermLpf.apply(target * m_kff);
	}
	else
	{
		m_ffTerm = 0.0f;
	}

	// Proportional term
	m_pTerm = error * m_kp;

	// Derivative term
	const float frequency = 1.0f / dt;
	const float dMax = m_kd * frequency * m_maxDpercent;
	if (m_derivativeMode == DerivativeMode::OnError)
	{
		// Compute derivative on error
		const float derivative = (error - m_previousError) * frequency * m_kd;
		// Apply cascaded low pass filters
		m_dTerm = m_dTermLpf.apply(m_dTermLpf2.apply(derivative));
		m_previousError = error;
	}
	else if (m_derivativeMode == DerivativeMode::OnMeasurement)
	{
		// Compute derivative on measure
		const float derivative = -(measure - m_previousMeasure) * frequency * m_kd;
		// Apply cascaded low pass filters
		m_dTerm = m_dTermLpf.apply(m_dTermLpf2.apply(derivative));
		m_previousMeasure = measure;
	}
	else
	{
		m_dTerm = 0.0f;
	}
	// Cap D term
	if (m_dTerm > dMax)
	{
		m_dTerm = dMax;
	}
	else if (m_dTerm < -dMax)
	{
		m_dTerm = -dMax;
	}

	// Integral term
	if (integrate) // Do not integrate error if the drone is not flying
	{
		m_sommeError += error * dt;

		// Prevent windup
		if (m_sommeError > m_saturation)
		{
			m_sommeError = m_saturation;
		}
		else if (m_sommeError < -m_saturation)
		{
			m_sommeError = -m_saturation;
		}
	}

	m_iTerm = m_sommeError * m_ki;

	return m_pTerm + m_iTerm + m_dTerm + m_ffTerm;
}



/*void PIDBlock::run(const float& dt)
{
	const float error = m_target - m_measure;
	m_output = computePID(error, m_measure, m_heavilyFilteredMeasure, dt, true);
}*/




