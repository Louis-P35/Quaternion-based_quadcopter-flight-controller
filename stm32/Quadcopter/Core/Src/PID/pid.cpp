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
float PID::computePID(const float& error, const float& measure, const float& dt, const bool& integrate)
{
	float d = 0.0f;

	// Proportional gain
	const float p = error * m_kp;

	// Derivative gain
	const float frequency = 1.0f / dt;
	const float dMax = m_kd * frequency * m_maxDpercent;
	if (m_derivativeMode == DerivativeMode::OnError)
	{
		const float derivative = (error - m_previousError) * frequency;
		d = derivative * m_kd;
		m_previousError = error;
	}
	else if (m_derivativeMode == DerivativeMode::OnMeasurement)
	{
		const float derivative = (measure - m_previousMeasure) * frequency;
		d = derivative * m_kd;
		m_previousMeasure = measure;
	}
	if (d > dMax)
	{
		d = dMax;
	}
	else if (d < -dMax)
	{
		d = -dMax;
	}

	// Integral gain
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

	const float i = m_sommeError * m_ki;

	return p + i + d;
}



void PIDBlock::run(const float& dt)
{
	const float error = m_target - m_measure;
	m_output = computePID(error, m_measure, dt, true);
}




