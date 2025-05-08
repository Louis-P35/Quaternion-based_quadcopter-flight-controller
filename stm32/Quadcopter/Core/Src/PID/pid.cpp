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
double PID::getError(const double& current, const double& target)
{
	return target - current;
}

/*
 * Compute error
 * For Quaternion-based PID
 */
Quaternion<double> PID::getError(const Quaternion<double>& current, const Quaternion<double>& target)
{
	/*
	Computes the rotation necessary to go from the current orientation to the target orientation.
	This operation is from the perspective of the current orientation being the reference frame, and
	it calculates the relative rotation needed to align exactly with the target.
	*/
	Quaternion<double> qError = target * current.inverse();

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
double PID::computePID(const double& error, const double& dt, const bool& integrate)
{
	// Proportionnal gain
	double p = error * m_kp;

	// Derivative gain
	double derivedError = (error - m_previousError) / dt;
	double d = derivedError * m_kd;
	m_previousError = error;

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

	double i = m_sommeError * m_ki;

	return p + i + d;
}



void PIDBlock::run(double dt)
{
	const double error = m_target - m_measure;
	m_output = computePID(error, dt, true);
}




