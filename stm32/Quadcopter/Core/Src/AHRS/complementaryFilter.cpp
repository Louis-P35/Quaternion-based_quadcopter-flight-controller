/*
 * complementaryFilter.cpp
 *
 *  Created on: Jun 22, 2024
 *      Author: Louis
 */


// Project
#include "AHRS/complementaryFilter.hpp"



/*
 * Implementation of the quaternion based complementary filter
 */
Quaternion ComplementaryFilter::compute(
		const Vector<double, 3>& acc,
		const Vector<double, 3>& gyro,
		const Vector<double, 3>& magneto,
		const double& dt
		)
{
	// Quaternion from gyroscope
	const Quaternion qGyroDelta = Quaternion::gyroToQuaternion(gyro * DEGREE_TO_RAD, dt);
	const Quaternion qGyroNew = m_qEst * qGyroDelta;

	// Quaternion from accelerometer and magnetometer
	const Quaternion qAccMag = Quaternion::accMagToQuaternion(
			acc.normalized(),
			magneto.normalized()
			);

	// Complementary filter to blend the two quaternions
	m_qEst = qGyroNew * (1.0 - m_alpha) + qAccMag * m_alpha;
	m_qEst.normalize();

	return m_qEst;
}

