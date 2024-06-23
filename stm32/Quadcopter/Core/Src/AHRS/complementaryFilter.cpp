/*
 * complementaryFilter.cpp
 *
 *  Created on: Jun 22, 2024
 *      Author: Louis
 */


// Project
#include "AHRS/complementaryFilter.hpp"




/*
 * Integrate the angular velocity and return
 * the corresponding quaternion rotation
 */
Quaternion ComplementaryFilter::gyroToQuaternion(
		const Vector<double, 3>& gyro,
		const double dt
		) const
{
	// Update the orientation quaternion using half `dt` to correctly integrate angular velocity.
	// This accounts for quaternion properties where each component of angular velocity contributes
	// half as much as it would in a straightforward vector integration,
	// ensuring accurate rotational updates.

	const double half_dt = 0.5 * dt;

	const double w = 1.0;
	const double x = gyro.m_vect[0] * half_dt;
	const double y = gyro.m_vect[1] * half_dt;
	const double z = gyro.m_vect[2] * half_dt;

	return Quaternion(w, x, y, z).normalize();
}


/*
 * Return the quaternion that represent the orientation given
 * by the accelerometer and magnetometer
 */
Quaternion ComplementaryFilter::accMagToQuaternion(
		const Vector<double, 3>& accel,
		const Vector<double, 3>& mag
		) const
{
	// Calculate roll and pitch from accelerometer
	const double roll = atan2(accel.m_vect[1], accel.m_vect[2]);
	const double pitch = atan2(
			-accel.m_vect[0],
			sqrt(accel.m_vect[1] * accel.m_vect[1] + accel.m_vect[2] * accel.m_vect[2])
			);

	// Calculate yaw from magnetometer (simplified, assumes flat Earth and no tilt compensation)
	// TODO: Use final estimated attitude ?
	const double magX = mag.m_vect[0] * cos(pitch) + mag.m_vect[1] * sin(roll) * sin(pitch) +
			mag.m_vect[2] * cos(roll) * sin(pitch);
	const double magY = mag.m_vect[1] * cos(roll) - mag.m_vect[2] * sin(roll);
	const double yaw = atan2(-magY, magX);

	return Quaternion::fromEuler(roll, pitch, yaw);
}


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
	const Quaternion qGyroDelta = gyroToQuaternion(gyro, dt);
	const Quaternion qGyroNew = m_qEst * qGyroDelta;

	// Quaternion from accelerometer and magnetometer
	const Quaternion qAccMag = accMagToQuaternion(acc.normalized(), magneto.normalized());

	// Complementary filter to blend the two quaternions
	m_qEst = qGyroNew * (1.0 - m_alpha) + qAccMag * m_alpha;
	m_qEst.normalize();

	return m_qEst;
}

