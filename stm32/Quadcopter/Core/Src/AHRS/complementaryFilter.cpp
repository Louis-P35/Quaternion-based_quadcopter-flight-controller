/*
 * complementaryFilter.cpp
 *
 *  Created on: Jun 22, 2024
 *      Author: Louis
 */


// Project
#include "AHRS/complementaryFilter.hpp"
#include "Utils/utilsAlgebra.hpp"



/*
 * Implementation of the quaternion based complementary filter
 */
Eigen::Quaterniond ComplementaryFilter::compute(
		const Eigen::Vector3d& acc,
		const Eigen::Vector3d& gyro,
		const Eigen::Vector3d& magneto,
		const double& dt
		)
{
	// Quaternion from gyroscope
	const Eigen::Quaterniond qGyroDelta = Utils::gyroToQuaternion(gyro * DEGREE_TO_RAD, dt);
	const Eigen::Quaterniond qGyroNew = m_qEst * qGyroDelta;
	//m_qEst = qGyroNew;

	// Quaternion from accelerometer and magnetometer
	const Eigen::Quaterniond qAccMag = Utils::accMagToQuaternion(acc, magneto);

	// Complementary filter to blend the two quaternions
	//m_qEst = qGyroNew * (1.0 - m_alpha) + qAccMag * m_alpha;
	// Complementary filter to blend the two quaternions using slerp
	m_qEst = qGyroNew.slerp(m_alpha, qAccMag);
	m_qEst.normalize();

	return m_qEst;
}

