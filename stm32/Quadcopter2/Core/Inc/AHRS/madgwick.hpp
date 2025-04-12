/*
 * madgwick.hpp
 *
 *  Created on: Jun 15, 2024
 *      Author: Louis
 */

#pragma once

// Project
//#include "Utils/quaternion.hpp"
#include "AHRS/ahrs.hpp"

// External lib
#include "Utils/Eigen/Dense"

/*
The Madgwick filter utilizes quaternion representations for calculating orientations.
Quaternions are particularly well-suited for representing three-dimensional rotations because
they do not suffer from gimbal lock—a condition where the loss of one degree of freedom
in three-dimensional space causes two of the three rotation axes to align, which can
happen with Euler angles. Quaternions provide a compact, non-singular representation which
ensures stable and continuous calculations even during full 360-degree rotations.
*/


class MadgwickFilter : public IFilter
{
public:
	MadgwickFilter()
	{
		m_qEst = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	}

	Eigen::Quaterniond compute(
			const Eigen::Vector3d& acc,
			const Eigen::Vector3d& gyro,
			const Eigen::Vector3d& magneto,
			const double& dt
			) override;

	void getEulerAngle(double& roll, double& pitch, double& yaw) const; // TODO remove ?
};



