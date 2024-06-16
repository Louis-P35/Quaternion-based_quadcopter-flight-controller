/*
 * madgwick.hpp
 *
 *  Created on: Jun 15, 2024
 *      Author: Louis
 */

#pragma once

// Project
#include "Utils/quaternion.hpp"

/*
The Madgwick filter utilizes quaternion representations for calculating orientations.
Quaternions are particularly well-suited for representing three-dimensional rotations because
they do not suffer from gimbal lock—a condition where the loss of one degree of freedom
in three-dimensional space causes two of the three rotation axes to align, which can
happen with Euler angles. Quaternions provide a compact, non-singular representation which
ensures stable and continuous calculations even during full 360-degree rotations.
*/


class MadgwickFilter
{
public:
	Quaternion m_qEst = Quaternion(1.0, 0.0, 0.0, 0.0);

public:
	MadgwickFilter()
	{
		m_qEst = Quaternion(1.0, 0.0, 0.0, 0.0);
	}

	void compute(
			const Vector<double, 3>& acc,
			const Vector<double, 3>& gyro,
			const double& dt
			);

	void getEulerAngle(double& roll, double& pitch, double& yaw) const;
};



