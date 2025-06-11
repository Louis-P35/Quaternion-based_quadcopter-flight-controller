/*
 * madgwick.hpp
 *
 *  Created on: Apr 26, 2025
 *      Author: louis
 */

#pragma once

#include "Utils/quaternion.hpp"

/*
The Madgwick filter utilizes quaternion representations for calculating orientations.
Quaternions are particularly well-suited for representing three-dimensional rotations because
they do not suffer from gimbal lock—a condition where the loss of one degree of freedom
in three-dimensional space causes two of the three rotation axes to align, which can
happen with Euler angles. Quaternions provide a compact, non-singular representation which
ensures stable and continuous calculations even during full 360-degree rotations.
*/

template<typename T>
class MadgwickFilter
{
public:
	Quaternion<T> m_qEst = Quaternion<T>::identity();

public:
	MadgwickFilter()
	{
		m_qEst = Quaternion<T>::identity();
	}

	void compute(
		const T& ax,
		const T& ay,
		const T& az,
		const T& gx,
		const T& gy,
		const T& gz,
		const T& dt
		);

	void getEulerAngle(T& roll, T& pitch, T& yaw);
};
