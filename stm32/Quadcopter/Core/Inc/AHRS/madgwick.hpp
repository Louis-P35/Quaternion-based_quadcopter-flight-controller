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

	// Accel + gyro only (existing, call at IMU rate)
	void compute(
		const T& ax,
		const T& ay,
		const T& az,
		const T& gx,
		const T& gy,
		const T& gz,
		const T& dt
		);

	// Accel + gyro + magnetometer — Madgwick MARG algorithm (Madgwick 2010, eq. 29/34)
	// Gravity reference : [0, 0, 1] (Z up, ENU-compatible)
	// Magnetic reference: derived from current estimate; yaw tracks magnetic North
	// Call at the magnetometer update rate (typ. 100 Hz); falls back to compute() if mag is invalid
	void computeMARG(
		const T& ax, const T& ay, const T& az,  // accelerometer, any unit (normalised internally)
		const T& gx, const T& gy, const T& gz,  // gyroscope, rad/s
		const T& mx, const T& my, const T& mz,  // magnetometer, any unit (normalised internally)
		const T& dt
		);

	void getEulerAngle(T& roll, T& pitch, T& yaw);
};
