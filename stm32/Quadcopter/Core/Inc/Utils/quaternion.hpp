/*
 * quaternion.hpp
 *
 *  Created on: Jun 15, 2024
 *      Author: Louis
 */

#pragma once

// STL
#include <cmath>

// Project
#include "Utils/vectorNd.hpp"

#define RAD_TO_DEGREE (180.0 / M_PI)


/*
 * Quaternion class
 */
// TODO: template type
class Quaternion
{
public:
	// Init quaternion as unit vector
	Vector<double, 4> m_q = {1.0, 0.0, 0.0, 0.0};

	Quaternion(double w, double x, double y, double z) : m_q({w, x, y, z})
	{

	}

	Quaternion(const Vector<double, 4>& v) : m_q(v)
	{

	}

	Quaternion()
	{

	}

	/*
	 * Return the conjugated quaternion (imaginary component with the reverse sign)
	 */
	inline Quaternion conjugate() const
	{
		return Quaternion(m_q.m_vect[0], -m_q.m_vect[1], -m_q.m_vect[2], -m_q.m_vect[3]);
	}

	/*
	 * Return the norm of the quaternion
	 */
	inline double norm() const
	{
		return m_q.norm();
	}

	/*
	 * Return the norm squared of the quaternion
	 */
	inline double normSquared() const
	{
		// The dot product with itself is equal to the squared norm
		return m_q.dot(m_q);
	}

	/*
	 * Normalize the quaternion
	 */
	inline Quaternion& normalize()
	{
		m_q.normalize();

		return *this;
	}

	/*
	 * Inverse of the quaternion
	 */
	inline Quaternion inverse() const
	{
		double norm_sq = normSquared(); // TODO: norm ? norm squared ?

		// Handle zero norm case, though it should ideally never occur
		if (norm_sq == 0.0)
			return Quaternion(0.0, 0.0, 0.0, 0.0);

		const Quaternion conj = conjugate();

		return Quaternion(
				conj.m_q.m_vect[0] / norm_sq,
				conj.m_q.m_vect[1] / norm_sq,
				conj.m_q.m_vect[2] / norm_sq,
				conj.m_q.m_vect[3] / norm_sq
				);
	}

	/*
	 * Return the dot product of two quaternion
	 */
	inline double dotProduct(const Quaternion& b) const
	{
		return m_q.dot(b.m_q);
	}


	//void print() const // TODO


	/*
	 * Compute the Euler angle from the quaternion
	 * Assume roll is x, pitch is y and yaw is z
	 * Assume right hand system
	 */
	void toEuler(double& roll, double& pitch, double& yaw) const
	{
		// Roll
		double sinr_cosp = 2.0 * (m_q.m_vect[0] * m_q.m_vect[1] + m_q.m_vect[2] * m_q.m_vect[3]);
		double cosr_cosp = 1.0 - 2.0 * (m_q.m_vect[1] * m_q.m_vect[1] + m_q.m_vect[2] * m_q.m_vect[2]);
		roll = atan2(sinr_cosp, cosr_cosp);
		roll *= RAD_TO_DEGREE;

		// Pitch
		double sinp = 2.0 * (m_q.m_vect[0] * m_q.m_vect[2] - m_q.m_vect[3] * m_q.m_vect[1]);
		if (abs(sinp) >= 1.0)
		{
			pitch = copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
		}
		else
		{
			pitch = asin(sinp);
		}
		pitch *= RAD_TO_DEGREE;

		// Yaw
		double siny_cosp = 2.0 * (m_q.m_vect[0] * m_q.m_vect[3] + m_q.m_vect[1] * m_q.m_vect[2]);
		double cosy_cosp = 1.0 - 2.0 * (m_q.m_vect[2] * m_q.m_vect[2] + m_q.m_vect[3] * m_q.m_vect[3]);
		yaw = atan2(siny_cosp, cosy_cosp);
		yaw *= RAD_TO_DEGREE;
	}

	/*
	 * Static function to convert Euler angles to Quaternion (in ZXY order)
	 */
	static Quaternion fromEuler(const double& roll, const double& pitch, const double& yaw)
	{
		const double cy = cos(yaw * 0.5);
		const double sy = sin(yaw * 0.5);
		const double cp = cos(pitch * 0.5);
		const double sp = sin(pitch * 0.5);
		const double cr = cos(roll * 0.5);
		const double sr = sin(roll * 0.5);

		const double w = cr * cp * cy + sr * sp * sy;
		const double x = sr * cp * cy - cr * sp * sy;
		const double y = cr * sp * cy + sr * cp * sy;
		const double z = cr * cp * sy - sr * sp * cy;

		return Quaternion(w, x, y, z);
	}

	/*
	 * Integrate the angular velocity and return
	 * the corresponding quaternion rotation
	 */
	static Quaternion gyroToQuaternion(
			const Vector<double, 3>& gyro,
			const double dt
			)
	{
		if (gyro.norm() > 0.0 && dt != 0.0) // TODO: Norm computed twice, need to optimize that
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

		// Return identity quaternion if norm is zero
		return Quaternion(1.0, 0.0, 0.0, 0.0);
	}


	/*
	 * Return the quaternion that represent the orientation given
	 * by the accelerometer and magnetometer
	 */
	static Quaternion accMagToQuaternion(
			const Vector<double, 3>& accel,
			const Vector<double, 3>& mag
			)
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
	 * Rotate a vector using this quaternion
	 */
	Vector<double, 3> rotate(const Vector<double, 3>& v) const
	{
		// Convert vector to quaternion
		const Quaternion vec(0, v.m_vect[0], v.m_vect[1], v.m_vect[2]);

		// Compute this * vec * conjugate(this)
		const Quaternion q_vec = *this * vec * conjugate();

		// Return the vector part of the resulting quaternion
		return Vector<double, 3>{q_vec.m_q.m_vect[1], q_vec.m_q.m_vect[2], q_vec.m_q.m_vect[3]};
	}

	/* Operators overload */

	/*
	 * Quaternion multiplication overload
	 */
	inline Quaternion operator*(const Quaternion& b) const
	{
		return Quaternion(
				m_q.m_vect[0] * b.m_q.m_vect[0] - m_q.m_vect[1] * b.m_q.m_vect[1] - m_q.m_vect[2] * b.m_q.m_vect[2] - m_q.m_vect[3] * b.m_q.m_vect[3],  // Real part
				m_q.m_vect[0] * b.m_q.m_vect[1] + m_q.m_vect[1] * b.m_q.m_vect[0] + m_q.m_vect[2] * b.m_q.m_vect[3] - m_q.m_vect[3] * b.m_q.m_vect[2],  // i
				m_q.m_vect[0] * b.m_q.m_vect[2] - m_q.m_vect[1] * b.m_q.m_vect[3] + m_q.m_vect[2] * b.m_q.m_vect[0] + m_q.m_vect[3] * b.m_q.m_vect[1],  // j
				m_q.m_vect[0] * b.m_q.m_vect[3] + m_q.m_vect[1] * b.m_q.m_vect[2] - m_q.m_vect[2] * b.m_q.m_vect[1] + m_q.m_vect[3] * b.m_q.m_vect[0]   // k
				);
	}

	/*
	 * Assignment and Quaternion multiplication overload
	 */
	inline Quaternion& operator*=(const Quaternion& b)
	{
		Quaternion tmp = *this * b;

		m_q = tmp.m_q;

		return *this;
	}

	/*
	 * Scalar multiplication overload
	 */
	inline Quaternion operator*(const double& scalar) const
	{
		return Quaternion(m_q * scalar);
	}

	/*
	 * Assignment and scalar multiplication overload
	 */
	inline Quaternion& operator*=(const double& scalar)
	{
		m_q *= scalar;

		return *this;
	}

	/*
	 * Quaternion addition overload
	 */
	inline Quaternion operator+(const Quaternion& b) const
	{
		return Quaternion(m_q + b.m_q);
	}

	/*
	 * Assignment and quaternion addition overload
	 */
	inline Quaternion& operator+=(const Quaternion& b)
	{
		m_q += b.m_q;

		return *this;
	}

	/*
	 * Quaternion subtraction overload
	 */
	inline Quaternion operator-(const Quaternion& b) const
	{
		return Quaternion(m_q - b.m_q);
	}

	/*
	 * Assignment and quaternion subtraction overload
	 */
	inline Quaternion& operator-=(const Quaternion& b)
	{
		m_q -= b.m_q;

		return *this;
	}
};



