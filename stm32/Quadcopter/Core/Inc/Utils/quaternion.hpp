/*
 * quaternion.hpp
 *
 *  Created on: Apr 26, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Utils/vector.hpp"

// Includes from STL
#include <cmath>


namespace quaternionMath
{
	template<typename T>
	inline constexpr T rad_to_deg = T(180) / M_PI;
	template<typename T>
	inline constexpr T half_pi    = M_PI / T(2);
}


/*
 * Quaternion class
 */
template<typename T>
class Quaternion
{
public:
	// Init quaternion as unit vector
	T m_w = T(1.0);
	T m_x = T(0.0);
	T m_y = T(0.0);
	T m_z = T(0.0);

	constexpr Quaternion(T w, T x, T y, T z) noexcept : m_w(w), m_x(x), m_y(y), m_z(z)
	{

	};

	constexpr Quaternion() noexcept = default;

	constexpr static Quaternion iddentity()
	{
		return Quaternion(T(1.0), T(0.0), T(0.0), T(0.0));
	};

	// Return the conjugated quaternion (imaginary component with the reverse sign)
	constexpr Quaternion conjugate() const noexcept
	{
		return Quaternion(m_w, -m_x, -m_y, -m_z);
	};

	// Return the norm of the quaternion
	T norm() const noexcept
	{
		return std::sqrt(m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z);
	};

	// Return the norm squared of the quaternion
	constexpr T normSquared() const noexcept
	{
		return m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z;
	};

	// Normalize the quaternion
	Quaternion& normalize() noexcept
	{
		T _norm = norm();

		if (_norm != static_cast<T>(0.0))
		{
			m_w /= _norm;
			m_x /= _norm;
			m_y /= _norm;
			m_z /= _norm;
		}

		return *this;
	};

	// Inverse of the quaternion
	Quaternion inverse() const noexcept
	{
		T norm_sq = normSquared();

		// Handle zero norm case, though it should ideally never occur
		if (norm_sq == 0.0)
		{
			return Quaternion(
					static_cast<T>(0.0),
					static_cast<T>(0.0),
					static_cast<T>(0.0),
					static_cast<T>(0.0)
					);
		}

		Quaternion conj = conjugate();

		// If the quaternion is normalized already (norm_sq ≈ 1),
		// return it's conjugate
		if (std::abs(norm_sq - static_cast<T>(1.0)) < static_cast<T>(1e-6))
		{
			return conj;
		}

		return Quaternion(conj.m_w / norm_sq, conj.m_x / norm_sq, conj.m_y / norm_sq, conj.m_z / norm_sq);
	}

	// Return the dot product of two quaternion
	constexpr T dotProduct(const Quaternion& b) const noexcept
	{
		return m_w * b.m_w + m_x * b.m_x + m_y * b.m_y + m_z * b.m_z;
	}

	// Compute the Euler angle from the quaternion
	// Assume roll is x, pitch is y and yaw is z
	// Assume right hand system
	void toEuler(T& roll, T& pitch, T& yaw) const noexcept
	{
		// Roll
		T sinr_cosp = static_cast<T>(2.0) * (m_w * m_x + m_y * m_z);
		T cosr_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (m_x * m_x + m_y * m_y);
		roll = std::atan2(sinr_cosp, cosr_cosp);
		roll *= quaternionMath::rad_to_deg<T>;

		// Pitch
		T sinp = static_cast<T>(2.0) * (m_w * m_y - m_z * m_x);
		if (std::abs(sinp) >= static_cast<T>(1.0))
		{
			pitch = std::copysign(quaternionMath::half_pi<T>, sinp); // use 90 degrees if out of range
		}
		else
		{
			pitch = std::asin(sinp);
		}
		pitch  *= quaternionMath::rad_to_deg<T>;

		// Yaw
		T siny_cosp = 2.0 * (m_w * m_z + m_x * m_y);
		T cosy_cosp = 1.0 - 2.0 * (m_y * m_y + m_z * m_z);
		yaw = std::atan2(siny_cosp, cosy_cosp);
		yaw *= quaternionMath::rad_to_deg<T>;
	}

	// Static function to convert Euler angles to Quaternion (in XYZ order) (roll, pitch then yaw)
	static Quaternion fromEuler(const T& roll, const T& pitch, const T& yaw) noexcept
	{
		const T cr = std::cos(roll * static_cast<T>(0.5));
		const T sr = std::sin(roll * static_cast<T>(0.5));
		const T cp = std::cos(pitch * static_cast<T>(0.5));
		const T sp = std::sin(pitch * static_cast<T>(0.5));
		const T cy = std::cos(yaw * static_cast<T>(0.5));
		const T sy = std::sin(yaw * static_cast<T>(0.5));

		const T w = cr * cp * cy - sr * sp * sy;
		const T x = sr * cp * cy + cr * sp * sy;
		const T y = cr * sp * cy - sr * cp * sy;
		const T z = cr * cp * sy + sr * sp * cy;

		return Quaternion(w, x, y, z);
	}

	inline static constexpr  Quaternion canonical(const Quaternion& q) noexcept
	{
	    return (q.m_w >= static_cast<T>(0.0)) ? q : Quaternion(-q.m_w, -q.m_x, -q.m_y, -q.m_z);
	}


	/*
	 * Get the quaternion's axis of rotation and angle of rotation in radian.
	 */
	void toAxisAngle(Vector3<T>& axis, T& angleRad) const noexcept
	{
	    Quaternion q = *this;

	    // Normalize to ensure it's a unit quaternion
	    q.normalize();

	    // Clamp w to [-1, 1] to avoid domain errors in acos
	    double w_clamped = q.m_w;
	    if (w_clamped > static_cast<T>(1.0))
	    {
	    	w_clamped = static_cast<T>(1.0);
	    }
	    if (w_clamped < static_cast<T>(-1.0))
	    {
	    	w_clamped = static_cast<T>(-1.0);
	    }

	    // Compute the angle (in radians)
	    angleRad = static_cast<T>(2.0) * static_cast<T>(std::acos(w_clamped));

	    // Compute the scale factor (norm of the imaginary part)
	    T s = std::sqrt(static_cast<T>(1.0) - w_clamped * w_clamped);

	    if (s < 1e-6)
	    {
	        // If s is very small, the rotation is near zero; axis is arbitrary
	        axis.m_x = q.m_x;
	        axis.m_y = q.m_y;
	        axis.m_z = q.m_z;
	    }
	    else
	    {
	        // Normalize the imaginary part to get the rotation axis
	        axis.m_x = q.m_x / s;
	        axis.m_y = q.m_y / s;
	        axis.m_z = q.m_z / s;
	    }
	}



  /* Operators override */

	// Quaternion multiplication override
	constexpr Quaternion operator*(const Quaternion& b) const noexcept
	{
		return Quaternion(
		  m_w * b.m_w - m_x * b.m_x - m_y * b.m_y - m_z * b.m_z,  // Real part
		  m_w * b.m_x + m_x * b.m_w + m_y * b.m_z - m_z * b.m_y,  // i
		  m_w * b.m_y - m_x * b.m_z + m_y * b.m_w + m_z * b.m_x,  // j
		  m_w * b.m_z + m_x * b.m_y - m_y * b.m_x + m_z * b.m_w   // k
		);
	}

	// Assignment and Quaternion multiplication override
	constexpr Quaternion& operator*=(const Quaternion& b) noexcept
	{
		Quaternion tmp = *this * b;

		m_w = tmp.m_w;
		m_x = tmp.m_x;
		m_y = tmp.m_y;
		m_z = tmp.m_z;

		return *this;
	}

	// Scalar multiplictaion override
	constexpr Quaternion operator*(const T& scalar) const noexcept
	{
		return Quaternion(m_w * scalar, m_x * scalar, m_y * scalar, m_z * scalar);
	}

	// Assignment and scalar multiplication override
	constexpr Quaternion& operator*=(const T& scalar) noexcept
	{
		m_w *= scalar;
		m_x *= scalar;
		m_y *= scalar;
		m_z *= scalar;

		return *this;
	}

	// Quaternion addition override
	constexpr Quaternion operator+(const Quaternion& b) const noexcept
	{
		return Quaternion(m_w + b.m_w, m_x + b.m_x, m_y + b.m_y, m_z + b.m_z);
	}

	// Assignment and quaternion addition override
	constexpr Quaternion& operator+=(const Quaternion& b) noexcept
	{
		m_w += b.m_w;
		m_x += b.m_x;
		m_y += b.m_y;
		m_z += b.m_z;

		return *this;
	}

	// Quaternion substraction override
	constexpr Quaternion operator-(const Quaternion& b) const noexcept
	{
		return Quaternion(m_w - b.m_w, m_x - b.m_x, m_y - b.m_y, m_z - b.m_z);
	}

	// Assignment and quaternion substraction override
	constexpr Quaternion& operator-=(const Quaternion& b) noexcept
	{
		m_w -= b.m_w;
		m_x -= b.m_x;
		m_y -= b.m_y;
		m_z -= b.m_z;

		return *this;
	}
};
