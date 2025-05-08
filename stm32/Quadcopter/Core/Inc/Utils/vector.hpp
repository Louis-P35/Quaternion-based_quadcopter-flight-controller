/*
 * vector3.hpp
 *
 *  Created on: May 2, 2025
 *      Author: louis
 */

#pragma once

// Includes from STL
#include <cmath>



template<typename T>
class Vector3
{
public:
	T m_x = 0.0;
	T m_y = 0.0;
	T m_z = 0.0;

	constexpr Vector3() noexcept = default;
	constexpr Vector3(const T& x, const T& y, const T& z) noexcept :
		m_x(x), m_y(y), m_z(z) {};
	constexpr Vector3(const T& a) noexcept :
			m_x(a), m_y(a), m_z(a) {};

	constexpr Vector3& operator+=(const Vector3& b) noexcept
	{
		m_x += b.m_x;
		m_y += b.m_y;
		m_z += b.m_z;

		return *this;
	};

	constexpr Vector3 operator+(const Vector3& b) const noexcept
	{
		return Vector3(m_x + b.m_x, m_y + b.m_y, m_z + b.m_z);
	};

	constexpr Vector3& operator-=(const Vector3& b) noexcept
	{
		m_x -= b.m_x;
		m_y -= b.m_y;
		m_z -= b.m_z;

		return *this;
	};

	constexpr Vector3 operator-(const Vector3& b) const noexcept
	{
		return Vector3(m_x - b.m_x, m_y - b.m_y, m_z - b.m_z);
	};

	constexpr Vector3 operator*(T scalar) const noexcept
	{
		return Vector3(m_x * scalar, m_y * scalar, m_z * scalar);
	};

	constexpr Vector3& operator*=(T scalar) const noexcept
	{
		m_x *= scalar;
		m_y *= scalar;
		m_z *= scalar;

		return *this;
	};

	constexpr Vector3 operator/(T scalar) const
	{
		return Vector3(m_x / scalar, m_y / scalar, m_z / scalar);
	};

	constexpr Vector3& operator/=(T scalar) noexcept
	{
		m_x /= scalar;
		m_y /= scalar;
		m_z /= scalar;

		return *this;
	};

	constexpr Vector3 cross(Vector3 const& b) const noexcept
	{
		return Vector3(
				m_y * b.m_z - m_z * b.m_y,
				m_z * b.m_x - m_x * b.m_z,
				m_x * b.m_y - m_y * b.m_x
				);
	};

	T norm() const noexcept
	{
		return std::sqrt(dot(*this));
	};

	constexpr T dot(Vector3 const& b) const noexcept
	{
		return m_x * b.m_x + m_y * b.m_y + m_z * b.m_z;
	};

	Vector3 normalized() const
	{
		T n = norm();

		return n > static_cast<T>(0) ? *this / n : Vector3{};
	};
};
