/*
 * VectorNd.hpp
 *
 *  Created on: Jun 15, 2024
 *      Author: Louis
 */

#pragma once

// STL
#include <array>
#include <cmath>


/*
 * Class that handle N dimensional vector
 * of any type.
 */
template<typename T, size_t N>
class Vector
{
public:
	// std::array is preferred as std::vector
	// because the target is a microcontroler
	std::array<T, N> m_vect;


public:
	Vector() {};

	/*
	 * Variadic template constructor to initialize vector elements
	 */
	template<typename ... Args>
	Vector(const Args&... args) : m_vect({static_cast<T>(args)...})
	{
		static_assert(sizeof...(Args) == N, "Incorrect vector dimension.");
	}

	/*
	 * Type conversion constructor
	 */
	template<typename U>
	Vector(const Vector<U, N>& v)
	{
		for (size_t i = 0; i < N; ++i)
		{
			m_vect[i] = static_cast<T>(v.m_vect[i]);
		}
	}

	//void print(); // TODO, over UART

	/*
	 * Addition operator overload
	 */
	inline Vector operator+(const Vector& v) const
	{
		Vector result;

		for (size_t i = 0; i < N; ++i)
		{
			result.m_vect[i] = m_vect[i] + v.m_vect[i];
		}

		return result;
	}

	inline void operator+=(const Vector& v)
	{
		for (size_t i = 0; i < N; ++i)
		{
			m_vect[i] += v.m_vect[i];
		}
	}

	/*
	 * Subtraction operator overload
	 */
	inline Vector operator-(const Vector& v) const
	{
		Vector result;

		for (size_t i = 0; i < N; ++i)
		{
			result.m_vect[i] = m_vect[i] - v.m_vect[i];
		}

		return result;
	}

	inline void operator-=(const Vector& v)
	{
		for (size_t i = 0; i < N; ++i)
		{
			m_vect[i] -= v.m_vect[i];
		}
	}

	/*
	 * Scalar multiplication operator overload
	 */
	inline Vector operator*(const T& scalar) const
	{
		Vector result;

		for (size_t i = 0; i < N; ++i)
		{
			result.m_vect[i] = m_vect[i] * scalar;
		}

		return result;
	}

	inline void operator*=(const T& scalar)
	{
		for (size_t i = 0; i < N; ++i)
		{
			m_vect[i] *= scalar;
		}
	}

	/*
	 * Scalar division operator overload
	 */
	inline Vector operator/(const T& scalar) const
	{
		// TODO: Handle division by 0 ?

		Vector result;

		for (size_t i = 0; i < N; ++i)
		{
			result.m_vect[i] = m_vect[i] / scalar;
		}

		return result;
	}

	inline void operator/=(const T& scalar)
	{
		// TODO: Handle division by 0 ?

		for (size_t i = 0; i < N; ++i)
		{
			m_vect[i] /= scalar;
		}
	}

	/*
	 * Dot product
	 */
	inline T dot(const Vector& v) const
	{
		T sum = 0;

		for (size_t i = 0; i < N; i++)
		{
			sum += m_vect[i] * v.m_vect[i];
		}

		return sum;
	}

	/*
	 * Norm calculation
	 */
	inline T norm() const
	{
		// The dot product with itself is equal to the squared norm
		return std::sqrt(dot(*this));
	}

	/*
	 * Normalize the vector
	 */
	inline void normalize()
	{
		const double _norm = norm();

		if (_norm != 0.0)
		{
			*this /= _norm;
		}
	}

	/*
	 * Return the normalized vector
	 */
	inline Vector normalized() const
	{
		Vector result = Vector(*this);

		result.normalize();

		return result;
	}

	/*
	 * Cross product
	 */
	inline Vector operator^(const Vector& v) const
	{
		//static_assert(std::is_same<decltype(v), Vector<T, N>>::value == 0, "Vectors must be the same size");
		static_assert(N == 3, "Can not cross product vectors with dimensions different than 3");

		Vector result;

		result.m_vect[0] = m_vect[1] * v.m_vect[2] - m_vect[2] * v.m_vect[1];
		result.m_vect[1] = m_vect[0] * v.m_vect[2] - m_vect[2] * v.m_vect[0];
		result.m_vect[2] = m_vect[0] * v.m_vect[1] - m_vect[1] * v.m_vect[0];

		return result;
	}
};
