/*
 * complementaryFilter.hpp
 *
 *  Created on: Jun 22, 2024
 *      Author: Louis
 */

#pragma once

// Project
#include "AHRS/ahrs.hpp"



/*
 * This is the complementary filter class, operating in quaternion space.
 * Complementary filter is a basic low pass filter and high pass filter.
 * Low pass filter for accelerometer data.
 * high pass filter for gyroscope data.
 */
class ComplementaryFilter : public IFilter
{
private:
	// Coefficient
	double m_alpha = 0.98;

public:
	ComplementaryFilter() {};
	ComplementaryFilter(const double alpha) : m_alpha(alpha) {};

	Quaternion compute(
			const Vector<double, 3>& acc,
			const Vector<double, 3>& gyro,
			const Vector<double, 3>& magneto,
			const double& dt
			) override;

private:
	Quaternion gyroToQuaternion(const Vector<double, 3>& gyro, const double dt) const;
	Quaternion accMagToQuaternion(
			const Vector<double, 3>& accel,
			const Vector<double, 3>& mag
			) const;
};
