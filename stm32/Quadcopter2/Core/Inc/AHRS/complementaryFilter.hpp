/*
 * complementaryFilter.hpp
 *
 *  Created on: Jun 22, 2024
 *      Author: Louis
 */

#pragma once

// Project
#include "AHRS/ahrs.hpp"
#include "Utils/Eigen/Dense"



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
	ComplementaryFilter()
	{
		m_qEst = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	};
	ComplementaryFilter(const double alpha) : m_alpha(alpha) {};

	Eigen::Quaterniond compute(
			const Eigen::Vector3d& acc,
			const Eigen::Vector3d& gyro,
			const Eigen::Vector3d& magneto,
			const double& dt
			) override;
};
