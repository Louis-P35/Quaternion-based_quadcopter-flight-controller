/*
 * ahrs.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Louis
 */

#pragma once

// Project
#include "Utils/Eigen/Dense"


/*
 * Interface class for all the filters
 * (Madgwick filter, extended Kalman filter and complementary filter)
 */
class IFilter
{
protected:
	Eigen::Quaterniond m_qEst;

public:
	IFilter()
	{
		m_qEst = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	};
	virtual Eigen::Quaterniond compute(const Eigen::Vector3d& acc,
				const Eigen::Vector3d& gyro,
				const Eigen::Vector3d& magneto,
				const double& dt
				) = 0;

	Eigen::Quaterniond getEstimateAttitude() const
	{
		return m_qEst;
	};
};



/*
 * AHRS class.
 * Use one filter (extended Kalman, complementary or Madgwick)
 * to estimate attitude.
 */
class AHRS
{
private:
	// Filter, can be Extended Kalman, Madgwick or complementary filter
	IFilter* m_pFilter = nullptr;

public:
	AHRS(IFilter* const pF) : m_pFilter(pF) {};

	Eigen::Quaterniond computeAHRS(
			const Eigen::Vector3d& acc,
			const Eigen::Vector3d& gyro,
			const Eigen::Vector3d& magneto,
			const double& dt
			)
	{
		if (m_pFilter != nullptr)
			return m_pFilter->compute(acc, gyro, magneto, dt);

		return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	};
};
