/*
 * ahrs.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Louis
 */

#pragma once

// Project
#include "Utils/quaternion.hpp"


/*
 * Interface class for all the filters
 * (Madgwick filter, extended Kalman filter and complementary filter)
 */
class IFilter
{
protected:
	Quaternion m_qEst;

public:
	IFilter() {};
	virtual Quaternion compute(const Vector<double, 3>& acc,
				const Vector<double, 3>& gyro,
				const Vector<double, 3>& magneto,
				const double& dt
				) = 0;

	Quaternion getEstimateAttitude() const
	{
		return m_qEst;
	};
};



/*
 * AHRS class
 */
class AHRS
{
private:
	// Filter, can be Extended Kalman, Madgwick or complementary filter
	IFilter* m_pFilter = nullptr;

public:
	AHRS(IFilter* const pF) : m_pFilter(pF) {};

	Quaternion computeAHRS(const Vector<double, 3>& acc,
			const Vector<double, 3>& gyro,
			const Vector<double, 3>& magneto,
			const double& dt
			);
};
