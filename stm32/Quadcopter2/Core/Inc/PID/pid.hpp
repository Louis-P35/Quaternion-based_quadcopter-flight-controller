/*
 * pid.hpp
 *
 *  Created on: Jun 17, 2024
 *      Author: Louis
 */

#pragma once

// Project

// External lib
#include "Utils/Eigen/Dense"


/*
 * PID class
 */
class PID
{
private:
	// PID coefficients
	double m_kp = 0.0;
	double m_ki = 0.0;
	double m_kd = 0.0;

	// Max integrated value
	double m_saturation = 100.0;

	double m_previousError = 0.0;
	double m_sommeError = 0.0;

public:
	// Constructor to initialize PID gains
	PID(double kp, double ki, double kd, double sat) :
		m_kp(kp), m_ki(ki), m_kd(kd), m_saturation(sat)
	{
		m_previousError = 0.0;
		m_sommeError = 0.0;
	}

	// For Euler-based PID
	static double getError(const double& current, const double& target);

	// For Quaternion-based PID
	static Eigen::Quaterniond getError(
			const Eigen::Quaterniond& current,
			const Eigen::Quaterniond& target
			);

	double computePID(const double& error, const double& dt, const bool& integrate);
};



