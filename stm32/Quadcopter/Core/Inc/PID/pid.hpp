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
	float m_kp = 0.0f;
	float m_ki = 0.0f;
	float m_kd = 0.0f;

	// Max integrated value
	float m_saturation = 100.0f;

	float m_previousError = 0.0f;
	float m_sommeError = 0.0f;

public:
	// Constructor to initialize PID gains
	PID(float kp, float ki, float kd, float sat) :
		m_kp(kp), m_ki(ki), m_kd(kd), m_saturation(sat)
	{
		m_previousError = 0.0;
		m_sommeError = 0.0;
	}

	// For Euler-based PID
	static float getError(const float& current, const float& target);

	// For Quaternion-based PID
	static Eigen::Quaternionf getError(
			const Eigen::Quaternionf& current,
			const Eigen::Quaternionf& target
			);

	float computePID(const float& error, const float& dt, const bool& integrate);
};



