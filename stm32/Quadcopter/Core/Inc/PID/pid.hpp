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


struct PIDConf
{
	float m_kp = 0.0f;
	float m_ki = 0.0f;
	float m_kd = 0.0f;
	float m_saturation = 0.0f;
	float m_outMin = 0.0f;
	float m_outMax = 0.0f;
};


/*
 * PID class
 */
class PID
{
private:
	// PID coefficients
	PIDConf m_conf;

	float m_previousError = 0.0f;
	float m_sommeError = 0.0f;

public:
	// Constructor to initialize PID gains
	PID()
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


class PIDBlock : public PID
{
public:
	float m_measure = 0.0f;
	float m_target = 0.0f;
	float m_output = 0.0f;

	PIDBlock() : PID() {}

	inline void run(float dt);
};



