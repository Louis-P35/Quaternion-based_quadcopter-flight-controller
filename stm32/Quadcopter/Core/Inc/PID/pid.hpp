/*
 * pid.hpp
 *
 *  Created on: Jun 17, 2024
 *      Author: Louis
 */

#pragma once

// Include from project
#include "Utils/quaternion.hpp"


struct PIDConf
{
	double m_kp = 0.0;
	double m_ki = 0.0;
	double m_kd = 0.0;
	double m_saturation = 0.0;
	double m_outMin = 0.0;
	double m_outMax = 0.0;
};


/*
 * PID class
 */
class PID
{
private:
	// PID coefficients
	PIDConf m_conf;

	double m_previousError = 0.0;
	double m_sommeError = 0.0;

public:
	// Constructor to initialize PID gains
	PID()
	{
		m_previousError = 0.0;
		m_sommeError = 0.0;
	}

	// For Euler-based PID
	static double getError(const double& current, const double& target);

	// For Quaternion-based PID
	static Quaternion<double> getError(const Quaternion<double>& current, const Quaternion<double>& target);

	double computePID(const double& error, const double& dt, const bool& integrate);
};


/*
 * PID that can be chained
 */
class PIDBlock : public PID
{
public:
	double m_measure = 0.0;
	double m_target = 0.0;
	double m_output = 0.0;

	PIDBlock() : PID() {}

	void run(double dt);
};



