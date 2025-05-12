/*
 * pid.hpp
 *
 *  Created on: Jun 17, 2024
 *      Author: Louis
 */

#pragma once

// Include from project
#include "Utils/quaternion.hpp"

enum class DerivativeMode
{
	OnError,
	OnMeasurement,
	NoDerivative
};


class PIDConf
{
protected:
	float m_kp = 0.0f;
	float m_ki = 0.0f;
	float m_kd = 0.0f;
	float m_saturation = 1000.0f;	// Inf
	float m_maxD = 1000.0;			// Inf
	float m_outMin = -1000.0f;		// Inf
	float m_outMax = 1000.0f;		// Inf
	float m_maxDpercent = 1.0f;

	DerivativeMode m_derivativeMode = DerivativeMode::OnError;

public:
	PIDConf() = default;

	void setPIDcoeffs(const float& kp, const float& ki, const float& kd)
	{
		m_kp = kp;
		m_ki = ki;
		m_kd = kd;
	};

	void setBoundcoeffs(const float& sat, const float& outMin, const float& outMax, const float& maxDpercent)
	{
		m_saturation = sat;
		m_outMin = outMin;
		m_outMax = outMax;
		m_maxDpercent = maxDpercent;
	};

	void setDerivativeMode(const DerivativeMode& derivativeMode)
	{
		m_derivativeMode = derivativeMode;
	};
};


/*
 * PID class
 */
class PID : public PIDConf
{
//private:
public:
	float m_previousError = 0.0f;
	float m_previousMeasure = 0.0f;
	float m_sommeError = 0.0f;

public:
	// Constructor to initialize PID gains
	PID()
	{
		m_previousError = 0.0;
		m_previousMeasure = 0.0;
		m_sommeError = 0.0;
	}

	// For Euler-based PID
	static float getError(const float& current, const float& target);

	// For Quaternion-based PID
	static Quaternion<float> getError(const Quaternion<float>& current, const Quaternion<float>& target);

	float computePID(const float& error, const float& measure, const float& dt, const bool& integrate);
};


/*
 * PID that can be chained
 */
class PIDBlock : public PID
{
public:
	float m_measure = 0.0f;
	float m_target = 0.0f;
	float m_output = 0.0f;

	PIDBlock() : PID() {}

	void run(const float& dt);
};



