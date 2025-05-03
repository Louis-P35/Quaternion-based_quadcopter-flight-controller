/*
 * radio.hpp
 *
 *  Created on: May 1, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Utils/quaternion.hpp"

// Includes from STL
#include <stdint.h>

class Radio
{
public:
	// Thrust params
	static constexpr double m_rawThrustRadioMin = 1070.0;
	static constexpr double m_rawThrustRadioMax = 1941.0;
	double m_throttleHoverOffset = 0.0;
	double m_throttleExpo = 0.0;
	double m_targetAngleMax = 0.0;


	double m_targetYaw = 0.0;
	double m_targetPitch = 0.0;
	double m_targetRoll = 0.0;
	double m_targetThrust = 0.0;

	bool m_signalLost = true;

	uint32_t m_radioChannel1 = 0;
	uint32_t m_radioChannel2 = 0;
	uint32_t m_radioChannel3 = 0;
	uint32_t m_radioChannel4 = 0;

public:
	Radio(const double& mid, const double& expo, const double& targetAngleMax);

	bool readRadioReceiver(const bool& isFlying, const double& dt);
	double getThrottle(double radioInput) const;
	double msToDegree(const uint32_t& duration, const double& amplitudeMax, const bool& invertAxe);
	double integrateTargetYaw(const uint32_t& duration, const double& dt, const bool& invertAxe, const bool& isFlying);
};
