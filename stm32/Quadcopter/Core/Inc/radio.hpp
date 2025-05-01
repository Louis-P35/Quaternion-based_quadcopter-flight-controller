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
private:
	double m_throttleMID = 0.0;
	double m_throttleExpo = 0.0;
	double m_a = 0.0;
	double m_b = 0.0;

	double m_targetAngleMax = 20.0;

public:
	double m_targetYaw = 0.0;
	double m_targetPitch = 0.0;
	double m_targetRoll = 0.0;
	double m_targetThrust = 0.0;

	uint32_t m_radioChannel1 = 0;
	uint32_t m_radioChannel2 = 0;
	uint32_t m_radioChannel3 = 0;
	uint32_t m_radioChannel4 = 0;

public:
	Radio(const double& mid, const double& expo, const double& targetAngleMax);

	bool readRadioReceiver(const bool& isFlying, const double& dt);
	double getThrottle(const double& radioInput) const;
	double msToDegree(const uint32_t& duration, const double& amplitudeMax, const bool& invertAxe);
	double integrateTargetYaw(const uint32_t& duration, const double& dt, const bool& isFlying);
};
