/*
 * radio.hpp
 *
 *  Created on: May 1, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Utils/quaternion.hpp"
#include "Radio/radioSbus.hpp"

// Includes from STL
#include <stdint.h>

class Radio
{
public:
	// Thrust params
	float m_throttleHoverOffset = 0.0f;
	float m_throttleExpo = 0.0f;
	float m_targetAngleMax = 0.0f;
	float m_targetRateMax = 0.0f;


	float m_targetYaw = 0.0f;
	float m_targetPitch = 0.0f;
	float m_targetRoll = 0.0f;
	float m_targetRateYaw = 0.0f;
	float m_targetRatePitch = 0.0f;
	float m_targetRateRoll = 0.0f;
	float m_targetThrust = 0.0f;

	SbusParser m_radioProtocole;
	bool m_signalLost = true;
	uint32_t m_radioChannel1 = 0;
	uint32_t m_radioChannel2 = 0;
	uint32_t m_radioChannel3 = 0;
	uint32_t m_radioChannel4 = 0;

public:
	Radio(const float& mid, const float& expo, const float& targetAngleMax, const float& targetRateMax);

	bool readRadioReceiver(const bool& isFlying, const float& dt);
	float getThrottle(float radioInput) const;
	float msToDegree(
			const uint32_t& duration,
			const float& amplitudeMax,
			const bool& invertAxe,
			const float deadZone
			);
	float integrateTargetYaw(const uint32_t& duration, const float& dt, const bool& invertAxe, const bool& isFlying);
};
