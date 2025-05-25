/*
 * radio.cpp
 *
 *  Created on: May 1, 2025
 *      Author: louis
 */


// Includes from projects
#include "radio.hpp"
#include "PWM/readRadio.hpp"
#include "logManager.hpp"

//#define DEBUG_NO_RADIO 1

Radio::Radio(
		const float& hoverOffset,
		const float& expo,
		const float& targetAngleMax,
		const float& targetRateMax
		)
: m_throttleHoverOffset(hoverOffset),
  m_throttleExpo(expo),
  m_targetAngleMax(targetAngleMax),
  m_targetRateMax(targetRateMax)
{
	m_radioProtocole.init();
}

bool Radio::readRadioReceiver(const bool& isFlying, const float& dt)
{
	/* Read the radio receiver */
	/*m_radioChannel1 = PWM_GetPulse(0); // Roll
	m_radioChannel2 = PWM_GetPulse(1); // Pitch
	m_radioChannel3 = PWM_GetPulse(2); // Yaw
	m_radioChannel4 = PWM_GetPulse(3); // Thrust*/
	m_radioChannel1 = m_radioProtocole.getChannel(0); // Roll
	m_radioChannel2 = m_radioProtocole.getChannel(1); // Pitch
	m_radioChannel3 = m_radioProtocole.getChannel(2); // Yaw
	m_radioChannel4 = m_radioProtocole.getChannel(3); // Thrust
	//LogManager::getInstance().serialPrint(m_radioChannel1, m_radioChannel2, m_radioChannel3, m_radioChannel4);
	// 1070 - 1941

#ifndef DEBUG_NO_RADIO
	// Handle signal lost
	if (m_radioChannel1 == 0 || m_radioChannel2 == 0 || m_radioChannel3 == 0 || m_radioChannel4 == 0)
	{
		m_signalLost = true;
		return true;
	}
	m_signalLost = false;
#else
	m_radioChannel1 = 1500;
	m_radioChannel2 = 1500;
	m_radioChannel3 = 2000;
	m_radioChannel4 = 1000;
	m_signalLost = false;
#endif

	// Compute target thrust
	const float rawThrottle = (static_cast<float>(m_radioChannel4) - m_rawThrustRadioMin) / (m_rawThrustRadioMax - m_rawThrustRadioMin);
	m_targetThrust = getThrottle(rawThrottle);

	// Compute target angles
	constexpr float angleDeadZone = 1.0f;
	m_targetRoll = msToDegree(m_radioChannel1, m_targetAngleMax, true, angleDeadZone);
	m_targetPitch = msToDegree(m_radioChannel2, m_targetAngleMax, false, angleDeadZone);
	m_targetYaw = integrateTargetYaw(m_radioChannel3, dt, true, isFlying);

	// Compute target rates
	constexpr float rateDeadZone = 4.0f;
	m_targetRateRoll = msToDegree(m_radioChannel1, m_targetRateMax, true, rateDeadZone);
	m_targetRatePitch = msToDegree(m_radioChannel2, m_targetRateMax, false, rateDeadZone);
	m_targetRateYaw = msToDegree(m_radioChannel3, m_targetRateMax / 5.0f, true, rateDeadZone);

	return false;
}


/*
 * Return the throttle value according to the radio receiver input
 * and the defined throttle curve.
 */
float Radio::getThrottle(float radioInput) const
{
	if (radioInput < 0.0f)
	{
		radioInput = 0.0f;
	}
	else if (radioInput > 1.0f)
	{
		radioInput = 1.0f;
	}

	// The thrust is proportional to the square of the propeller velocity.
	// So we compute the radio input square root to make the thrust proportional
	// to the radio stick.
	float linearized = std::sqrt(radioInput);

	// From there the expo function give more precision close to the ground
	float expoPart = (1.0f - m_throttleExpo) * linearized + m_throttleExpo * linearized * linearized * linearized;
	//LogManager::getInstance().serialPrint(expoPart);

	// Hover point
	float out = m_throttleHoverOffset + expoPart * (1.0f - m_throttleHoverOffset);
	//LogManager::getInstance().serialPrint(out);
	if (out < 0.0f)
	{
		out = 0.0f;
	}
	else if (out > 1.0f)
	{
		out = 1.0f;
	}

	// Totally shut down motors if stick is all the way down
	if (expoPart < 0.01f)
	{
		out = 0.0f;
	}

	return out * 1000.0f;
}


/*
 * Method to convert microseconds to angle
 */
float Radio::msToDegree(
		const uint32_t& duration,
		const float& amplitudeMax,
		const bool& invertAxe,
		const float deadZone
		)
{
	float tmp = (((static_cast<float>(duration) - 1000.0f) / 1000.0f) * (amplitudeMax * 2.0f)) - amplitudeMax;

	// Hysteresis to have a stable zero
	if (tmp > -deadZone && tmp < deadZone)
	{
		tmp = 0.0f;
	}
	else if (tmp > 0.0)
	{
		tmp -= deadZone;
	}
	else
	{
		tmp += deadZone;
	}

	const float retVal = (tmp < -amplitudeMax) ? (-amplitudeMax) : (tmp > amplitudeMax) ? amplitudeMax : tmp;

	return invertAxe ? -retVal : retVal;
}


/*
 * Method to integrate yaw command
 */
float Radio::integrateTargetYaw(const uint32_t& duration, const float& dt, const bool& invertAxe, const bool& isFlying)
{
	constexpr float deadZone = 0.05f;
	constexpr float speed = 400.0f;

	// Update target yaw only when flying
	if (!isFlying)
	{
		return m_targetYaw;
	}

	float tmp = ((static_cast<float>(duration) - 1000.0f) / 1000.0f) - 0.5f;

	if (invertAxe)
	{
		tmp = -tmp;
	}

	// Hysteresis to have a stable zero
	if (tmp > -deadZone && tmp < deadZone)
	{
		tmp = 0.0f;
	}
	else if (tmp > 0.0f)
	{
		tmp -= deadZone;
	}
	else
	{
		tmp += deadZone;
	}

	// Integrate target yaw
	m_targetYaw += tmp * speed * dt;

	// Clamp value to [-180;+180]
	if (m_targetYaw > 180.0f)
	{
		m_targetYaw -= 360.0f;
	}
	else if (m_targetYaw < -180.0f)
	{
		m_targetYaw += 360.0f;
	}

	return m_targetYaw;
}







