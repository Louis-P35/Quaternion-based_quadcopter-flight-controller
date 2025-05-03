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

Radio::Radio(const double& hoverOffset, const double& expo, const double& targetAngleMax)
: m_throttleHoverOffset(hoverOffset), m_throttleExpo(expo), m_targetAngleMax(targetAngleMax)
{

}

bool Radio::readRadioReceiver(const bool& isFlying, const double& dt)
{
	/* Read the radio receiver */
	m_radioChannel1 = PWM_GetPulse(0); // Roll
	m_radioChannel2 = PWM_GetPulse(1); // Pitch
	m_radioChannel3 = PWM_GetPulse(2); // Yaw
	m_radioChannel4 = PWM_GetPulse(3); // Thrust
	//LogManager::getInstance().serialPrint(m_radioChannel1, m_radioChannel2, m_radioChannel3, m_radioChannel4);
	// 1070 - 1941

	// Handle signal lost
	if (m_radioChannel1 == 0 || m_radioChannel2 == 0 || m_radioChannel3 == 0 || m_radioChannel4 == 0)
	{
		m_signalLost = true;
		return true;
	}
	m_signalLost = false;

	m_targetRoll = msToDegree(m_radioChannel1, m_targetAngleMax, true);
	m_targetPitch = msToDegree(m_radioChannel2, m_targetAngleMax, false);
	m_targetYaw = integrateTargetYaw(m_radioChannel3, dt, true, isFlying);
	const double rawThrottle = (static_cast<double>(m_radioChannel4) - m_rawThrustRadioMin) / (m_rawThrustRadioMax - m_rawThrustRadioMin);
	m_targetThrust = getThrottle(rawThrottle);

	return false;
}


/*
 * Return the throttle value according to the radio receiver input
 * and the defined throttle curve.
 */
double Radio::getThrottle(double radioInput) const
{
	if (radioInput < 0.0)
	{
		radioInput = 0.0;
	}
	else if (radioInput > 1.0)
	{
		radioInput = 1.0;
	}

	// The thrust is proportional to the square of the propeller velocity.
	// So we compute the radio input square root to make the thrust proportional
	// to the radio stick.
	double linearized = std::sqrt(radioInput);

	// From there the expo function give more precision close to the ground
	double expoPart = (1.0 - m_throttleExpo) * linearized + m_throttleExpo * linearized * linearized * linearized;
	//LogManager::getInstance().serialPrint(expoPart);

	// Hover point
	double out = m_throttleHoverOffset + expoPart * (1.0 - m_throttleHoverOffset);
	//LogManager::getInstance().serialPrint(out);
	if (out < 0.0)
	{
		out = 0.0;
	}
	else if (out > 1.0)
	{
		out = 1.0;
	}

	// Totally shut down motors if stick is all the way down
	if (expoPart < 0.01)
	{
		out = 0.0;
	}

	return out;
}


/*
 * Method to convert microseconds to angle
 */
double Radio::msToDegree(const uint32_t& duration, const double& amplitudeMax, const bool& invertAxe)
{
	constexpr double deadZone = 1.0;

	double tmp = (((static_cast<double>(duration) - 1000.0) / 1000.0) * (amplitudeMax * 2.0)) - amplitudeMax;

	// Hysteresis to have a stable zero
	if (tmp > -deadZone && tmp < deadZone)
	{
		tmp = 0.0;
	}
	else if (tmp > 0.0)
	{
		tmp -= deadZone;
	}
	else
	{
		tmp += deadZone;
	}

	const double retVal = (tmp < -amplitudeMax) ? (-amplitudeMax) : (tmp > amplitudeMax) ? amplitudeMax : tmp;

	return invertAxe ? -retVal : retVal;
}


/*
 * Method to integrate yaw command
 */
double Radio::integrateTargetYaw(const uint32_t& duration, const double& dt, const bool& invertAxe, const bool& isFlying)
{
	constexpr double deadZone = 0.05;
	constexpr double speed = 400.0;

	// Update target yaw only when flying
	if (!isFlying)
	{
		return m_targetYaw;
	}

	double tmp = ((static_cast<double>(duration) - 1000.0) / 1000.0) - 0.5;

	if (invertAxe)
	{
		tmp = -tmp;
	}

	// Hysteresis to have a stable zero
	if (tmp > -deadZone && tmp < deadZone)
	{
		tmp = 0.0;
	}
	else if (tmp > 0.0)
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
	if (m_targetYaw > 180.0)
	{
		m_targetYaw -= 360.0;
	}
	else if (m_targetYaw < -180.0)
	{
		m_targetYaw += 360.0;
	}

	return m_targetYaw;
}







