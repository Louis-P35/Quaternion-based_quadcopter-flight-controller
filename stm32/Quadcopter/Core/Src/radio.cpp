/*
 * radio.cpp
 *
 *  Created on: May 1, 2025
 *      Author: louis
 */


#include "radio.hpp"
#include "PWM/readRadio.hpp"

Radio::Radio(const double& mid, const double& expo, const double& targetAngleMax)
: m_throttleMID(mid), m_throttleExpo(expo), m_targetAngleMax(targetAngleMax)
{
	// Calculate A and B coefficients for throttle curve
	m_a = (1.0 - m_throttleMID) / pow(m_throttleMID, m_throttleExpo);
	m_b = 1.0 - m_a;
}

bool Radio::readRadioReceiver(const bool& isFlying, const double& dt)
{
	/* Read the radio receiver */
	m_radioChannel1 = PWM_GetPulse(0); // Roll
	m_radioChannel2 = PWM_GetPulse(1); // Pitch
	m_radioChannel3 = PWM_GetPulse(3); // Thrust
	m_radioChannel4 = PWM_GetPulse(2); // Yaw
	//LogManager::getInstance().serialPrint(m_radioChannel1, m_radioChannel2, m_radioChannel3, m_radioChannel4);

	// Handle signal lost
	if (m_radioChannel1 == 0 || m_radioChannel2 == 0 || m_radioChannel3 == 0 || m_radioChannel3 == 0)
	{
		return true;
	}

	m_targetRoll = msToDegree(m_radioChannel1, m_targetAngleMax, true);
	m_targetPitch = msToDegree(m_radioChannel2, m_targetAngleMax, false);
	m_targetYaw = integrateTargetYaw(m_radioChannel4, dt, isFlying);
	const double rawThrottle = (static_cast<double>(m_radioChannel3) - 1000.0) / 10.0;
	m_targetThrust = getThrottle(rawThrottle);

	return false;
}


/*
 * Return the throttle value according to the radio receiver input
 * and the defined throttle curve.
 */
double Radio::getThrottle(const double& radioInput) const
{
	return radioInput * (m_a * pow(radioInput, m_throttleExpo) + m_b);
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
double Radio::integrateTargetYaw(const uint32_t& duration, const double& dt, const bool& isFlying)
{
	constexpr double deadZone = 0.05;
	constexpr double speed = 400.0;

	// Update target yaw only when flying
	if (!isFlying)
	{
		return m_targetYaw;
	}

	double tmp = ((static_cast<double>(duration) - 1000.0) / 1000.0) - 0.5;

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







