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
		const double& hoverOffset,
		const double& expo,
		const double& targetAngleMax,
		const double& targetRateMax
		)
: m_throttleHoverOffset(hoverOffset),
  m_throttleExpo(expo),
  m_targetAngleMax(targetAngleMax),
  m_targetRateMax(targetRateMax)
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
	const double rawThrottle = (static_cast<double>(m_radioChannel4) - m_rawThrustRadioMin) / (m_rawThrustRadioMax - m_rawThrustRadioMin);
	m_targetThrust = getThrottle(rawThrottle);

	// Compute target angles
	constexpr double angleDeadZone = 1.0;
	m_targetRoll = msToDegree(m_radioChannel1, m_targetAngleMax, true, angleDeadZone);
	m_targetPitch = msToDegree(m_radioChannel2, m_targetAngleMax, false, angleDeadZone);
	m_targetYaw = integrateTargetYaw(m_radioChannel3, dt, true, isFlying);

	// Compute target rates
	constexpr double rateDeadZone = 4.0;
	m_targetRateRoll = msToDegree(m_radioChannel1, m_targetRateMax, true, rateDeadZone);
	m_targetRatePitch = msToDegree(m_radioChannel2, m_targetRateMax, false, rateDeadZone);
	m_targetRateYaw = msToDegree(m_radioChannel3, m_targetRateMax / 5.0, true, rateDeadZone);

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
double Radio::msToDegree(
		const uint32_t& duration,
		const double& amplitudeMax,
		const bool& invertAxe,
		const double deadZone
		)
{
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







