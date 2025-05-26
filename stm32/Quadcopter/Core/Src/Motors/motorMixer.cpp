/*
 * motorMixer.cpp
 *
 *  Created on: May 3, 2025
 *      Author: louis
 */

// Include from project
#include "Motors/motorMixer.hpp"

template class Mixer<4>;

template<std::size_t N>
void Mixer<N>::clampRescale()
{
	float minVal = m_powerMotor[0];
	float maxVal = m_powerMotor[0];

	// Find the minimum and maximum power values
	for (std::size_t i = 1; i < N; ++i)
	{
		if (m_powerMotor[i] < minVal)
		{
			minVal = m_powerMotor[i];
		}
		if (m_powerMotor[i] > maxVal)
		{
			maxVal = m_powerMotor[i];
		}
	}

	// If minVal < 0, shift everything
	if (minVal < 0.0f)
	{
		for (std::size_t i = 0; i < N; ++i)
		{
			m_powerMotor[i] -= minVal;
			if (m_powerMotor[i] < 0.0f) // To be sure, because of floating precision
			{
				m_powerMotor[i] = 0.0;
			}
			if (m_powerMotor[i] > maxVal) // update maxVal
			{
				maxVal = m_powerMotor[i];
			}
		}
	}
	if (minVal < 0.0f) // Update minVal. It is 0 if it was negative before
	{
		minVal = 0.0f;
	}

	// At this point there is no negative values

	// Nothing to do if all values are in range [0, 1000]
	if (maxVal <= 1000.0)
	{
		return;
	}

	// Prevent Nan... Should be very rare (all motors equal and out of bounds)
	if (maxVal == minVal)
	{
		for (std::size_t i = 0; i < N; ++i)
		{
			if (m_powerMotor[i] < 0.0f)
			{
				m_powerMotor[i] = 0.0f;
			}
			else if (m_powerMotor[i] > 1000.0f)
			{
				m_powerMotor[i] = 1000.0f;
			}
		}

		return;
	}

	// Linear rescaling
	double scale = 1.0 / (maxVal - minVal);
	for (std::size_t i = 0; i < N; ++i)
	{
		m_powerMotor[i] = (m_powerMotor[i] - minVal) * scale;

		// Clamp because of floating precision
		if (m_powerMotor[i] > 1000.0f)
		{
			m_powerMotor[i] = 1000.0f;
		}
		else if (m_powerMotor[i] < 0.0f)
		{
			m_powerMotor[i] = 0.0f;
		}
	}
}


void XquadMixer::mixThrustTorque(
		const float& T,
		const float& tx,
		const float& ty,
		const float& tz)
{
	m_powerMotor[0] =  m_a*T - m_b*tx - m_b*ty - m_c*tz;   // M1
	m_powerMotor[1] =  m_a*T - m_b*tx + m_b*ty + m_c*tz;   // M2
	m_powerMotor[2] =  m_a*T + m_b*tx + m_b*ty - m_c*tz;   // M3
	m_powerMotor[3] =  m_a*T + m_b*tx - m_b*ty + m_c*tz;   // M4
}


/*
 * Get the battery voltage to estimate the charge.
 * Filter it with a low pass filter.
 */
template<std::size_t N>
float Mixer<N>::getBatteryVoltage() const
{
	static float voltage = m_nominalVoltage;
	float adcVoltage = m_nominalVoltage; // TODO

	// Low pass filter
	voltage = voltage * 0.99f + adcVoltage * 0.01f;

	return voltage;
}

/*
 * Compute the compensation to make the quad fly the same
 * whatever the battery level.
 */
template<std::size_t N>
void Mixer<N>::ComputeVoltageCompensation()
{
	const float voltage = getBatteryVoltage();

	m_voltageCompensation = m_nominalVoltage / voltage;

	// Avoid abusing critically low battery
	if (m_voltageCompensation < 1.0f)
	{
		m_voltageCompensation = 1.0f;
	}
	else if (m_voltageCompensation > 1.5f)
	{
		m_voltageCompensation = 1.5f;
	}
}


template<std::size_t N>
void Mixer<N>::applyVoltageCompensation()
{
	for (std::size_t i = 0; i < N; ++i)
	{
		m_powerMotor[i] *= m_voltageCompensation;
	}
}



