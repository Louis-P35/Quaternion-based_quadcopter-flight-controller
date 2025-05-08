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
	double minVal = m_powerMotor[0];
	double maxVal = m_powerMotor[0];

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
	if (minVal < 0.0)
	{
		for (std::size_t i = 0; i < N; ++i)
		{
			m_powerMotor[i] -= minVal;
			if (m_powerMotor[i] < 0.0) // To be sure, because of floating precision
			{
				m_powerMotor[i] = 0.0;
			}
			if (m_powerMotor[i] > maxVal) // update maxVal
			{
				maxVal = m_powerMotor[i];
			}
		}
	}

	// At this point there is no negative values

	// Nothing to do if all values are in range [0, 1]
	if (maxVal <= 1.0)
	{
		return;
	}

	// Prevent Nan... Should be very rare (all motors equal and out of bounds)
	if (maxVal == minVal)
	{
		for (std::size_t i = 0; i < N; ++i)
		{
			if (m_powerMotor[i] < 0.0)
			{
				m_powerMotor[i] = 0.0;
			}
			else if (m_powerMotor[i] > 1.0)
			{
				m_powerMotor[i] = 1.0;
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
		if (m_powerMotor[i] > 1.0)
		{
			m_powerMotor[i] = 1.0;
		}
		else if (m_powerMotor[i] < 0.0)
		{
			m_powerMotor[i] = 0.0;
		}
	}
}


void XquadMixer::mixThrustTorque(
		const double& T,
		const double& tx,
		const double& ty,
		const double& tz)
{
	m_powerMotor[0] =  m_a*T - m_b*tx - m_b*ty - m_c*tz;   // M1
	m_powerMotor[1] =  m_a*T - m_b*tx + m_b*ty + m_c*tz;   // M2
	m_powerMotor[2] =  m_a*T + m_b*tx + m_b*ty - m_c*tz;   // M3
	m_powerMotor[3] =  m_a*T + m_b*tx - m_b*ty + m_c*tz;   // M4
}
