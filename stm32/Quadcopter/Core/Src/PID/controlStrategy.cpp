/*
 * controlStrategy.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: louis
 */

#include "PID/controlStrategy.hpp"



PIDs_out ControlStrategy::controlLoop(const float dt, const bool ctrlLoop)
{
	// Filter out illegal combinations
	if (m_controlMode == POSHOLD && m_flightMode == ACRO)
	{
		m_flightMode = STAB;
	}

	// First loop (low frequency, 50hz)
	if (ctrlLoop)
	{
		switch (m_controlMode)
		{
		case ControlMode::RADIO:
			// Radio -> angles
			m_angleLoop[0].m_measure = 0.0f; // TODO: From quaternion
			m_angleLoop[1].m_measure = 0.0f; // TODO: From quaternion
			m_angleLoop[2].m_measure = 0.0f; // TODO: From quaternion

			m_angleLoop[0].m_target = 0.0f; // TODO: Radio roll
			m_angleLoop[1].m_target = 0.0f; // TODO: Radio pitch
			m_angleLoop[2].m_target = 0.0f; // TODO: Radio yaw

			// Radio -> rates
			m_rateLoop[0].m_measure = 0.0f; // TODO: From gyro
			m_rateLoop[1].m_measure = 0.0f; // TODO: From gyro
			m_rateLoop[2].m_measure = 0.0f; // TODO: From gyro

			m_rateLoop[0].m_target = 0.0f; // TODO: Radio roll
			m_rateLoop[1].m_target = 0.0f; // TODO: Radio pitch
			m_rateLoop[2].m_target = 0.0f; // TODO: Radio yaw
			break;

		case ControlMode::POSHOLD:
			// v(x), v(y), v(z) -> angles

			m_linearVelocityLoop[0].m_measure = 0.0f; // TODO: vx
			m_linearVelocityLoop[1].m_measure = 0.0f; // TODO: vy
			m_linearVelocityLoop[2].m_measure = 0.0f; // TODO: vz

			m_linearVelocityLoop[0].m_target = 0.0f;
			m_linearVelocityLoop[1].m_target = 0.0f;
			m_linearVelocityLoop[2].m_target = 0.0f;

			for (int i = 0; i < 3; ++i)
			{
				m_linearVelocityLoop[i].run(dt);
				m_angleLoop[i].m_target = m_linearVelocityLoop[i].m_output; // Chain
			}
			break;
		}
	}

	// Second loop (high frequency, 2khz)
	switch (m_flightMode)
	{
	case StabilizationMode::STAB:
		for (int i = 0; i < 3; ++i)
		{
			m_angleLoop[i].run(dt);
		}
		return PIDs_out{m_angleLoop[0].m_output, m_angleLoop[1].m_output, m_angleLoop[2].m_output};

	case StabilizationMode::ACRO:
		for (int i = 0; i < 3; ++i)
		{
			m_rateLoop[i].run(dt);
		}
		return PIDs_out{m_rateLoop[0].m_output, m_rateLoop[1].m_output, m_rateLoop[2].m_output};

	case StabilizationMode::HORIZON:
		// TODO
		break;
	}

	return PIDs_out{0.0f, 0.0f, 0.0f}; // Should never happened
}
