/*
 * setPoints.hpp
 *
 *  Created on: Jun 8, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Utils/quaternion.hpp"


/*
 * Setpoint class, is is the input of the PIDs controller.
 * This interface the radio or autonomous commands.
 * All setpoints are defined here, the controller must pick the right one according to the control strategy.
 */
template<typename T>
class SetPoint
{
public:
	// Rates setpoint
	T m_targetRateRoll = T(0);
	T m_targetRatePitch = T(0);
	T m_targetRateYaw = T(0);

	// Angle setpoint
	Quaternion<T> m_targetQuaternion = Quaternion<T>::iddentity();

	// Position setpoint
	// TODO

public:
	SetPoint() = default;

	void setRateRoll(const T& rateRoll) {m_targetRateRoll = rateRoll;};
	void setRatePitch(const T& ratePitch) {m_targetRatePitch = ratePitch;};
	void setRateYaw(const T& rateYaw) {m_targetRateYaw = rateYaw;};

	void setAngle(const Quaternion<T>& quat) {m_targetQuaternion = quat;};
};






