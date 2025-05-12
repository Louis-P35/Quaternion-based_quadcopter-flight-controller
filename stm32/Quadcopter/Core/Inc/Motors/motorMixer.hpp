/*
 * motorMixer.hpp
 *
 *  Created on: May 3, 2025
 *      Author: louis
 */

#pragma once

// Includes from STL
#include <cstddef>


//	    	   FRONT
//              Y- (Pitch)
//               |
//   M2[1] (CW)  | M3[2] (CCW)
//               |
//    -----------+----------- X- (Roll)
//               |
//   M1[0] (CCW) | M4[3] (CW)
//               |
//              Y+
// 			   BACK

// Yaw+: CCW rotation
// Yaw-: CW rotation


/*
 * Convert a “physic” command vector u = [T, tx, ty, tz]
 * 		T: Total target thrust in Newton (N)
 * 		tx, ty, tz: Target torque arount x, y, z axis
 * in normalized power for each motor
 */
template<std::size_t N>
class Mixer
{
public:
	static constexpr float m_armLenghtM = 0.1f;
	// m_kT = 1.0 => 1 unit of power motor = 1 unit of thrust
	static constexpr float m_kT = 1.0f;		// Thrust gain (keep 1.0 for normalized output)
	static constexpr float m_kQ = 0.02f;	// Torque gain (Yaw authority over Roll & Pitch)

	float m_powerMotor[N];

public:
	Mixer() {};

	virtual void mixThrustTorque(
			const float& T,
			const float& tx,
			const float& ty,
			const float& tz
			) = 0;

	void clampRescale();
};


/*
 * Mixer class for a X type quad
 */
class XquadMixer : public Mixer<4>
{
private:
	static constexpr float m_a = 1.0f / (4.0f * m_kT);
	static constexpr float m_b = 1.0f / (4.0f * m_kT * m_armLenghtM);
	static constexpr float m_c = 1.0f / (4.0f * m_kQ);

public:
	XquadMixer() {};

	virtual void mixThrustTorque(
				const float& T,
				const float& tx,
				const float& ty,
				const float& tz
				);
};



