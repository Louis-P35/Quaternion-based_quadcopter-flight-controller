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
	static constexpr double m_armLenghtM = 0.1;
	static constexpr double m_kT = 1.0;		// Thrust gain (keep 1.0 for normalized output)
	static constexpr double m_kQ = 0.02;	// Torque gain (Yaw authority over Roll & Pitch)

	double m_powerMotor[N];

public:
	Mixer() {};

	virtual void mixThrustTorque(
			const double& T,
			const double& tx,
			const double& ty,
			const double& tz
			) = 0;

	void clampRescale();
};


/*
 * Mixer class for a X type quad
 */
class XquadMixer : public Mixer<4>
{
private:
	static constexpr double m_a = 1.0 / (4.0 * m_kT);
	static constexpr double m_b = 1.0 / (4.0 * m_kT * m_armLenghtM);
	static constexpr double m_c = 1.0 / (4.0 * m_kQ);

public:
	XquadMixer() {};

	virtual void mixThrustTorque(
				const double& T,
				const double& tx,
				const double& ty,
				const double& tz
				);
};



