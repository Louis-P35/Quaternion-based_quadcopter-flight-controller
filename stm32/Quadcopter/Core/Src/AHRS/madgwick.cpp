/*
 * madgwick.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: louis
 */

#include "AHRS/madgwick.hpp"
#include "Utils/quaternion.hpp"


#define GYRO_MEAN_ERROR M_PI * (5.0 / 180.0) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#define BETA std::sqrt(3.0/4.0) * GYRO_MEAN_ERROR
#define BETA_MAG 0.75   // Magnetic yaw correction gain — tuned independently of BETA


// Gyroscope Angular Velocity components are in Radians per Second
// Accelerometer componets will be normalized
template<class T>
void MadgwickFilter<T>::compute(
	const T& ax,
	const T& ay,
	const T& az,
	const T& gx,
	const T& gy,
	const T& gz,
	const T& dt
	)
{
	//Variables and constants
	Quaternion<T> q_est_prev = m_qEst;
	Quaternion<T> q_est_dot = Quaternion<T>(0.0, 0.0, 0.0, 0.0);
	Quaternion<T> q_a = Quaternion<T>(0.0, ax, ay, az);    // equation (24) raw acceleration values, needs to be normalized

	T F_g [3] = {0.0};		// equation(15/21/25) objective function for gravity
	T J_g [3][4] = {0.0};	// jacobian matrix for gravity

	Quaternion<T> gradient = Quaternion<T>(0.0, 0.0, 0.0, 0.0);

	/* Integrate angluar velocity to obtain position in angles. */
	Quaternion<T> q_w = Quaternion<T>(0.0, gx, gy, gz);	// equation (10), places gyroscope readings in a quaternion
	// the real component is zero, which the Madgwick uses to simplfy quat. mult.


	q_w *= 0.5;				// equation (12) dq/dt = (1/2)q*w
	q_w = q_est_prev * q_w;	// equation (12)

	/* NOTE
	* Page 10 states equation (40) substitutes equation (13) into it. This seems false, as he actually
	* substitutes equation (12), q_se_dot_w, not equation (13), q_se_w.
	*
	* // quat_scalar(&q_w, deltaT);               // equation (13) integrates the angles velocity to position
	* // quat_add(&q_w, q_w, q_est_prev);         // addition part of equation (13)
	*/

	/* Compute the gradient by multiplying the jacobian matrix by the objective function. This is equation 20.
	The Jacobian matrix, J, is a 3x4 matrix of partial derivatives for each quaternion component in the x y z axes
	The objective function, F, is a 3x1 matrix for x y and z.
	To multiply these together, the inner dimensions must match, so use J'.
	I calculated "by hand" the transpose of J, so I will be using "hard coordinates" to get those values from J.
	The matrix multiplcation can also be done hard coded to reduce code.

	Note: it is possible to compute the objective function with quaternion multiplcation functions, but it does not take into account the many zeros that cancel terms out and is not optimized like the paper shows
	*/

	q_a.normalize();              // normalize the acceleration quaternion to be a unit quaternion
	//Compute the objective function for gravity, equation(15), simplified to equation (25) due to the 0's in the acceleration reference quaternion
	F_g[0] = 2.0*(q_est_prev.m_x * q_est_prev.m_z - q_est_prev.m_w * q_est_prev.m_y) - q_a.m_x;
	F_g[1] = 2.0*(q_est_prev.m_w * q_est_prev.m_x + q_est_prev.m_y* q_est_prev.m_z) - q_a.m_y;
	F_g[2] = 2.0*(0.5 - q_est_prev.m_x * q_est_prev.m_x - q_est_prev.m_y * q_est_prev.m_y) - q_a.m_z;

	//Compute the Jacobian matrix, equation (26), for gravity
	J_g[0][0] = -2.0 * q_est_prev.m_y;
	J_g[0][1] =  2.0 * q_est_prev.m_z;
	J_g[0][2] = -2.0 * q_est_prev.m_w;
	J_g[0][3] =  2.0 * q_est_prev.m_x;

	J_g[1][0] = 2.0 * q_est_prev.m_x;
	J_g[1][1] = 2.0 * q_est_prev.m_w;
	J_g[1][2] = 2.0 * q_est_prev.m_z;
	J_g[1][3] = 2.0 * q_est_prev.m_y;

	J_g[2][0] = 0.0;
	J_g[2][1] = -4.0 * q_est_prev.m_x;
	J_g[2][2] = -4.0 * q_est_prev.m_y;
	J_g[2][3] = 0.0;

	// now computer the gradient, equation (20), gradient = J_g'*F_g
	gradient.m_w = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
	gradient.m_x = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
	gradient.m_y = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
	gradient.m_z = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

	// Normalize the gradient, equation (44)
	gradient.normalize();

	/* This is the sensor fusion part of the algorithm.
	Combining Gyroscope position angles calculated in the beginning, with the quaternion orienting the accelerometer to gravity created above.
	Noticably this new quaternion has not be created yet, I have only calculated the gradient in equation (19).
	Madgwick however uses assumptions with the step size and filter gains to optimize the gradient descent,
	  combining it with the sensor fusion in equations (42-44).
	He says the step size has a var alpha, which he assumes to be very large.
	This dominates the previous estimation in equation (19) to the point you can ignore it.
	Eq. 36 has the filter gain Gamma, which is related to the step size and thus alpha. With alpha being very large,
	  you can make assumptions to simplify the fusion equatoin of eq.36.
	Combining the simplification of the gradient descent equation with the simplification of the fusion equation gets you eq.
	41 which can be subdivided into eqs 42-44.
	*/
	gradient *= BETA;             // multiply normalized gradient by beta
	q_est_dot = q_w - gradient;        // subtract above from q_w, the integrated gyro quaternion
	q_est_dot *= dt;
	m_qEst = q_est_prev + q_est_dot;     // Integrate orientation rate to find position
	m_qEst.normalize();                 // normalize the orientation of the estimate
												//(shown in diagram, plus always use unit quaternions for orientation)
}


/*
Retreive the Euler angle
This is subject to gimbal lock
*/
template<class T>
void MadgwickFilter<T>::getEulerAngle(T& roll, T& pitch, T& yaw)
{
	m_qEst.toEuler(roll, pitch, yaw);
}


/*
 * Madgwick MARG filter — accel + gyro + magnetometer.
 *
 * Implements equations 29 (F_b), 33 (J_b), and 34 (combined gradient) from
 * "An efficient orientation filter for inertial and inertial/magnetic sensor
 * arrays", S.O.H. Madgwick, 2010.
 *
 * Frame: NWU (North-West-Up) — gravity reference [0,0,1] → Z up.
 * Earth X = magnetic North, Earth Y = West (right-hand rule: North×West=Up).
 * The magnetic reference is not hardcoded: it is derived from the current
 * estimate by rotating the measured field into Earth frame and discarding its
 * West component (by=0). This means yaw tracks magnetic North automatically
 * without any prior knowledge of declination.
 *
 * Typical usage:
 *   - compute()     called at IMU rate (e.g. 1 kHz) every cycle
 *   - computeMARG() called at mag rate (e.g. 100 Hz) when new data is ready;
 *     it replaces one compute() call for that cycle
 */
template<class T>
void MadgwickFilter<T>::computeMARG(
    const T& ax, const T& ay, const T& az,
    const T& gx, const T& gy, const T& gz,
    const T& mx, const T& my, const T& mz,
    const T& dt)
{
    // ── Validate magnetometer ─────────────────────────────────────────────────
    Quaternion<T> q_m(0, mx, my, mz);
    if (q_m.norm() < static_cast<T>(1e-6))
    {
        compute(ax, ay, az, gx, gy, gz, dt);
        return;
    }
    q_m.normalize();

    // ── Normalise accelerometer ───────────────────────────────────────────────
    Quaternion<T> q_a(0, ax, ay, az);
    q_a.normalize();

    const Quaternion<T> q_prev = m_qEst;
    const T qw = q_prev.m_w, qx = q_prev.m_x;
    const T qy = q_prev.m_y, qz = q_prev.m_z;

    // ── Reference magnetic field in Earth frame ───────────────────────────────
    // Rotate the normalised mag measurement into Earth frame: h = q ⊗ m̂ ⊗ q*
    const Quaternion<T> h = q_prev * q_m * q_prev.conjugate();
    // Keep only bx (horizontal magnitude) and bz (vertical component),
    // setting by = 0 to make yaw observable from the horizontal mag vector.
    const T bx = std::sqrt(h.m_x * h.m_x + h.m_y * h.m_y);
    const T bz = h.m_z;

    // ── Gravity objective function F_g (eq. 25) ───────────────────────────────
    // F_g = q* ⊗ [0,0,0,1] ⊗ q − â  (gravity reference [0,0,1] → Z up)
    const T Fg0 = 2*(qx*qz - qw*qy)                       - q_a.m_x;
    const T Fg1 = 2*(qw*qx + qy*qz)                       - q_a.m_y;
    const T Fg2 = 2*(static_cast<T>(0.5) - qx*qx - qy*qy) - q_a.m_z;

    // ── Magnetic objective function F_b (eq. 29) ──────────────────────────────
    // F_b = q* ⊗ [0,bx,0,bz] ⊗ q − m̂
    const T Fb0 = 2*bx*(static_cast<T>(0.5)-qy*qy-qz*qz) + 2*bz*(qx*qz - qw*qy) - q_m.m_x;
    const T Fb1 = 2*bx*(qx*qy - qw*qz)                   + 2*bz*(qw*qx + qy*qz) - q_m.m_y;
    const T Fb2 = 2*bx*(qw*qy + qx*qz)                   + 2*bz*(static_cast<T>(0.5)-qx*qx-qy*qy) - q_m.m_z;

    // ── Combined gradient: J_g^T·F_g + J_b^T·F_b (eq. 20 / 34) ─────────────
    // Sub-gradients are normalised and scaled independently: BETA governs
    // roll/pitch convergence, BETA_MAG governs yaw convergence.
    // Normalising together would let a large gravity error dilute the magnetic
    // correction in flight and prevent BETA_MAG from being tuned independently.
    Quaternion<T> gradG;
    gradG.m_w = (-2*qy)*Fg0 + ( 2*qx)*Fg1;
    gradG.m_x = ( 2*qz)*Fg0 + ( 2*qw)*Fg1 + (-4*qx)*Fg2;
    gradG.m_y = (-2*qw)*Fg0 + ( 2*qz)*Fg1 + (-4*qy)*Fg2;
    gradG.m_z = ( 2*qx)*Fg0 + ( 2*qy)*Fg1;

    Quaternion<T> gradB;
    gradB.m_w = (-2*bz*qy)*Fb0                  + (-2*bx*qz + 2*bz*qx)*Fb1 + ( 2*bx*qy)*Fb2;
    gradB.m_x = ( 2*bz*qz)*Fb0                  + ( 2*bx*qy + 2*bz*qw)*Fb1 + ( 2*bx*qz - 4*bz*qx)*Fb2;
    gradB.m_y = (-4*bx*qy - 2*bz*qw)*Fb0       + ( 2*bx*qx + 2*bz*qz)*Fb1 + ( 2*bx*qw - 4*bz*qy)*Fb2;
    gradB.m_z = (-4*bx*qz + 2*bz*qx)*Fb0       + (-2*bx*qw + 2*bz*qy)*Fb1 + ( 2*bx*qx)*Fb2;

    const T nG = gradG.norm();
    const T nB = gradB.norm();
    if (nG > static_cast<T>(1e-6)) gradG *= static_cast<T>(BETA) / nG;
    // Proportional correction when |gradB| < 1 (steady-state regime):
    // avoids applying a fixed-magnitude BETA_MAG×dt correction in a random direction
    // when F_b is tiny (residual noise after convergence).
    // When |gradB| >= 1 (large initial error): normalised — fast convergence unchanged.
    if (nB > static_cast<T>(1e-6)) gradB *= static_cast<T>(BETA_MAG) / std::max(nB, static_cast<T>(1.0));

    const Quaternion<T> gradient = gradG + gradB;

    // ── Sensor fusion (eq. 42–44) ─────────────────────────────────────────────

    Quaternion<T> q_w(0, gx, gy, gz);
    q_w *= static_cast<T>(0.5);
    q_w = q_prev * q_w;

    Quaternion<T> q_dot = q_w - gradient;
    q_dot *= dt;
    m_qEst = q_prev + q_dot;
    m_qEst.normalize();
}


// Explicite instanciation for float
template class MadgwickFilter<float>;


