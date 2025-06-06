/*
 * madgwick.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: louis
 */

#include "AHRS/madgwick.hpp"
#include "Utils/quaternion.hpp"


#define GYRO_MEAN_ERROR M_PI * (5.0 / 180.0) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#define BETA std::sqrt(3.0/4.0) * GYRO_MEAN_ERROR    //*from paper*


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


// Explicite instanciation for float
template class MadgwickFilter<float>;


