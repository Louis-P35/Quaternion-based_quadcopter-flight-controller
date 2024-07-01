/*
 * madgwick.cpp
 *
 *  Created on: Jun 15, 2024
 *      Author: Louis
 */


// STL
#include <cmath>

// Project
#include "AHRS/madgwick.hpp"


#define GYRO_MEAN_ERROR M_PI * (5.0 / 180.0) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#define BETA sqrt(3.0/4.0) * GYRO_MEAN_ERROR    //*from paper*

/*
Sensor Fusion: The Madgwick filter uses accelerometer, gyroscope and magnetometer
inputs to estimate orientation. The core of the filter is based on forming an optimization
problem that aims to reduce the error between the estimated orientation and the measurement
from each sensor.

Gradient Descent Algorithm: The filter applies a gradient descent algorithm to
solve this optimization problem iteratively. The objective is to find the orientation
that minimizes the error. This error is defined as the difference between the
predicted gravity and magnetic field direction (from the sensor measurements) and
those predicted by the orientation estimate.

Update and Predict: At each step, the filter updates its estimate of the orientation using
the new sensor data. The gyroscope provides very accurate short-term changes in orientation
but drifts over time. The accelerometer and magnetometer provide information to correct
this drift, although they are susceptible to short-term disturbances.
*/


/*
 * Gyroscope Angular Velocity components are in Radians per Second.
 * Accelerometer components will be normalized.
 */
Quaternion MadgwickFilter::compute(
		const Vector<double, 3>& acc,
		const Vector<double, 3>& gyro,
		const Vector<double, 3>& magneto,
		const double& dt
		)
{
	// Radian per second conversion
	Vector<double, 3> gyroRad = gyro * DEGREE_TO_RAD;

	Quaternion q_est_prev = m_qEst;
	Quaternion q_est_dot = Quaternion(0.0, 0.0, 0.0, 0.0);
	Quaternion q_a = Quaternion(0.0, acc.m_vect[0], acc.m_vect[1], acc.m_vect[2]);

	double F_g[3] = {0}; 		// Objective function for gravity
	double J_g[3][4] = {0}; 	// Jacobian matrix for gravity

	Quaternion gradient = Quaternion(0.0, 0.0, 0.0, 0.0);

	// Integrate angluar velocity to obtain position in angles
	Quaternion q_w = Quaternion(0.0, gyroRad.m_vect[0], gyroRad.m_vect[1], gyroRad.m_vect[2]);                   // equation (10), places gyroscope readings in a quaternion
	// the real component is zero, which the Madgwick uses to simplfy quat. mult.


	q_w *= 0.5;				// dq/dt = (1/2)q*w
	q_w = q_est_prev * q_w;

	/*
	Compute the gradient by multiplying the jacobian matrix by the objective function.
	The Jacobian matrix, J, is a 3x4 matrix of partial derivatives for each quaternion component in the x y z axes
	The objective function, F, is a 3x1 matrix for x y and z.
	To multiply these together, the inner dimensions must match, so use J'.
	I calculated "by hand" the transpose of J, so I will be using "hard coordinates" to get those values from J.
	The matrix multiplcation can also be done hard coded to reduce code.

	Note: it is possible to compute the objective function with quaternion multiplcation functions, but it does not
	take into account the many zeros that cancel terms out and is not optimized like the paper shows
	*/

	q_a.normalize();	// normalize the acceleration quaternion to be a unit quaternion
	//Compute the objective function for gravity, simplified due to the 0's in the acceleration reference quaternion
	F_g[0] = 2.0*(q_est_prev.m_q.m_vect[1] * q_est_prev.m_q.m_vect[3] - q_est_prev.m_q.m_vect[0] * q_est_prev.m_q.m_vect[2]) - q_a.m_q.m_vect[1];
	F_g[1] = 2.0*(q_est_prev.m_q.m_vect[0] * q_est_prev.m_q.m_vect[1] + q_est_prev.m_q.m_vect[2]* q_est_prev.m_q.m_vect[3]) - q_a.m_q.m_vect[2];
	F_g[2] = 2.0*(0.5 - q_est_prev.m_q.m_vect[1] * q_est_prev.m_q.m_vect[1] - q_est_prev.m_q.m_vect[2] * q_est_prev.m_q.m_vect[2]) - q_a.m_q.m_vect[3];

	//Compute the Jacobian matrix for gravity
	J_g[0][0] = -2.0 * q_est_prev.m_q.m_vect[2];
	J_g[0][1] =  2.0 * q_est_prev.m_q.m_vect[3];
	J_g[0][2] = -2.0 * q_est_prev.m_q.m_vect[0];
	J_g[0][3] =  2.0 * q_est_prev.m_q.m_vect[1];

	J_g[1][0] = 2.0 * q_est_prev.m_q.m_vect[1];
	J_g[1][1] = 2.0 * q_est_prev.m_q.m_vect[0];
	J_g[1][2] = 2.0 * q_est_prev.m_q.m_vect[3];
	J_g[1][3] = 2.0 * q_est_prev.m_q.m_vect[2];

	J_g[2][0] = 0.0;
	J_g[2][1] = -4.0 * q_est_prev.m_q.m_vect[1];
	J_g[2][2] = -4.0 * q_est_prev.m_q.m_vect[2];
	J_g[2][3] = 0.0;

	// now compute the gradient, gradient = J_g'*F_g
	gradient.m_q.m_vect[0] = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
	gradient.m_q.m_vect[1] = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
	gradient.m_q.m_vect[2] = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
	gradient.m_q.m_vect[3] = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

	// Normalize the gradient
	gradient.normalize();

	/*
	This is the sensor fusion part of the algorithm.
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
	gradient *= BETA;             		// multiply normalized gradient by beta
	q_est_dot = q_w - gradient;     	// subtract above from q_w, the integrated gyro quaternion
	q_est_dot *= dt;
	m_qEst = q_est_prev + q_est_dot;	// Integrate orientation rate to find position
	m_qEst.normalize();                 // normalize the orientation of the estimate
										//(shown in diagram, plus always use unit quaternions for orientation)
	return m_qEst;
}


/*
 * Retreive the Euler angle
 * This is subject to gimbal lock
 */
void MadgwickFilter::getEulerAngle(double& roll, double& pitch, double& yaw) const
{
	m_qEst.toEuler(roll, pitch, yaw);
}




