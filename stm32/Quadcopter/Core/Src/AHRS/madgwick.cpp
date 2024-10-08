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
#include "Utils/utilsAlgebra.hpp"


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
Eigen::Quaterniond MadgwickFilter::compute(
		const Eigen::Vector3d& acc,
		const Eigen::Vector3d& gyro,
		const Eigen::Vector3d& magneto,
		const double& dt
		)
{
	// Radian per second conversion
	const Eigen::Vector3d gyroRad = gyro * DEGREE_TO_RAD;

	const Eigen::Quaterniond q_est_prev = m_qEst;
	Eigen::Quaterniond q_est_dot = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
	Eigen::Quaterniond q_a = Eigen::Quaterniond(0.0, acc(0), acc(1), acc(2));

	double F_g[3] = {0}; 		// Objective function for gravity
	double J_g[3][4] = {0}; 	// Jacobian matrix for gravity

	Eigen::Quaterniond gradient = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

	// Integrate angluar velocity to obtain position in angles
	Eigen::Quaterniond q_w = Eigen::Quaterniond(0.0, gyroRad(0), gyroRad(1), gyroRad(2));                   // equation (10), places gyroscope readings in a quaternion
	// the real component is zero, which the Madgwick uses to simplfy quat. mult.


	q_w.w() *= 0.5;
	q_w.x() *= 0.5;
	q_w.y() *= 0.5;
	q_w.z() *= 0.5; // dq/dt = 0.5 * q * omega
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
	F_g[0] = 2.0 * (q_est_prev.x() * q_est_prev.z() - q_est_prev.w() * q_est_prev.y()) - q_a.x();
	F_g[1] = 2.0 * (q_est_prev.w() * q_est_prev.x() + q_est_prev.y() * q_est_prev.z()) - q_a.y();
	F_g[2] = 2.0 * (0.5 - q_est_prev.x() * q_est_prev.x() - q_est_prev.y() * q_est_prev.y()) - q_a.z();

	//Compute the Jacobian matrix for gravity
	J_g[0][0] = -2.0 * q_est_prev.y();
	J_g[0][1] =  2.0 * q_est_prev.z();
	J_g[0][2] = -2.0 * q_est_prev.w();
	J_g[0][3] =  2.0 * q_est_prev.x();

	J_g[1][0] = 2.0 * q_est_prev.x();
	J_g[1][1] = 2.0 * q_est_prev.w();
	J_g[1][2] = 2.0 * q_est_prev.z();
	J_g[1][3] = 2.0 * q_est_prev.y();

	J_g[2][0] = 0.0;
	J_g[2][1] = -4.0 * q_est_prev.x();
	J_g[2][2] = -4.0 * q_est_prev.y();
	J_g[2][3] = 0.0;

	// now compute the gradient, gradient = J_g'*F_g
	gradient.w() = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
	gradient.x() = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
	gradient.y() = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
	gradient.z() = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

	// Normalize the gradient
	gradient.normalize();

	// multiply normalized gradient by beta
	gradient.w() *= BETA;
	gradient.x() *= BETA;
	gradient.y() *= BETA;
	gradient.z() *= BETA;
	// subtract above from q_w, the integrated gyro quaternion
	q_est_dot.w() = q_w.w() - gradient.w();
	q_est_dot.x() = q_w.x() - gradient.x();
	q_est_dot.y() = q_w.y() - gradient.y();
	q_est_dot.z() = q_w.z() - gradient.z();
	q_est_dot.w() *= dt;
	q_est_dot.x() *= dt;
	q_est_dot.y() *= dt;
	q_est_dot.z() *= dt;

	// Integrate orientation rate to find position
	m_qEst.w() = q_est_prev.w() + q_est_dot.w();
	m_qEst.x() = q_est_prev.x() + q_est_dot.x();
	m_qEst.y() = q_est_prev.y() + q_est_dot.y();
	m_qEst.z() = q_est_prev.z() + q_est_dot.z();
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
	// Convert quaternion to rotation matrix
	Eigen::Matrix3d rotationMatrix = m_qEst.toRotationMatrix();

	// Extract Euler angles from rotation matrix
	Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2); // XYZ order
	roll = eulerAngles(0);
	pitch = eulerAngles(1);
	yaw = eulerAngles(2);
}




