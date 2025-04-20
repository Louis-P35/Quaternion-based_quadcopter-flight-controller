/*
 * utilsAlgebra.hpp
 *
 *  Created on: Jul 5, 2024
 *      Author: Louis
 */

#pragma once

#include "Utils/Eigen/Dense"

#define DEGREE_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEGREE (180.0f / M_PI)

namespace Utils
{

// Convert quaternion to euler angle in radian (X, Y, Z convention)
Eigen::Vector3d quaternionToEulerRad(const Eigen::Quaterniond& q);

// Convert quaternion to euler angle in degree (X, Y, Z convention)
Eigen::Vector3d quaternionToEulerDeg(const Eigen::Quaterniond& q);

// Return a quaternion representing the small rotation of the angular velocity during dt
Eigen::Quaterniond gyroToQuaternion(const Eigen::Vector3d& gyro, const double dt);

// Return the quaternion that represent the orientation given by the accelerometer and
// magnetometer measurment.
Eigen::Quaterniond accMagToQuaternion(const Eigen::Vector3d& acc, const Eigen::Vector3d& mag);

}



