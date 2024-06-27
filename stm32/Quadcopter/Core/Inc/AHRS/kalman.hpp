/*
 * kalman.hpp
 *
 *  Created on: Jun 24, 2024
 *      Author: Louis
 */

#pragma once

//#include <Eigen/Dense>

// Project
#include "AHRS/ahrs.hpp"


/*
 * Extended Kalman filter for attitude estimation.
 * Fuse accelerometer, gyroscope and magnetometer data.
 * Work with quaternions.
 */
class AttitudeExtKalman : public IFilter
{

};
