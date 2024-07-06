/*
 * utilsAlgebra.cpp
 *
 *  Created on: Jul 5, 2024
 *      Author: Louis
 */

#include "Utils/utilsAlgebra.hpp"

namespace Utils
{


/*
 * Convert quaternion to euler angle in radian (X, Y, Z convention)
 */
Eigen::Vector3d quaternionToEulerRad(const Eigen::Quaterniond& q)
{
	// Convert quaternion to rotation matrix
	const Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

	// Extract Euler angles from rotation matrix
	return rotationMatrix.eulerAngles(0, 1, 2); // XYZ order
}

/*
 * Convert quaternion to euler angle in degree (X, Y, Z convention)
 */
Eigen::Vector3d quaternionToEulerDeg(const Eigen::Quaterniond& q)
{
	const Eigen::Vector3d rad = quaternionToEulerRad(q);

	return rad * RAD_TO_DEGREE;
}


/*
 * Integrate the angular velocity (in RAD/S) over the periode dt (in S).
 * Return the corresponding quaternion.
 */
Eigen::Quaterniond gyroToQuaternion(const Eigen::Vector3d& gyro, const double dt)
{
    // Compute the angle of rotation (magnitude of the angular velocity vector)
    const double angle = gyro.norm() * dt;

    // If the angle is small, return the identity quaternion
    if (angle < 1e-10)
    {
        return Eigen::Quaterniond::Identity();
    }

    // Compute the axis of rotation (normalized angular velocity vector)
    const Eigen::Vector3d axis = gyro.normalized();

    // Create the quaternion representing the incremental rotation
    const Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));

    return q;

	/*if (gyro.norm() > 0.0 && dt != 0.0) // TODO: Norm computed twice, need to optimize that
	{
		// Update the orientation quaternion using half `dt` to correctly integrate angular velocity.
		// This accounts for quaternion properties where each component of angular velocity contributes
		// half as much as it would in a straightforward vector integration,
		// ensuring accurate rotational updates.

		const double half_dt = 0.5 * dt;

		const double w = 1.0;
		const double x = gyro(0) * half_dt;
		const double y = gyro(1) * half_dt;
		const double z = gyro(2) * half_dt;

		Eigen::Quaterniond q(w, x, y, z);
		q.normalize();

		return q;
	}

	// Return identity quaternion if norm is zero
	return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);*/
}


/*
 * Return the quaternion that represent the orientation given by the accelerometer and
 * magnetometer measurment.
 */
Eigen::Quaterniond accMagToQuaternion(const Eigen::Vector3d& acc, const Eigen::Vector3d& mag)
{
    // Normalize accelerometer and magnetometer measurement
    const Eigen::Vector3d normAccel = acc.normalized();
    const Eigen::Vector3d normMag = mag.normalized();

    // Calculate roll and pitch from accelerometer data
    const double roll = atan2(normAccel.y(), normAccel.z());
    const double pitch = atan2(-normAccel.x(), sqrt(normAccel.y() * normAccel.y() + normAccel.z() * normAccel.z()));

    // Calculate yaw from magnetometer data
    const double magX = normMag.x() * cos(pitch) + normMag.y() * sin(roll) * sin(pitch) + normMag.z() * cos(roll) * sin(pitch);
    const double magY = normMag.y() * cos(roll) - normMag.z() * sin(roll);
    const double yaw = atan2(-magY, magX);

    // Create the quaternion from Euler angles
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    q.normalize();

    return q;
}


} // Utils
