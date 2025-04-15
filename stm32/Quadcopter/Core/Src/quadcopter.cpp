/*
 * quadcopter.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

// Includes from STL
#include <string.h>  // Include for memcpy

// Includes from Project
#include "quadcopter.hpp"
#include "logManager.hpp"
#include "PID/controlStrategy.hpp"
#include "Utils/utilsAlgebra.hpp"
#include "AHRS/Quaternion.h"
//#include "Utils/matrix.hpp"

// screen /dev/tty.usbserial-14220 115200

DroneController::DroneController(
		SPI_HandleTypeDef hspi,
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port,
		UART_HandleTypeDef uart_ext
		)
: m_huart_ext(uart_ext)
{

}



/*
 * Called once at the beginning of the software
 */
void DroneController::mainSetup()
{
	// Setup the serial print
	LogManager::getInstance().setup(m_huart_ext);

	// Setup the IMU (ICM20948)
	icm20948_init(); // Accelerometer & gyroscope
	ak09916_init();  // Magnetometer
	icm20948_gyro_full_scale_select(_500dps);
	icm20948_accel_full_scale_select(_8g);

	// fill magnetometer calibration stuff
	// Winv <<
	// W <<
	// B =
	// incl =

	float ax = 0.0f;
	float ay = 0.0f;
	float az = 1.0f;
	// read accelerometer
	m_EKF.initWithAcc(ax, ay, az); // Norm must not be 0
}


/*
 * Called indefinitely in a loop
 * This is the main loop of this software
 */
void DroneController::mainLoop(const float dt)
{
	// Read IMU
	icm20948_gyro_read_dps(&m_gyro);
	icm20948_accel_read_g(&m_accel);
	ak09916_mag_read_uT(&m_mag);

	// Debug print IMU data
	/*char pBuffer[256];
	sprintf(pBuffer,
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f\r\n",
		m_accel.x, m_accel.y, m_accel.z,
		m_gyro.x,  m_gyro.y,  m_gyro.z,
		m_mag.x,   m_mag.y,   m_mag.z);
	LogManager::getInstance().serialPrint(pBuffer);*/


	// AHRS
	bool readMag = true;
	m_EKF.predict(dt);
	m_EKF.correctGyr(m_gyro.x, m_gyro.y, m_gyro.z);
	m_EKF.correctAcc(m_accel.x, m_accel.y, m_accel.z);
	if (readMag)
	{
		m_EKF.correctMag(m_mag.x, m_mag.y, m_mag.z, m_incl, m_B, m_W, m_V);
	}
	m_EKF.reset();

	// get attitude as roll, pitch, yaw
	float roll, pitch, yaw;
	m_EKF.getAttitude(roll, pitch, yaw);
	// or quaternion
	//IMU_EKF::Quaternion<float> q = filter.getAttitude();

	// Debug print AHRS result
	char pBuffer[256];
	sprintf(pBuffer,
		"%7.2f, %7.2f, %7.2f, %2.7f\r\n",
		roll, pitch, yaw, dt);
	LogManager::getInstance().serialPrint(pBuffer);
}


void motorsControl(const float& dt)
{
	float targetRoll = 0.0f;
	float targetPitch = 0.0f;
	float targetYaw = 0.0f;

	// Get the target quaternion from the target Euler angles
	IMU_EKF::Quaternion<float> _qTarget = IMU_EKF::Quaternion<float>::fromEuler(
		targetRoll * DEGREE_TO_RAD,
		targetPitch * DEGREE_TO_RAD,
		targetYaw * DEGREE_TO_RAD
    	);

	Eigen::Quaternionf qTarget; // TODO: Convert _qTarget to Eigen

	// Compute angular errors in the Quaternion space
	// This give the rotation error in the bodyframe
	Eigen::Quaternionf qError;// = PID::getError(g_calibratedAttitude, qTarget);

	// Use directly the vector part of the error quaternion as inputs of the PIDs
	/*The vector part of the error quaternion (qError.m_x, qError.m_y, qError.m_z)
	is not exactly the angular error in radians for large rotations. It’s an
	approximation based on the small-angle assumption (where sin(θ/2) ≈ θ/2).
	For larger errors, this can lead to inaccuracies.
	Specifically, the vector part is scaled by sin(θ/2), where θ is the rotation angle.
	To get true angular errors, we’d need to compute θ = 2 * acos(qError.m_w) and
	normalize the vector part accordingly*/
	float theta = 2.0f * acos(qError.w()); // Total rotation angle
	float scale = (theta > 1e-6f) ? (theta / sin(theta / 2.0f)) : 2.0f; // Avoid division by zero
	float error_roll = qError.x() * scale;
	float error_pitch = qError.y() * scale;
	float error_yaw = qError.z() * scale;

	// Compute PIDs
}


