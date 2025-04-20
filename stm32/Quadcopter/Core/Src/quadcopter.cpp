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

	// fill magnetometer calibration stuff
	m_incl = 63.0f * DEG_TO_RAD; // ~63° inclination in radians (france)
	m_B    = 48.0f;                 // μT (France averages around 47–50 μT)
	m_W    = Eigen::Matrix3f::Identity() * 1e-4f;	// EKF magnetometer process noise
	m_V    = Eigen::Vector3f::Zero(); //Eigen::Vector3f::Ones() * 1.0f;		// EKF magnetometer measurement noise

	// Read accelerometer
	icm20948_accel_read_g(&m_accel);
	m_EKF.initWithAcc(m_accel.x, m_accel.y, m_accel.z); // Norm must not be 0
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
	bool readMag = ak09916_mag_read_uT(&m_mag);

	// Magnetometer calibration correction
	if (readMag)
	{
		Eigen::Vector3f magRaw = Eigen::Vector3f(m_mag.x, m_mag.y, m_mag.z);
		Eigen::Vector3f m_calibrated = magRaw - m_magBias;
		m_mag.x = m_calibrated.x();
		m_mag.y = m_calibrated.y();
		m_mag.z = m_calibrated.z();
		//m_mag.x /= 50.0f;
		//m_mag.y /= 50.0f;
		//m_mag.z /= 50.0f;
		float norm = m_calibrated.norm();
		char pBuffer2[256];
		sprintf(pBuffer2,
			"%4.2f\r\n",
			norm);
		LogManager::getInstance().serialPrint(pBuffer2);
	}

	// Debug print IMU data
	/*char pBuffer3[256];
	sprintf(pBuffer3,
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f\r\n",
		m_accel.x, m_accel.y, m_accel.z,
		m_gyro.x,  m_gyro.y,  m_gyro.z,
		m_mag.x,   m_mag.y,   m_mag.z);
	LogManager::getInstance().serialPrint(pBuffer3);
	return;*/

	/*static float gyroX = 0.0f;
	static float gyroY = 0.0f;
	static float gyroZ = 0.0f;

	gyroX += m_gyro.x * dt;
	gyroY += m_gyro.y * dt;
	gyroZ += m_gyro.z * dt;
	char pBuffer2[256];
	sprintf(pBuffer2,
		"%4.2f, %4.2f, %4.2f\r\n",
		gyroX, gyroY, gyroZ);
	LogManager::getInstance().serialPrint(pBuffer2);*/

	/*char pBuffer2[256];
	sprintf(pBuffer2,
		"%4.2f, %4.2f, %4.2f\r\n",
		m_accel.x, m_accel.y, m_accel.z);
	LogManager::getInstance().serialPrint(pBuffer2);*/

	// AHRS
	m_EKF.predict(dt);
	m_EKF.correctGyr(m_gyro.x, m_gyro.y, m_gyro.z);
	m_EKF.correctAcc(m_accel.x, m_accel.y, m_accel.z);
	if (readMag)
	{
		m_EKF.correctMag(m_mag.x, m_mag.y, m_mag.z, m_incl, m_B, m_W, m_V);
	}
	m_EKF.reset();

	// get attitude as roll, pitch, yaw
	//float roll, pitch, yaw;
	//m_EKF.getAttitude(roll, pitch, yaw);
	// Debug print AHRS result
	/*char pBuffer[256];
	sprintf(pBuffer,
		"%7.2f, %7.2f, %7.2f, %2.7f\r\n",
		roll, pitch, yaw, dt);
	LogManager::getInstance().serialPrint(pBuffer);*/

	// or quaternion
	IMU_EKF::Quaternion<float> q = m_EKF.getAttitude();

	// Debug print AHRS result
	//char pBuffer[256];
	//sprintf(pBuffer,
	//	"%2.7f, %2.7f, %2.7f, %2.7f\r\n",
	//	q[IMU_EKF::QuaternionIndex::w], q[IMU_EKF::QuaternionIndex::v1], q[IMU_EKF::QuaternionIndex::v2], q[IMU_EKF::QuaternionIndex::v3]);
	//LogManager::getInstance().serialPrint(pBuffer);


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


/*
 * Calibrate magnetometer.
 * Print the "center of the sphere" over UART.
 * Run this function and move the IMU in every direction.
 * Function to run once.
 */
void magnetometerCalibration()
{
	axises mag;
	Eigen::Vector3f magMin( 1e6,  1e6,  1e6);
	Eigen::Vector3f magMax(-1e6, -1e6, -1e6);

	while(1)
	{
		HAL_Delay(10);

		bool readMag = ak09916_mag_read_uT(&mag);
		if (!readMag)
		{
			continue;
		}

		Eigen::Vector3f m = Eigen::Vector3f(mag.x, mag.y, mag.z);
		magMin = magMin.cwiseMin(m);
		magMax = magMax.cwiseMax(m);

		// Compute of the center of the sphere
		Eigen::Vector3f bias = (magMax + magMin) * 0.5f;

		// Print result
		char pBuffer[256];
		sprintf(pBuffer,
			"%4.4f, %4.4f, %4.4f\r\n",
			bias.x(), bias.y(), bias.z());
		LogManager::getInstance().serialPrint(pBuffer);
	}
}


