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
//#include "Utils/matrix.hpp"

// screen /dev/tty.usbserial-14220 115200

#define ROLLPITCH_ATT_KP 1.0f
#define ROLLPITCH_ATT_KI 1.0f
#define ROLLPITCH_ATT_KD 1.0f

#define YAW_ATT_KP 1.0f
#define YAW_ATT_KI 1.0f
#define YAW_ATT_KD 1.0f

DroneController::DroneController(
		SPI_HandleTypeDef hspi,
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port,
		UART_HandleTypeDef uart_ext
		) :
		m_huart_ext(uart_ext)
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
	//ak09916_init();  // Magnetometer

	// Init AHRS
	m_madgwickFilter = MadgwickFilter();

	// fill magnetometer calibration stuff
	//m_incl = 63.0 * DEG_TO_RAD; // ~63° inclination in radians (france)
	//m_B    = 48.0f * 1e-6f;                 // T (France averages around 47–50 μT)
	//m_W    = Eigen::Matrix3d::Identity() * 1e-4;	// EKF magnetometer process noise
	//m_V    = Eigen::Vector3d::Zero(); //Eigen::Vector3f::Ones() * 1.0f;		// EKF magnetometer measurement noise

	// Read accelerometer
	//icm20948_accel_read_g(&m_accel);
	//m_EKF.initWithAcc(m_accel.x, m_accel.y, m_accel.z);
}


/*
 * Called indefinitely in a loop
 * This is the main loop of this software
 */
void DroneController::mainLoop(const double dt)
{
	static bool initialized = false;
	static int ggg = 0;
	static double rrr = 0.0;

	ggg++;
	rrr += dt;

	// Read IMU
	icm20948_gyro_read_dps(&m_gyro);
	icm20948_accel_read_g(&m_accel);
	//bool readMag = ak09916_mag_read_uT(&m_mag);
	//Eigen::Vector3f calibratedMag;

	// Magnetometer calibration correction
	/*if (readMag)
	{
		Eigen::Vector3f magRaw = Eigen::Vector3f(m_mag.x, m_mag.y, m_mag.z);
		// Correct bias to raw data
		calibratedMag = magRaw - m_magBias;
		// Then reflect Y and Z axes to remap them aligned to the accelerometer of the Sparkfun board
		calibratedMag << calibratedMag.x(), -calibratedMag.y(), -calibratedMag.z();

		float norm = calibratedMag.norm();
		// TODO: The bigger std::abs(norm - m_B) the less impact it should have on the EKF (20 - 70)
		if (norm != 0.0f)//norm > 0.1f && std::abs(norm - m_B) < 10.0f) // Reasonable bounds
		{
			calibratedMag *= 1e-6f;
		}
		else
		{
			readMag = false;
		}
	}*/

	if (!initialized)
	{
		gyroAccelCalibration();

		initialized = true;
		return;
	}
	if (!initialized)
	{
		return;
	}

	Eigen::Vector3f gyro;
	Eigen::Vector3f acc;
	gyro << m_gyro.x, m_gyro.y, m_gyro.z;
	acc << m_accel.x, m_accel.y, m_accel.z;
	gyro -= m_gyroOffset;
	acc -= m_accelOffset;

	// Debug print IMU data
	/*char pBuffer3[256];
	sprintf(pBuffer3,
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f\r\n",
		acc.x(), acc.y(), acc.z(),
		gyro.x(),  gyro.y(),  gyro.z(),
		calibratedMag.x(),   calibratedMag.y(),   calibratedMag.z());
	LogManager::getInstance().serialPrint(pBuffer3);
	return;*/


	// AHRS, Madgwick filter
	m_madgwickFilter.compute(
			acc.x(), // Acceleration vector will be normalized
			acc.y(),
			acc.z(),
			gyro.x() * DEGREE_TO_RAD,
			gyro.y() * DEGREE_TO_RAD,
			gyro.z() * DEGREE_TO_RAD,
			dt
			);


	/*if (ggg >= 100)
	{
		ggg = 0;
		double f = 1.0/(rrr/100.0);
		char pBuffer[256];
		sprintf(pBuffer,"%4.7f\n\r", (float)f);
		LogManager::getInstance().serialPrint(pBuffer);
		rrr = 0.0;
	}*/

	// Debug print AHRS result
	char pBuffer[256];
	sprintf(pBuffer,
		"%4.7f, %4.7f, %4.7f, %4.7f\r\n",
		m_madgwickFilter.m_qEst.m_w, m_madgwickFilter.m_qEst.m_x, m_madgwickFilter.m_qEst.m_y, m_madgwickFilter.m_qEst.m_z);
	LogManager::getInstance().serialPrint(pBuffer);
}


void motorsControl(const float& dt)
{
	float targetRoll = 0.0;
	float targetPitch = 0.0;
	float targetYaw = 0.0;

	// Get the target quaternion from the target Euler angles
	Eigen::Quaterniond qTarget; // TODO: Convert _qTarget to Eigen

	// Compute angular errors in the Quaternion space
	// This give the rotation error in the bodyframe
	Eigen::Quaterniond qError;// = PID::getError(g_calibratedAttitude, qTarget);

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


void DroneController::gyroAccelCalibration()
{
	Eigen::Vector3f gyro = Eigen::Vector3f(0.0, 0.0, 0.0);
	Eigen::Vector3f accel = Eigen::Vector3f(0.0, 0.0, 0.0);

	const int nbIteration = 200;

	for (int i = 0; i < nbIteration; ++i)
	{
		// Read IMU
		icm20948_gyro_read_dps(&m_gyro);
		icm20948_accel_read_g(&m_accel);

		gyro += Eigen::Vector3f(m_gyro.x, m_gyro.y, m_gyro.z);
		accel += Eigen::Vector3f(m_accel.x, m_accel.y, m_accel.z);

		HAL_Delay(10);
	}

	m_gyroOffset = gyro / (float)nbIteration;
	accel /= (float)nbIteration;

	// Find the g vector and normalize it
	float norm = accel.norm();
	Eigen::Vector3f g = accel / norm;

	// Remove the gravity vector from the offset
	m_accelOffset = accel - g;
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
		float B = (magMax - magMin).norm() * 0.5f;

		// Print result
		char pBuffer[256];
		sprintf(pBuffer,
			"%4.4f, %4.4f, %4.4f, %4.4f\r\n",
			bias.x(), bias.y(), bias.z(), B);
		LogManager::getInstance().serialPrint(pBuffer);
	}
}


