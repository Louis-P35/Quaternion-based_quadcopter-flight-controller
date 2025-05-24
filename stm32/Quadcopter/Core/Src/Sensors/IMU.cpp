/*
 * IMU.cpp
 *
 *  Created on: May 12, 2025
 *      Author: louis
 */


// Includes from project
#include "Sensors/IMU.hpp"
#include "Sensors/icm20948.h"
#include "logManager.hpp"


// So the linker can find them
constexpr float IMU::m_lpf_gyro_gain;
constexpr float IMU::m_lpf_acc_gain;


void IMU::init(
		const float& dataAcquisitionRate/*,
		const float& gyroNotchF0,
		const float& gyroNotchQ,
		const float& accelNotchF0,
		const float& accelNotchQ*/
		)
{
	// Setup the IMU (ICM20948)
	icm20948_init(); // Accelerometer & gyroscope
	//ak09916_init();  // Magnetometer

	// Init filters
	m_lpfAccelX.init(m_lpf_acc_gain);
	m_lpfAccelY.init(m_lpf_acc_gain);
	m_lpfAccelZ.init(m_lpf_acc_gain);

	float params[1] = {75.0f};
	m_lpfBiquadGyroX.init(dataAcquisitionRate, params);
	m_lpfBiquadGyroY.init(dataAcquisitionRate, params);
	m_lpfBiquadGyroZ.init(dataAcquisitionRate, params);

	m_lpfGyroX.init(dataAcquisitionRate, 75.0f);
	m_lpfGyroY.init(dataAcquisitionRate, 75.0f);
	m_lpfGyroZ.init(dataAcquisitionRate, 75.0f);

	m_lpfGyroX2.init(dataAcquisitionRate, 75.0f);
	m_lpfGyroY2.init(dataAcquisitionRate, 75.0f);
	m_lpfGyroZ2.init(dataAcquisitionRate, 75.0f);

	/*m_notchGyroX.init(dataAcquisitionRate, gyroNotchF0, gyroNotchQ);
	m_notchGyroY.init(dataAcquisitionRate, gyroNotchF0, gyroNotchQ);
	m_notchGyroZ.init(dataAcquisitionRate, gyroNotchF0, gyroNotchQ);

	m_notchGyroX2.init(dataAcquisitionRate, gyroNotchF0, gyroNotchQ);
	m_notchGyroY2.init(dataAcquisitionRate, gyroNotchF0, gyroNotchQ);
	m_notchGyroZ2.init(dataAcquisitionRate, gyroNotchF0, gyroNotchQ);

	m_notchAccelX.init(dataAcquisitionRate, accelNotchF0, accelNotchQ);
	m_notchAccelY.init(dataAcquisitionRate, accelNotchF0, accelNotchQ);
	m_notchAccelZ.init(dataAcquisitionRate, accelNotchF0, accelNotchQ);*/
}

void IMU::setAccelOffset(const Vector3<float>& offset)
{
	m_accelOffset = offset;
}

void IMU::setGyroOffset(const Vector3<float>& offset)
{
	m_gyroOffset = offset;
}


/*
 * Read the accelerometer and gyroscope and filter data
 */
void IMU::readAndFilterIMU_gdps()
{
	axises rawGyro;
	axises rawAccel;
	//axises rawMag;

	icm20948_gyro_read_dps(&rawGyro);
	icm20948_accel_read_g(&rawAccel);
	//bool readMag = ak09916_mag_read_uT(&rawMag);

	m_gyroRaw.m_x = rawGyro.y;

	// Cascade Notch filters on gyroscope data
	//rawGyro.x = m_notchGyroX2.apply(m_notchGyroX.apply(rawGyro.x));
	//rawGyro.y = m_notchGyroY2.apply(m_notchGyroY.apply(rawGyro.y));
	//rawGyro.z = m_notchGyroZ2.apply(m_notchGyroZ.apply(rawGyro.z));

	// Cascade LPFs on gyroscope data
	//m_gyro.m_x = m_lpfGyroX2.apply(m_lpfGyroX.apply(rawGyro.x));
	//m_gyro.m_y = m_lpfGyroY2.apply(m_lpfGyroY.apply(rawGyro.y));
	//m_gyro.m_z = m_lpfGyroZ2.apply(m_lpfGyroZ.apply(rawGyro.z));

	m_gyroRaw.m_y = m_lpfGyroY2.apply(m_lpfGyroY.apply(rawGyro.y));


	// Butterworth biquad LPFs on gyroscope data
	m_gyro.m_x = m_lpfBiquadGyroX.apply(rawGyro.x);
	m_gyro.m_y = m_lpfBiquadGyroY.apply(rawGyro.y);
	m_gyro.m_z = m_lpfBiquadGyroZ.apply(rawGyro.z);

	m_gyroRaw.m_z = m_gyro.m_y;

	// Remove gyro's offset
	m_gyro -= m_gyroOffset;

	// Notch filter accelerometer
	//rawAccel.x = m_notchAccelX.apply(rawAccel.x);
	//rawAccel.y = m_notchAccelY.apply(rawAccel.y);
	//rawAccel.z = m_notchAccelZ.apply(rawAccel.z);

	// LPF accelerometer data
	m_accel.m_x = m_lpfAccelX.apply(rawAccel.x);
	m_accel.m_y = m_lpfAccelY.apply(rawAccel.y);
	m_accel.m_z = m_lpfAccelZ.apply(rawAccel.z);

	// Remove accel's offset
	m_accel -= m_accelOffset;
}


/*
 * Find the gyroscope and accelerometer offsets for each axis.
 */
void IMU::gyroAccelCalibration()
{
	Vector3<float> sumGyro = Vector3<float>(0.0f, 0.0f, 0.0f);
	Vector3<float> averageAccel = Vector3<float>(0.0f, 0.0f, 0.0f);

	const int nbIteration = 2000;

	axises gyro;
	axises accel;

	for (int i = 0; i < nbIteration; ++i)
	{
		// Read IMU
		icm20948_gyro_read_dps(&gyro);
		icm20948_accel_read_g(&accel);

		sumGyro += Vector3<float>(gyro.x, gyro.y, gyro.z);
		averageAccel += Vector3<float>(accel.x, accel.y, accel.z);

		HAL_Delay(1);
	}

	m_gyroOffset = sumGyro / static_cast<float>(nbIteration);
	averageAccel /= static_cast<float>(nbIteration);

	// Find the g vector and normalize it
	float norm = averageAccel.norm();
	Vector3<float> g = averageAccel / norm;

	// Remove the gravity vector from the offset
	m_accelOffset = averageAccel - g;

	LogManager::getInstance().serialPrint("GyroOffset: \n\r");
	LogManager::getInstance().serialPrint(m_accelOffset.m_x, m_accelOffset.m_y, m_accelOffset.m_z, 0.0);
	LogManager::getInstance().serialPrint("AccelOffset: \n\r");
	LogManager::getInstance().serialPrint(m_gyroOffset.m_x, m_gyroOffset.m_y, m_gyroOffset.m_z, 0.0);
}





/*
 * Calibrate magnetometer.
 * Print the "center of the sphere" over UART.
 * Run this function and move the IMU in every direction.
 * Function to run once.
 */
void IMU::magnetometerCalibration()
{
	/*axises mag;
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
	}*/
}
