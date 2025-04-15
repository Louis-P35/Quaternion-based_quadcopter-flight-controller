/*
 * quadcopter.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

// STL
#include <string.h>  // Include for memcpy

// Project
#include "quadcopter.hpp"
#include "logManager.hpp"
#include "PID/controlStrategy.hpp"
#include "Utils/utilsAlgebra.hpp"
//#include "Utils/matrix.hpp"

// screen /dev/tty.usbserial-14220 115200

DroneController::DroneController(
		SPI_HandleTypeDef hspi,
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port,
		UART_HandleTypeDef uart_ext
		)
: m_huart_ext(uart_ext),
  m_complementaryFilter(),
  m_ahrs(&m_complementaryFilter),
  m_ahrs3(&m_kalmanFilter)
{

}



/*
 * Called once at the beginning of the software
 */
void DroneController::mainSetup()
{
	// Setup the serial print
	LogManager::getInstance().setup(m_huart_ext);


	//m_imu.init(AccScale::_8G, GyroScale::DPS500);

	// Setup the IMU (ICM20948)
	icm20948_init(); // Accelerometer & gyroscope
	ak09916_init();  // Magnetometer
	icm20948_gyro_full_scale_select(_500dps);
	icm20948_accel_full_scale_select(_8g);
}


/*
 * Called indefinitely in a loop
 * This is the main loop of this software
 */
void DroneController::mainLoop(const double dt)
{
	// Read IMU
	icm20948_gyro_read_dps(&m_gyro);
	icm20948_accel_read_g(&m_accel);
	ak09916_mag_read_uT(&m_mag);

	char pBuffer[256];
	int numBytes = sprintf(pBuffer,
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f\r\n",
		m_accel.x, m_accel.y, m_accel.z,
		m_gyro.x,  m_gyro.y,  m_gyro.z,
		m_mag.x,   m_mag.y,   m_mag.z);
	LogManager::getInstance().serialPrint(pBuffer);

	/*Eigen::Vector3d gyroTmp = Eigen::Vector3d(0.1, 0.5, -0.1);
	Eigen::Vector3d magnetoTmp = Eigen::Vector3d(0.0, 1.0, 0.0);
	//Eigen::Vector3d accTmp = Eigen::Vector3d(0.1145, -0.9839, 0.1369);
	//Eigen::Vector3i accTmpR = Eigen::Vector3i(m_imu.m_rawAcc.m_vect[0], m_imu.m_rawAcc.m_vect[1], m_imu.m_rawAcc.m_vect[2]);
	Eigen::Quaterniond attitude = m_ahrs.computeAHRS(
			m_imu.m_filteredAcceloremeter,
			m_imu.m_filteredGyro,
			//gyroTmp,
			magnetoTmp,
			dt
			);

	Eigen::Vector3d gyroOffsetDebug(0.0, 0.0, 0.0);

	//Vector<double, 3> test(attitude2.m_q.m_vect[1], attitude2.m_q.m_vect[2], attitude2.m_q.m_vect[3]);
	Eigen::Quaterniond attitude3 = m_ahrs3.computeAHRS(
				m_imu.m_filteredAcceloremeter,
				m_imu.m_filteredGyro,// + gyroOffsetDebug,
				//gyroTmp,
				magnetoTmp,
				dt
				);*/

	//uint32_t sysClockFreq = HAL_RCC_GetSysClockFreq();

	//Vector<int, 3> gyroTmp2((int)(m_imu.m_filteredGyro.m_vect[2]*1000.0), (int)(m_imu.m_rawScaledGyro.m_vect[2]*1000.0), (int)(m_imu.m_gyroOffset.m_vect[2]*1000.0));

	/*double roll, pitch, yaw;
	roll = pitch = yaw = 0.0;
	Eigen::Vector3d vTmp0 = Utils::quaternionToEulerDeg(attitude);
	Vector<int, 3> tmppp((int)vTmp0(0), (int)vTmp0(1), (int)vTmp0(2));

	//attitude2.toEuler(roll, pitch, yaw);
	Eigen::Vector3d vTmp = Utils::quaternionToEulerDeg(attitude2);
	Vector<int, 3> tmppp2((int)vTmp(0), (int)vTmp(1), (int)vTmp(2));

	//attitude3.toEuler(roll, pitch, yaw);
	Vector<int, 3> tmppp3((int)roll, (int)pitch, (int)yaw);*/

	//Vector<double, 3> tm(attitude2.m_q.m_vect[1], attitude2.m_q.m_vect[2], attitude2.m_q.m_vect[3]);
	//LogManager::getInstance().serialPrint(attitude3);
	//Vector<double, 3> tm2(attitude.m_q.m_vect[1], attitude.m_q.m_vect[2], attitude.m_q.m_vect[3]);
	//LogManager::getInstance().serialPrint(m_imu.m_filteredAcceloremeter);
	//LogManager::getInstance().serialPrint(tmppp, tmppp2, tmppp3);


	//LogManager::getInstance().serialPrint(static_cast<int>(sysClockFreq / 1000000));

	//char pBuffer[16] = "Hello world!";
	//HAL_UART_Transmit(&m_huart_ext, (uint8_t*)pBuffer, 12, 100);

	//uint8_t buffer[4];  // Buffer to hold bytes of the float

	// Copy the float into the buffer
	//memcpy(buffer, &m_imu.m_filteredAcceloremeter.m_vect[0], sizeof(float));

	// Transmit the buffer over UART
	//HAL_UART_Transmit(&m_huart_ext, buffer, sizeof(buffer), 100);
}





