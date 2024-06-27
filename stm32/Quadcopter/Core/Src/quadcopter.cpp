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
#include "Utils/vectorNd.hpp"
#include "Utils/quaternion.hpp"
#include "logManager.hpp"
#include "PID/controlStrategy.hpp"
#include "Utils/matrix.hpp"

// screen /dev/tty.usbserial-14220 115200

DroneController::DroneController(
		SPI_HandleTypeDef hspi,
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port,
		UART_HandleTypeDef uart_ext
		)
: m_imu(hspi, spi_cs_pin, spi_cs_gpio_port), // Initialize m_imu
  m_huart_ext(uart_ext),
  m_complementaryFilter(),
  m_madgwickFilter(),
  m_ahrs(&m_complementaryFilter),
  m_ahrs2(&m_madgwickFilter)
{

}



/*
 * Called once at the beginning of the software
 */
void DroneController::mainSetup()
{
	// Setup the serial print
	LogManager::getInstance().setup(m_huart_ext);

	// Setup the IMU (MPU9250)
	m_imu.init(AccScale::_8G, GyroScale::DPS500);
}


/*
 * Called indefinitely in a loop
 * This is the main loop of this software
 */
void DroneController::mainLoop()
{
	// Read IMU
	m_imu.read_gyro_acc_data();
	m_imu.filter_and_calibrate_data();

	Vector<double, 3> magnetoTmp = Vector<double, 3>(0.0, 0.0, 0.0);
	Quaternion attitude = m_ahrs.computeAHRS(
			m_imu.m_filteredAcceloremeter,
			m_imu.m_filteredGyro,
			magnetoTmp,
			0.1
			);

	Quaternion attitude2 = m_ahrs2.computeAHRS(
			m_imu.m_filteredAcceloremeter,
			m_imu.m_filteredGyro,
			magnetoTmp,
			0.1
			);

	//uint32_t sysClockFreq = HAL_RCC_GetSysClockFreq();

	double roll, pitch, yaw;
	attitude.toEuler(roll, pitch, yaw);
	Vector<int, 3> tmppp((int)roll, (int)pitch, (int)yaw);
	attitude2.toEuler(roll, pitch, yaw);
	Vector<int, 3> tmppp2((int)roll, (int)pitch, (int)yaw);
	LogManager::getInstance().serialPrint(tmppp, tmppp2);
	//LogManager::getInstance().serialPrint(static_cast<int>(sysClockFreq / 1000000));

	//char pBuffer[16] = " ";
	//HAL_UART_Transmit(&m_huart_ext, (uint8_t*)pBuffer, 1, 100);

	//uint8_t buffer[4];  // Buffer to hold bytes of the float

	// Copy the float into the buffer
	//memcpy(buffer, &m_imu.m_filteredAcceloremeter.m_vect[0], sizeof(float));

	// Transmit the buffer over UART
	//HAL_UART_Transmit(&m_huart_ext, buffer, sizeof(buffer), 100);
}





