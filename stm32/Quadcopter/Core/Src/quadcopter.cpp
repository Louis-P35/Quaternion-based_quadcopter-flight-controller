/*
 * quadcopter.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */


// Project
#include "quadcopter.hpp"
#include "Utils/vectorNd.hpp"
#include "Utils/quaternion.hpp"
#include "logManager.hpp"



DroneController::DroneController(
		SPI_HandleTypeDef hspi,
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port,
		UART_HandleTypeDef uart_ext
		)
: m_imu(hspi, spi_cs_pin, spi_cs_gpio_port), // Initialize m_imu
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

	LogManager::getInstance().serialPrint(m_imu.m_filteredAcceloremeter.m_vect[0]);

	char pBuffer[16] = " ";
	HAL_UART_Transmit(&m_huart_ext, (uint8_t*)pBuffer, 1, 100);
}





