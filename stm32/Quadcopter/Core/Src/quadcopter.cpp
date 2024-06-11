/*
 * quadcopter.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

#include "quadcopter.hpp"



DroneController::DroneController(SPI_HandleTypeDef hspi, uint16_t spi_cs_pin, GPIO_TypeDef* spi_cs_gpio_port)
: m_imu(hspi, spi_cs_pin, spi_cs_gpio_port) // Initialize m_imu
{

}



/*
 * Called once at the beginning of the software
 */
void DroneController::mainSetup()
{
	// Setup the IMU (MPU9250)
	m_imu.init();
}



/*
 * Called indefinitely in a loop
 * This is the main loop of this software
 */
void DroneController::mainLoop()
{
	// Read IMU
	m_imu.read_gyro_acc_data();
}



