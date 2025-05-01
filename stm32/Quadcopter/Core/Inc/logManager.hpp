/*
 * logManager.hpp
 *
 *  Created on: Jun 16, 2024
 *      Author: Louis
 */

#pragma once

// Driver
#include "stm32h7xx_hal.h"

// Project
#include "Utils/quaternion.hpp"


/*
 * Singleton class allowing to access the UART
 * to print logs from everywhere.
 * Not using the Singleton from the STL because it use
 * dynamic allocation that is not supported on this microcontroler target.
 */
class LogManager
{
public:
	// Handle of the UART
	UART_HandleTypeDef m_huart;

private:
	LogManager() {};


public:
	// Delete copy constructor and assignment operator
	LogManager(const LogManager&) = delete;
	LogManager& operator=(const LogManager&) = delete;

	// Delete move constructor and assignment operator
	LogManager(LogManager&&) = delete;
	LogManager& operator=(LogManager&&) = delete;

	// Return the only class instance
	static LogManager& getInstance();

	void setup(const UART_HandleTypeDef& huart);

	// Write data to the the serial port
	void serialPrint(const int val);
	void serialPrint(const float val);
	void serialPrint(const double val);
	void serialPrint(char* pVal);
	void serialPrint(const Quaternion& q);
	void serialPrint(
			const double ax, const double ay, const double az,
			const double gx, const double gy, const double gz,
			const double mx, const double my, const double mz
			);
	void serialPrint(const uint32_t ch1, const uint32_t ch2, const uint32_t ch3, const uint32_t ch4);
	void serialPrint(const double ch1, const double ch2, const double ch3, const double ch4);
};



