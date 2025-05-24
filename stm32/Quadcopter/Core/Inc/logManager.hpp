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
	alignas(32) uint8_t m_txBuf[256]; // Alignement cache-line

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

	void setup();

	// Write data to the the serial port
	void serialPrint(const int val);
	void serialPrint(const float val, const bool dotAsComma = true);
	void serialPrint(const double val, const bool dotAsComma = true);
	void serialPrint(char* pVal);
	void serialPrint(const Quaternion<float>& q);
	void serialPrint(const Quaternion<float>& q1, const Quaternion<float>& q2);
	void serialPrint(
			const float ax, const float ay, const float az,
			const float gx, const float gy, const float gz,
			const float mx, const float my, const float mz
			);
	void serialPrint(const uint32_t ch1, const uint32_t ch2, const uint32_t ch3, const uint32_t ch4);
	void serialPrint(const float ch1, const float ch2, const float ch3, const float ch4);
	void serialPrint(const float val, const float val2, const bool dotAsComma);
};



