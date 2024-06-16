/*
 * logManager.cpp
 *
 *  Created on: Jun 16, 2024
 *      Author: Louis
 */

// STL
#include <stdio.h>
#include <cstring>

// Project
#include "logManager.hpp"


/*
 * This method memorize the UART handler
 * needed to print over UART.
 */
void LogManager::setup(const UART_HandleTypeDef& huart)
{
	m_huart = huart;
}


/*
 * Return the only class instance.
 * Create a local static instance.
 */
LogManager& LogManager::getInstance()
{
	// Local static instance
	static LogManager instance;

	return instance;
}



/*
 * Write a integer on the serial port
 */
void LogManager::serialPrint(const int val)
{
	char pBuffer[256];
	int numBytes;

	numBytes = sprintf(pBuffer, "%d", val);
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}


/*
 * Write a float on the serial port
 */
void LogManager::serialPrint(const float val)
{
	char pBuffer[256];
	int numBytes;

	/*
	For some reason sprintf %f doesn't work here.
	Printing interger and decimal part as two integers
	instead.
	*/

	// Extract the integer part
	int integer = static_cast<int>(val);
	// Extract the decimal part
	int decimal = abs(static_cast<int>((val - static_cast<float>(integer)) * 1000.0f));

	numBytes = sprintf(pBuffer, "%d.%d", integer, decimal);
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}


/*
 * Write a double on the serial port
 */
void LogManager::serialPrint(const double val)
{
	serialPrint(static_cast<float>(val));
}

/*
 * Write a string on the serial port
 */
void LogManager::serialPrint(char* pVal)
{
	// Safety check
	if (pVal == nullptr)
	{
		return;
	}

	// Get the size in byte of the string
	const int len = strlen(pVal);

	// Send it over UART
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pVal), len, 100);
}


