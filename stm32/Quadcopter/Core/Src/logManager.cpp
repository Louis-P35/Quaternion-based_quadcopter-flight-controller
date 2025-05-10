/*
 * logManager.cpp
 *
 *  Created on: Jun 16, 2024
 *      Author: Louis
 */

// STL
#include <stdio.h>
#include <cstring>
#include <string>
#include <algorithm>

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

	numBytes = sprintf(pBuffer, "%d\r\n", val);
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}


/*
 * Write a float on the serial port
 */
void LogManager::serialPrint(const float val, const bool dotAsComma)
{
	std::string tmp = std::to_string(val);
	tmp += "\n\r";

	if (!dotAsComma)
	{
		auto it = std::find_if(tmp.begin(), tmp.end(), [](char c){return c == '.';});
		if (it != tmp.end())
		{
			*it = ',';
		}
	}

	serialPrint((char*)tmp.c_str());
}


void LogManager::serialPrint(const Quaternion<double>& q)
{
	char pBuffer[256];
	sprintf(pBuffer,
		"%4.7f, %4.7f, %4.7f, %4.7f\r\n",
		q.m_w, q.m_x, q.m_y, q.m_z);
	serialPrint(pBuffer);
}

void LogManager::serialPrint(const Quaternion<double>& q1, const Quaternion<double>& q2)
{
	char pBuffer[256];
	sprintf(pBuffer,
		"%4.7f, %4.7f, %4.7f, %4.7f, %4.7f, %4.7f, %4.7f, %4.7f\r\n",
		q1.m_w, q1.m_x, q1.m_y, q1.m_z,
		q2.m_w, q2.m_x, q2.m_y, q2.m_z);
	serialPrint(pBuffer);
}

void LogManager::serialPrint(const uint32_t ch1, const uint32_t ch2, const uint32_t ch3, const uint32_t ch4)
{
	char pBuffer[256];
	sprintf(pBuffer,
		"%d, %d, %d, %d\r\n",
		ch1, ch2, ch3, ch4);
	serialPrint(pBuffer);
}

void LogManager::serialPrint(const double ch1, const double ch2, const double ch3, const double ch4)
{
	char pBuffer[256];
	sprintf(pBuffer,
		"%4.4f, %4.4f, %4.4f, %4.4f\r\n",
		ch1, ch2, ch3, ch4);
	serialPrint(pBuffer);
}

/*
 * Print IMU data
 */
void LogManager::serialPrint(
			const double ax, const double ay, const double az,
			const double gx, const double gy, const double gz,
			const double mx, const double my, const double mz
			)
{
	char pBuffer[256];
	sprintf(pBuffer,
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f, "
		"%7.2f, %7.2f, %7.2f\r\n",
		ax, ay, az,
		gx, gy, gz,
		mx, my, mz
		);
	LogManager::getInstance().serialPrint(pBuffer);
}

/*
 * Write a double on the serial port
 */
void LogManager::serialPrint(const double val, const bool dotAsComma)
{
	serialPrint(static_cast<float>(val), dotAsComma);
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


