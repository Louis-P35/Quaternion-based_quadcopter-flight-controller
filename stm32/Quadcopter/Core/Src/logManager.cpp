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

	numBytes = sprintf(pBuffer, "%d\n", val);
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}


/*
 * Write a float on the serial port
 */
void LogManager::serialPrint(const float val)
{
	char pBuffer[256];
	int numBytes;

	numBytes = sprintf(pBuffer, "%f", val);
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}

/*
 * Write a vector on the serial port
 */
void LogManager::serialPrint(const Eigen::Vector3d& v)
{
	char pBuffer[256];
	int numBytes;

	int x = static_cast<int>(v(0) * 1000.0);
	int y = static_cast<int>(v(1) * 1000.0);
	int z = static_cast<int>(v(2) * 1000.0);


	numBytes = sprintf(pBuffer, "%d,%d,%d\n", x, y, z);
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}


void LogManager::serialPrint(const Eigen::Vector3i& v)
{
	char pBuffer[256];
	int numBytes;


	numBytes = sprintf(pBuffer, "%d,%d,%d\n", v(0), v(1), v(2));
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}

void LogManager::serialPrint(
		const Eigen::Vector3d& v1,
		const Eigen::Vector3d& v2,
		const Eigen::Vector3d& v3
		)
{
	char pBuffer[256];
	int numBytes;


	numBytes = sprintf(
			pBuffer,
			"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			v1(0),
			v1(1),
			v1(2),
			v2(0),
			v2(1),
			v2(2),
			v3(0),
			v3(1),
			v3(2)
			);

	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}




void LogManager::serialPrint(const Eigen::Quaterniond& q)
{
	int w = static_cast<int>(q.w()*10000);
	int x = static_cast<int>(q.x()*10000);
	int y = static_cast<int>(q.y()*10000);
	int z = static_cast<int>(q.z()*10000);

	char pBuffer[256];
	int numBytes;


	numBytes = sprintf(pBuffer, "%d,%d,%d,%d\n", w, x, y, z);
	HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
}

void LogManager::serialPrint(const Eigen::Quaterniond& q, const Eigen::Vector3d& v)
{
	int w = static_cast<int>(q.w()*10000);
	int x = static_cast<int>(q.x()*10000);
	int y = static_cast<int>(q.y()*10000);
	int z = static_cast<int>(q.z()*10000);

	int vx = static_cast<int>(v(0)*10000);
	int vy = static_cast<int>(v(1)*10000);
	int vz = static_cast<int>(v(2)*10000);

	char pBuffer[256];
	int numBytes;


	numBytes = sprintf(pBuffer, "%d,%d,%d,%d,%d,%d,%d", w, x, y, z, vx, vy, vz);
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


