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


extern UART_HandleTypeDef huart2;

volatile int uart2TxBusy = 0;

/*
 * This method memorize the UART handler
 * needed to print over UART.
 */
void LogManager::setup()
{

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
	/*char pBuffer[256];
	int numBytes;

	numBytes = sprintf(pBuffer, "%d\r\n", val);
	//HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes, 100);
	HAL_UART_Transmit_DMA(&m_huart, reinterpret_cast<uint8_t*>(pBuffer), numBytes);*/
}


/*
 * Write a float on the serial port
 */
void LogManager::serialPrint(const float val, const bool dotAsComma)
{
	std::string tmp = std::to_string(val);
	tmp += "\r\n";

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

/*
 * Write a float on the serial port
 */
void LogManager::serialPrint(const float val, const float val2, const bool dotAsComma)
{
	std::string tmp = std::to_string(val);
	std::string tmp2 = std::to_string(val2);
	//tmp += "\r\n";

	if (!dotAsComma)
	{
		auto it = std::find_if(tmp.begin(), tmp.end(), [](char c){return c == '.';});
		if (it != tmp.end())
		{
			*it = ',';
		}

		auto it2 = std::find_if(tmp2.begin(), tmp2.end(), [](char c){return c == '.';});
		if (it2 != tmp2.end())
		{
			*it2 = ',';
		}
	}

	std::string str = tmp + ";" + tmp2 + "\r\n";
	serialPrint((char*)str.c_str());
}


void LogManager::serialPrint(const Quaternion<float>& q)
{
	char pBuffer[256];
	sprintf(pBuffer,
		"%4.7f, %4.7f, %4.7f, %4.7f\r\n",
		q.m_w, q.m_x, q.m_y, q.m_z);
	serialPrint(pBuffer);
}

void LogManager::serialPrint(const Quaternion<float>& q1, const Quaternion<float>& q2)
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

void LogManager::serialPrint(const float ch1, const float ch2, const float ch3, const float ch4)
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
			const float ax, const float ay, const float az,
			const float gx, const float gy, const float gz,
			const float mx, const float my, const float mz
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
void LogManager::serialPrint(const char* pVal)
{
	// Safety check
	if (pVal == nullptr || uart2TxBusy)
	{
		return;
	}

	// Get the size in byte of the string
	const int len = strlen(pVal);

	// Invalider le cache (spécifique STM32H7)
	//SCB_CleanDCache_by_Addr((uint32_t*)m_txBuf, len);
	uint32_t alignedLen = ((len + 31) / 32) * 32;
	SCB_CleanDCache_by_Addr((uint32_t*)m_txBuf, alignedLen);

	// Copier les données dans le buffer
	memcpy(m_txBuf, pVal, len > 256 ? 256 : len);

	// Workaround because the uart over dma stay in a non ready state after a transmission for some reason
	// HAL_UART_Transmit_DMA() work once and then keep returning HAL_BUSY
	// This is a crappy workaround but...
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
	huart2.gState = HAL_UART_STATE_READY;
	huart2.hdmatx->State = HAL_DMA_STATE_READY;

	// Send it over UART
	HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart2, reinterpret_cast<uint8_t*>(m_txBuf), len);
	if (st == HAL_OK)
	{
		uart2TxBusy = 1;
	}
	//HAL_UART_Transmit(&m_huart, reinterpret_cast<uint8_t*>(m_txBuf), len, 100);
}


