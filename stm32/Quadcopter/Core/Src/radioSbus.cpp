/*
 * radioSbus.cpp
 *
 *  Created on: May 24, 2025
 *      Author: louis
 */

// Includes from project
#include "Radio/radioSbus.hpp"
#include "logManager.hpp"

// Includes from HAL
#include "stm32h7xx_hal.h"

// Includes from STL
#include <string>

extern uint8_t sbusBufCopy[];
extern bool g_newFrameReady;
extern UART_HandleTypeDef huart6;


constexpr int SbusParser::m_nbChannel;

void SbusParser::init()
{
	// Too soon to initilize uart6 here
}


/*
 * Decode the Sbus buffer (25 bytes)
 * First byte is the start byte
 * Last byte is flags
 * The 23 'middle' bytes are data (16 integers of 11 bits)
 */
bool SbusParser::parseSbusFrame()
{
	if (!g_newFrameReady)
	{
		return false;
	}

	// Verify the start‐byte
	if (sbusBufCopy[0] != 0x0F)
	{
		return false;
	}

	// Example buffer:
	// byte1 = a b c d e f g h   (bit7…bit0)
	// byte2 = i j k l m n o p   (bit7…bit0)
	// byte3 = q r s t u v w x   (bit7…bit0)
	// Decoding:
	// channel 0: n o p a b c d e f g h
	// channel 1: s t u v w x i j k l m


	for (uint8_t i = 0; i < m_nbChannel; ++i)
	{
		uint16_t offset = i*11;
		uint16_t byteIndex = offset / 8;
		uint16_t bitShift = offset % 8;

		uint32_t temp = (static_cast<uint32_t>(sbusBufCopy[byteIndex + 1]) |
                (static_cast<uint32_t>(sbusBufCopy[byteIndex + 2]) << 8) |
                (static_cast<uint32_t>(sbusBufCopy[byteIndex + 3]) << 16)) >> bitShift;

		m_channel[i] = temp & 0x7FF;

		//m_channel[i] = (static_cast<uint32_t>(sbusBufCopy[byteIndex+2]) << (8 - bitShiftOffset) |
		//		(static_cast<uint32_t>(sbusBufCopy[byteIndex+1]) >> bitShiftOffset)) & 0x7FF;
	}

	// Flags (byte 23)
	m_frameLost = (sbusBufCopy[23] & (1 << 2)) != 0;
	m_failsafe  = (sbusBufCopy[23] & (1 << 3)) != 0;

	g_newFrameReady = false;

	return true;
}



uint32_t SbusParser::getChannel(const int& x) const
{
	if (x >= m_nbChannel || x < 0)
	{
		return 0;
	}

	return m_channel[x];
}


void SbusParser::print() const
{
	std::string str;

	for (int i = 0; i < m_nbChannel; ++i)
	{
		if (i != 0)
		{
			str += ", ";
		}
		str += std::to_string(getChannel(i));
	}
	str += "\r\n";

	LogManager::getInstance().serialPrint(str.c_str());
}




