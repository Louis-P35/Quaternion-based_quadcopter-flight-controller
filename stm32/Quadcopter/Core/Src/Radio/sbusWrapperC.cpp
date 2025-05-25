/*
 * sbusWrapperC.cpp
 *
 *  Created on: May 25, 2025
 *      Author: louis
 */

// Includes from project
#include "Radio/sbusWrapperC.h"

// Includes from STL
#include <cstring>

uint8_t sbusBuf[SBUS_FRAME_SIZE];
bool g_newFrameReady = false;


/*
 * Copy the received buffer from DMA to a global variable
 */
void copyFrame(uint8_t* pSbusBuf)
{
	if (!pSbusBuf)
	{
		return;
	}

	std::memcpy(sbusBuf, pSbusBuf, SBUS_FRAME_SIZE);
	g_newFrameReady = true;
}

