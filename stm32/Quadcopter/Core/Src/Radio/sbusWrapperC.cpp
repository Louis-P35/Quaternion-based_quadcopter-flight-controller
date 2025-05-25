/*
 * sbusWrapperC.cpp
 *
 *  Created on: May 25, 2025
 *      Author: louis
 */

// Includes from project
#include "Radio/sbusWrapperC.h"

// Place the SBUS DMA receive buffer into AXI-SRAM (D2) so the DMA engine can write to it.
// The default DTCM section is not accessible by DMA, causing the buffer to remain zeroed.
uint8_t sbusBuf[SBUS_FRAME_SIZE] __attribute__((section(".axisram_bss")));
uint8_t sbusBufCopy[SBUS_FRAME_SIZE];
bool g_newFrameReady = false;


/*
 * Copy the received buffer from DMA to a global variable
 */
void copyFrame()
{
	for (int i = 0; i < SBUS_FRAME_SIZE; ++i)
	{
		sbusBufCopy[i] = sbusBuf[i];
	}

	g_newFrameReady = true;
}

