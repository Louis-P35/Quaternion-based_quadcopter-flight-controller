/*
 * mtf01WrapperC.cpp
 *
 *  Created on: Jun 23, 2025
 *      Author: louis
 */

// Includes from project
#include "Sensors/mtf01WrapperC.h"

// Place the DMA receive buffer into AXI-SRAM (D2) so the DMA engine can write to it.
// The default DTCM section is not accessible by DMA, causing the buffer to remain zeroed.
uint8_t mtf01Buf[MTF01_FRAME_SIZE] __attribute__((section(".axisram_bss")));
uint8_t mtf01BufCopy[MTF01_FRAME_SIZE];
bool g_mtf01NewFrameReady = false;


/*
 * Copy the received buffer from DMA to a global variable
 */
void mtf01CopyFrame()
{
	for (int i = 0; i < MTF01_FRAME_SIZE; ++i)
	{
		mtf01BufCopy[i] = mtf01Buf[i];
	}

	g_mtf01NewFrameReady = true;
}


