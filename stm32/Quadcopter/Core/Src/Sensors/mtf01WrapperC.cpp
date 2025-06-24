/*
 * mtf01WrapperC.cpp
 *
 *  Created on: Jun 23, 2025
 *      Author: louis
 */

// Includes from project
#include "Sensors/mtf01WrapperC.h"

// Includes from STL
#include <cstring>


// Place the DMA receive buffer into AXI-SRAM (D2) so the DMA engine can write to it.
// The default DTCM section is not accessible by DMA, causing the buffer to remain zeroed.
uint8_t mtf01Buf[MTF01_FRAME_SIZE] __attribute__((section(".axisram_bss")));
uint8_t mtf01BufCopy[MTF01_FRAME_SIZE];
size_t mtf01BufLen = 0;
bool g_mtf01NewFrameReady = false;


/*
 * Copy the received buffer from DMA to a global variable
 * Param dmaPos: The DMA is in circular mode, so we need to know where it start and stop writing in the buffer.
 */
void mtf01CopyFrame(const size_t dmaPos)
{
	static size_t oldPos = 0;
	static size_t maxLen = 0;

	if (dmaPos == oldPos)
	{
		return;
	}

	size_t dataLength = 0;

	// Handle data between oldPos and dmaPos
	if (dmaPos > oldPos)
	{
		dataLength = dmaPos - oldPos;
		std::memcpy(mtf01BufCopy, &mtf01Buf[oldPos], dataLength);
	}
	else
	{
		// Wrapped
		size_t sizeFirstPart = sizeof(mtf01Buf) - oldPos;
		dataLength = (sizeof(mtf01Buf) - oldPos) + dmaPos;

		std::memcpy(mtf01BufCopy, &mtf01Buf[oldPos], sizeFirstPart);
		std::memcpy(&mtf01BufCopy[sizeFirstPart], mtf01Buf, dmaPos);
	}

	mtf01BufLen = dataLength;
	oldPos = dmaPos;

	if (dataLength > maxLen)
	{
		maxLen = dataLength;
	}

	//g_mtf01NewFrameReady = true;
}


