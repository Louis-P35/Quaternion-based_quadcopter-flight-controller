/*
 * mtf01WrapperC.cpp
 *
 *  Created on: Jun 23, 2025
 *      Author: louis
 */

// Includes from project
#include "Sensors/mtf01WrapperC.h"
#include "Sensors/mtf01.hpp"
#include "main.h"

// Includes from STL
#include <cstring>


static void* g_mtf01Instance = NULL;

#include "stm32h7xx_hal.h" // debug
extern UART_HandleTypeDef huart6; // debug

/*
 * DMA Buffer with Cache Coherency Support
 *
 * This buffer requires 32-byte alignment and cache invalidation before reading
 * to prevent cache coherency issues with DMA on STM32H7.
 *
 * For detailed explanation of why this is necessary, see the comment block
 * in Radio/sbusWrapperC.cpp (search for "DMA Buffer and Cache Coherency")
 */
// Place the DMA receive buffer into AXI-SRAM (D2) so the DMA engine can write to it.
// The default DTCM section is not accessible by DMA, causing the buffer to remain zeroed.
// Buffer size is 256 (next multiple of 32 above MTF01_FRAME_SIZE=255) for proper cache alignment
uint8_t mtf01Buf[256] __attribute__((aligned(32))) __attribute__((section(".axisram_bss")));
uint8_t mtf01BufCopy[MTF01_FRAME_SIZE];
size_t mtf01BufLen = 0;
bool g_mtf01NewFrameReady = false;


/*
 * Store the Mtf01 class instance
 */
void mtf01WrapperSetInstance(void* pInstance)
{
    g_mtf01Instance = pInstance;
}


/*
 * Copy the received buffer from DMA to a global variable
 * Param dmaPos: The DMA is in circular mode, so we need to know where it start and stop writing in the buffer.
 */
void mtf01CopyFrame(const size_t dmaPos)
{
	static size_t oldPos = 0;
	static size_t maxLen = 0;

	static int cnter = 0;
	cnter++;

	if (dmaPos == oldPos)
	{
		return;
	}

	// CRITICAL: Invalidate D-Cache before reading the DMA buffer
	// This forces the CPU to read fresh data from RAM instead of stale cached data
	// (See detailed explanation in Radio/sbusWrapperC.cpp)
	SCB_InvalidateDCache_by_Addr((uint32_t*)mtf01Buf, sizeof(mtf01Buf));

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

	// Call C++ method
	if (g_mtf01Instance)
	{
		for (size_t i = 0; i < dataLength; ++i)
		{
			// g_mtf01Instance is a void* (lost the inheritance informations), so we need to cast it to the
			// last type of the hierarchy (Mtf01*) so the class inheritance hierarchy can be reconstructed.
			static_cast<Mtf01*>(g_mtf01Instance)->handleByte(mtf01BufCopy[i]);
		}
	}
}


