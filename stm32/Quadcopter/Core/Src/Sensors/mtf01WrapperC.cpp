/*
 * mtf01WrapperC.cpp
 *
 *  Created on: Jun 23, 2025
 *      Author: louis
 */

// Includes from project
#include "Sensors/mtf01WrapperC.h"
#include "Sensors/mtf01.hpp"

// Includes from STL
#include <cstring>


static void* g_mtf01Instance = NULL;

#include "stm32h7xx_hal.h" // debug
extern UART_HandleTypeDef huart6; // debug

// Place the DMA receive buffer into AXI-SRAM (D2) so the DMA engine can write to it.
// The default DTCM section is not accessible by DMA, causing the buffer to remain zeroed.
uint8_t mtf01Buf[MTF01_FRAME_SIZE] __attribute__((section(".axisram_bss")));
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
		// debug
		auto  uart  = huart6.Instance;
		DMA_Stream_TypeDef* dma = reinterpret_cast<DMA_Stream_TypeDef*>(huart6.hdmarx->Instance);
		uint32_t cr   = dma->CR;    // OK
		uint32_t ndtr = dma->NDTR;  // OK
		unsigned long isr = (unsigned long)uart->ISR;
		USART_TypeDef *usart = USART6;
		if (usart->ISR & USART_ISR_ORE)
		{
		    cr = cr;
		}
		// end debug

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


