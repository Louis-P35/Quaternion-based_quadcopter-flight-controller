/*
 * blackbox.cpp
 *
 *  Created on: May 14, 2025
 *      Author: louis
 */


// Includes from project
#include "BlackboxSD/blackbox.hpp"
#include "logManager.hpp"

// Includes from STL
#include <cstring>


// External handles
extern SD_HandleTypeDef hsd1; // SDMMC handle
extern DMA_HandleTypeDef hdma_sdmmc1_tx; // DMA handle for SDMMC TX



template<size_t SAMPLE_SIZE_BYTE>
bool Blackbox<SAMPLE_SIZE_BYTE>::init()
{
	// Ensure SDMMC is initialized
	HAL_SD_StateTypeDef ret = HAL_SD_GetState(&hsd1);
	if (HAL_SD_GetState(&hsd1) != HAL_SD_STATE_READY)
	{
		return false;
	}

    // Mount FATFS
    if (f_mount(&m_fs, "0:", 1) != FR_OK)
    {
        return false; // Failed to mount
    }

    return true;
}


template<size_t SAMPLE_SIZE_BYTE>
bool Blackbox<SAMPLE_SIZE_BYTE>::startLogging()
{
    //m_fileName = generateFileName();
    if (f_open(&m_file, m_fileName.c_str(), FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
        return false;
    }

    m_isFileOpen = true;
    m_writeOffset = 0;
    m_currentBuffer = 0;
    m_isWriting = false;

    return true;
}



/*
 * Log sensors data to the SD card.
 * Useful to analyze noise frequencies with FFT.
 * Use double buffering & DMA for fast non-blocking logging (later).
 */
template<size_t SAMPLE_SIZE_BYTE>
void Blackbox<SAMPLE_SIZE_BYTE>::blackBoxLogToSdCard(const uint8_t pBuffer[SAMPLE_SIZE_BYTE])
{
	if (m_currentBuffer != 0) // Temporary, while the DMA is not used yet, so single buffer
	{
		return;
	}

	// Current buffer and index to write to
	//uint8_t* pDest = (m_currentBuffer == 0) ? m_buffer1 : m_buffer2;
	uint8_t* pDest = m_buffer1;

	// Copy data at the right index
	memcpy(pDest + m_writeOffset, pBuffer, SAMPLE_SIZE_BYTE);

	// Increment index for the next write
	m_writeOffset += SAMPLE_SIZE_BYTE;

	// Check if the current buffer is full
	if ((m_writeOffset + SAMPLE_SIZE_BYTE) > bufferSize)
	{
		// Write to the SD card (asynchronously through the DMA)
		//asyncWriteToSdCard(pDest);
		writeToSdCard(pDest);

		// Swap buffers
		m_currentBuffer = 1 - m_currentBuffer;

		// No need to initialize to 0 the new buffer here because it will be write on the SD card
		// only when it will be full again.

		// Reset write offset
		m_writeOffset = 0;
	}
}


template<size_t SAMPLE_SIZE_BYTE>
void Blackbox<SAMPLE_SIZE_BYTE>::writeToSdCard(const uint8_t* pBuffer)
{
	init();

	startLogging();

	asyncWriteToSdCard(pBuffer);

	f_close(&m_file);
}



/*
 * Write the data of the ready buffer to the SD file
 */
template<size_t SAMPLE_SIZE_BYTE>
void Blackbox<SAMPLE_SIZE_BYTE>::asyncWriteToSdCard(const uint8_t* pBuffer)
{
	UINT bytesWritten;
	m_isWriting = true;

	// Write to SD card using FATFS
	if (f_write(&m_file, pBuffer, bufferSize, &bytesWritten) != FR_OK || bytesWritten != bufferSize)
	{
		// Error
		m_isWriting = false;
		return;
	}

	// Ensure data is flushed to SD card
	f_sync(&m_file);

	m_isWriting = false;
}


// Explicit template definition
template class Blackbox<36>;
