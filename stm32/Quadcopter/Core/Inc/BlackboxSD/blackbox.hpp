/*
 * blackbox.hpp
 *
 *  Created on: May 14, 2025
 *      Author: louis
 */

#pragma once

// Includes from STL
#include <cstdlib>
#include <stdint.h>
#include <string>

// Includes from STM32CubeIDE
#include "fatfs.h"


template<size_t SAMPLE_SIZE_BYTE>
class Blackbox
{
private:
	static constexpr size_t sampleRateHz = 1000;
	static constexpr size_t bufferDurationSec = 5;
	static constexpr size_t bufferSize = sampleRateHz * SAMPLE_SIZE_BYTE * bufferDurationSec;

	FATFS m_fs; // FATFS filesystem object
	FIL m_file; // File object
	bool m_isFileOpen = false;
	volatile bool m_isWriting = false; // Flag to track ongoing DMA write

public:
	uint8_t m_buffer1[bufferSize] = {0};
	//uint8_t m_buffer2[bufferSize] = {0};

	volatile size_t m_writeOffset = 0;
	volatile int m_currentBuffer = 0;

	std::string m_fileName = "TotoTest.txt";

public:
	Blackbox() = default;

	bool init();

	void blackBoxLogToSdCard(const uint8_t pBuffer[SAMPLE_SIZE_BYTE]);

	// Start logging (open file)
	bool startLogging();

	// Stop logging (close file)
	void stopLogging();

private:
	void asyncWriteToSdCard(const uint8_t* pBuffer);

	void writeToSdCard(const uint8_t* pBuffer);
};
