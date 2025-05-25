/*
 * radioSbus.h
 *
 *  Created on: May 24, 2025
 *      Author: louis
 */

#pragma once

// Includes from project
#include "Radio/sbusWrapperC.h"
#include "Radio/radioBase.hpp"

// Includes from STL
#include <stdint.h>

/*
 * Class to parse the sbus protocol from the radio receiver
 */
class SbusParser : public RadioProtocole
{
public:
	static constexpr int m_minThrust = 306;
	static constexpr int m_maxThrust = 1693;
	static constexpr int m_minRoll = 306;
	static constexpr int m_maxRoll = 1693;
	static constexpr int m_minPitch = 306;
	static constexpr int m_maxPitch = 1693;
	static constexpr int m_minYaw = 306;
	static constexpr int m_maxYaw = 1693;
	static constexpr int m_nbChannel = 16;
	uint16_t m_channel[m_nbChannel] = {0};

private:
	uint8_t m_pSbusBuf[SBUS_FRAME_SIZE];
	bool m_frameLost = true;
	bool m_failsafe = false;

public:
	SbusParser() = default;
	bool parseSbusFrame() noexcept;
	virtual void init() noexcept override;
	virtual uint16_t getChannel(const int& x) const noexcept override;
	virtual uint16_t getThrustRadioStick() const noexcept override;
	virtual uint16_t getRollRadioStick() const noexcept override;
	virtual uint16_t getPitchRadioStick() const noexcept override;
	virtual uint16_t getYawRadioStick() const noexcept override;
	virtual bool getSignalLost() const noexcept override;
	virtual void print() const noexcept override;

private:
	void getBuffer() noexcept;
	uint16_t scaleToMs(uint16_t val, const int& min, const int& max) const noexcept;
};
