/*
 * radioPwm.hpp
 *
 *  Created on: Jul 23, 2024
 *      Author: Louis
 */

#pragma once



#pragma once

// Includes from project
#include "Radio/pwmWrapperC.h"
#include "Radio/radioBase.hpp"

// Includes from STL
#include <stdint.h>

/*
 * Class to parse the sbus protocol from the radio receiver
 */
class PWMParser : public RadioProtocole
{
public:
	static constexpr int m_minThrust = 1076;
	static constexpr int m_maxThrust = 1996;
	static constexpr int m_minRoll = 1076;
	static constexpr int m_maxRoll = 1996;
	static constexpr int m_minPitch = 1076;
	static constexpr int m_maxPitch = 1996;
	static constexpr int m_minYaw = 1076;
	static constexpr int m_maxYaw = 1996;
	static constexpr int m_nbChannel = 4;
	uint16_t m_channel[m_nbChannel] = {0};

private:
	bool m_failsafe = false;

public:
	PWMParser() = default;
	virtual void init() noexcept override;
	virtual uint16_t getChannel(const int& x) const noexcept override;
	virtual uint16_t getThrustRadioStick() const noexcept override;
	virtual uint16_t getRollRadioStick() const noexcept override;
	virtual uint16_t getPitchRadioStick() const noexcept override;
	virtual uint16_t getYawRadioStick() const noexcept override;
	virtual bool getSignalLost() const noexcept override;
	virtual void print() const noexcept override;

private:
	uint16_t scaleToMs(uint16_t val, const int& min, const int& max) const noexcept;
};
