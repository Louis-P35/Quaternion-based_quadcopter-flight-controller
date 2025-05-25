/*
 * radioBase.hpp
 *
 *  Created on: May 25, 2025
 *      Author: louis
 */

#pragma once

// Includes from STL
#include <stdint.h>



/*
 * Base class for handling the radio receiver protocol
 * Must be overridden for PWM, Sbus, etc
 */
class RadioProtocole
{
public:
	RadioProtocole() = default;
	virtual void init() noexcept = 0;
	virtual uint16_t getChannel(const int& x) const noexcept = 0;
	virtual uint16_t getThrustRadioStick() const noexcept = 0;
	virtual uint16_t getRollRadioStick() const noexcept = 0;
	virtual uint16_t getPitchRadioStick() const noexcept = 0;
	virtual uint16_t getYawRadioStick() const noexcept = 0;
	virtual bool getSignalLost() const noexcept = 0;
	virtual void print() const noexcept = 0;
};
