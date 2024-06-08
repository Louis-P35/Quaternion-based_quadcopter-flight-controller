#pragma once

#include "quaternion.h"
#include "radio.h"

// Scan for I2C adress of connected components
void I2C_scaner();

// Debug print the orientation after computed by the AHRS
void printAttitude(const Quaternion& qAttitude);

// Debug print the power sent to the ESCs
void printMotorsPower(
  const int& pwm1, 
  const int& pwm2, 
  const int& pwm3, 
  const int& pwm4, 
  const int& min, 
  const int& max
  );

// Debug print pid error
void printError(const Quaternion& qErr);

// Debug print target attitude values
void printRadio(const Radio& radio);