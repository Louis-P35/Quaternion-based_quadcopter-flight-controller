#include <Arduino.h>
#include <Wire.h>

#include "hmc5883l.h"

#define RAD_TO_DEGREE 57.2957795


const int HMC5883L_I2C_ADDR = 0x1E;             // HMC5883L I2C address (magnetometer)


void magnetometerSetup()
{
  // HMC5883L configuration
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x00); // Configuration register A
  Wire.write(0x70); // 8 averaged samples, 15Hz frequency, normal mode
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x01); // Configuration register B
  Wire.write(0xA0); // Gain = 5
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x02); // Mode register
  Wire.write(0x00); // Continus measure mode
  Wire.endTransmission();
}


double fetch_magnetometer_data()
{
  static unsigned long previousFetchTime = micros();
  
  // Get the ellapsed time since the last time data was fetched
  unsigned long currentTime = micros();
  double elapsedTime = (currentTime - previousFetchTime) / 1000000.0; // Divide by 1000000 to get seconds
  
  // Read data throught I2C only at 15 Hz
  if (elapsedTime < (1.0/15.0))
  {
    return;
  }
  previousFetchTime = currentTime;
  
  // Start reading from data register on X MSB axis
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  
  // Request 6 bytes of data : 2 for each axis
  Wire.requestFrom(HMC5883L_I2C_ADDR, 6);
  if (6 <= Wire.available())
  {
    int16_t x = Wire.read() << 8 | Wire.read(); // X axis
    int16_t y = Wire.read() << 8 | Wire.read(); // Z axis (careful about axis order)
    int16_t z = Wire.read() << 8 | Wire.read(); // Y axis

    // Compute the angle in radians
    double angleRadians = atan2((double)y, (double)x);

    // Convert it in degrees
    return angleRadians * RAD_TO_DEGREE;
  }
}