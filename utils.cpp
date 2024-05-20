#include <Arduino.h>
#include <Wire.h>

#include "utils.h"


/* Scan for I2C adress of connected components */
void I2C_scaner()
{
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0) Serial.println("No I2C devices found\n");
  else Serial.println("done\n");

  delay(5000);           // wait 5 seconds for the next scan
}


// Debug print the orientation after computed by the AHRS
void printAttitude(const Quaternion& qAttitude)
{
  double roll, pitch, yaw = 0.0;

  qAttitude.toEuler(roll, pitch, yaw);

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\t Pitch:");
  Serial.print(pitch);
  Serial.print("\t Yaw:");
  Serial.println(yaw);
}


// Debug print the power sent to the ESCs
void printMotorsPower(
  const int& pwm1, 
  const int& pwm2, 
  const int& pwm3, 
  const int& pwm4, 
  const int& min, 
  const int& max
  )
{
  Serial.print(pwm1);
  Serial.print("\t");
  Serial.print(pwm2);
  Serial.print("\t");
  Serial.print(pwm3);
  Serial.print("\t");
  Serial.print(pwm4);
  Serial.print("\t");
  // Also print min & maw power to force the plotter to keep the "full size" scale
  Serial.print(min);
  Serial.print("\t");
  Serial.println(max);
}



// Debug print pid error
void printError(const Quaternion& qErr)
{
  double errRoll = 0.0;
  double errPitch = 0.0;
  double errYaw = 0.0;

  // Switching to Euler representation can lead to gimbal lock
  qErr.toEuler(errRoll, errPitch, errYaw);

  Serial.print("Error roll: ");
  Serial.print(errRoll);
  Serial.print("\t Error pitch:");
  Serial.print(errPitch);
  Serial.print("\t Error yaw:");
  Serial.println(errYaw);
}


// Debug print target attitude values
void printRadio(const Radio& radio)
{
  Serial.print("Roll: ");
  Serial.print(radio.m_targetRoll);
  Serial.print("\t Pitch: ");
  Serial.print(radio.m_targetPitch);
  Serial.print("\t Yaw: ");
  Serial.print(radio.m_targetYaw);
  Serial.print("\t Thrust: ");
  Serial.println(radio.m_targetThrust);
}



