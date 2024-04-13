#include <Arduino.h>
#include <Wire.h>

#include "mpu6050.h"

#define MPU_I2C_ADDR 0x69                 // MPU6050 I2C address
#define G_FORCE 9.81
#define RAD_TO_DEGREE 57.295779513082321


MPU6050::MPU6050()
{

}

void MPU6050::init()
{
  Wire.begin();                             // Initialize comunication
  Wire.beginTransmission(MPU_I2C_ADDR);     // Start communication with MPU6050 // MPU=0x69
  Wire.write(0x6B);                         // Talk to the register 6B
  Wire.write(0x00);                         // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);               // end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x1C);                         // Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                         // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x1B);                         // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x01 << 3);                    // Set the register bits (500deg/s full scale)
  Wire.endTransmission(true);
  
  delay(20);
  
}


void MPU6050::calibrate()
{
  m_accOffsetX = 0.0;
  m_accOffsetY = 0.0;
  m_accOffsetZ = 0.0;

  const int range = 1000;

  for (int i = 0; i < range; ++i)
  {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, 6, true);

    m_accOffsetX += (Wire.read() << 8 | Wire.read()) / m_accScale; // TODO: Duplicate code...
    m_accOffsetY += (Wire.read() << 8 | Wire.read()) / m_accScale;
    m_accOffsetZ += (Wire.read() << 8 | Wire.read()) / m_accScale;
  }

  // Divide the sum to get the error value
  m_accOffsetX /= (double)range;
  m_accOffsetY /= (double)range;
  m_accOffsetZ /= (double)range;

  // Find the g vector and normalize it
  double norm = sqrt(m_accOffsetX * m_accOffsetX + m_accOffsetY * m_accOffsetY + m_accOffsetZ * m_accOffsetZ);
  double normalizedAccX = m_accOffsetX / norm;
  double normalizedAccY = m_accOffsetY / norm;
  double normalizedAccZ = m_accOffsetZ / norm;

  // Remove the gravity vector from the offset
  m_accOffsetX -= normalizedAccX;
  m_accOffsetY -= normalizedAccY;
  m_accOffsetZ -= normalizedAccZ;

  Serial.print("AccErrorX: ");
  Serial.println(m_accOffsetX);
  Serial.print("AccErrorY: ");
  Serial.println(m_accOffsetY);
  Serial.print("AccErrorZ: ");
  Serial.println(m_accOffsetZ);


  m_gyroOffsetX = 0.0;
  m_gyroOffsetY = 0.0;
  m_gyroOffsetZ = 0.0;
  
  // Read gyro values 200 times
  for (int i = 0; i < range; ++i)
  {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, 6, true);

    double gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    double gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    double gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Sum all readings
    m_gyroOffsetX += gyroX;
    m_gyroOffsetY += gyroY;
    m_gyroOffsetZ += gyroZ;
  }

  //Divide the sum by 200 to get the error value
  m_gyroOffsetX /= (double)range;
  m_gyroOffsetY /= (double)range;
  m_gyroOffsetZ /= (double)range;

  // Print the error values on the Serial Monitor
  Serial.print("GyroErrorX: ");
  Serial.println(m_gyroOffsetX);
  Serial.print("GyroErrorY: ");
  Serial.println(m_gyroOffsetY);
  Serial.print("GyroErrorZ: ");
  Serial.println(m_gyroOffsetZ);
}


void MPU6050::get_mpu6050_data()
{
  // Read acceleromter
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true);

  // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  // So for a range of +-8g, we need to divide the raw values by 4096
  double accX = ((Wire.read() << 8 | Wire.read()) / m_accScale);
  double accY = ((Wire.read() << 8 | Wire.read()) / m_accScale);
  double accZ = ((Wire.read() << 8 | Wire.read()) / m_accScale);

  // Filtering accelerometer data
  m_filteredAcceloremeterX = m_lpf_acc_gain * m_previousAccX + (1.0 - m_lpf_acc_gain) * accX;
  m_previousAccX = m_filteredAcceloremeterX;
  m_filteredAcceloremeterY = m_lpf_acc_gain * m_previousAccY + (1.0 - m_lpf_acc_gain) * accY;
  m_previousAccY = m_filteredAcceloremeterY;
  m_filteredAcceloremeterZ = m_lpf_acc_gain * m_previousAccZ + (1.0 - m_lpf_acc_gain) * accZ;
  m_previousAccZ = m_filteredAcceloremeterZ;

  // Remove offset
  m_filteredAcceloremeterX -= m_accOffsetX;
  m_filteredAcceloremeterY -= m_accOffsetY;
  m_filteredAcceloremeterZ -= m_accOffsetZ;

  /*Serial.print(accX);
  Serial.print("     ");
  Serial.print(accY);
  Serial.print("     ");
  Serial.println(accZ);*/

  /*
  Calculating Roll and Pitch from the accelerometer data
  Using atan2() function so:
    -> It return a 360 degrees range of values instead of the [-90, +90] range from atan()
    -> It handle the division by zero that can occur using atan(Y / sqrt(X*X + Z*Z))
  */
  m_angleAccX = RAD_TO_DEGREE * (atan2(-m_filteredAcceloremeterY, -m_filteredAcceloremeterZ) + PI);
  m_angleAccY = RAD_TO_DEGREE * (atan2(-m_filteredAcceloremeterX, -m_filteredAcceloremeterZ) + PI);

  // Map the [0, 360] range to [-180, 180]
  if (m_angleAccX > 180.0)
  {
    m_angleAccX -= 360.0;
  }
  if (m_angleAccY > 180.0)
  {
    m_angleAccY -= 360.0;
  }
  m_angleAccY = -m_angleAccY;

  /*Serial.print(m_angleAccX);
  Serial.print("     ");
  Serial.println(m_angleAccY);*/
  //Serial.print("     ");
  //Serial.println(z);

  // Read gyroscope
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true);

  /*
  0 ± 250 °/s 131 LSB/°/s
  1 ± 500 °/s 65.5 LSB/°/s
  2 ± 1000 °/s 32.8 LSB/°/s
  3 ± 2000 °/s 16.4 LSB/°/s
  */
  double gyroVelocityX = (Wire.read() << 8 | Wire.read()) / 65.5;
  double gyroVelocityY = (Wire.read() << 8 | Wire.read()) / 65.5;
  double gyroVelocityZ = (Wire.read() << 8 | Wire.read()) / 65.5;

  // Filtering gyroscope data. Maybe pointless... ?
  m_filteredGyroX = m_lpf_gyro_gain * m_previousGyroX + (1.0 - m_lpf_gyro_gain) * gyroVelocityX;
  m_previousGyroX = m_filteredGyroX;
  m_filteredGyroY = m_lpf_gyro_gain * m_previousGyroY + (1.0 - m_lpf_gyro_gain) * gyroVelocityY;
  m_previousGyroY = m_filteredGyroY;
  m_filteredGyroZ = m_lpf_gyro_gain * m_previousGyroZ + (1.0 - m_lpf_gyro_gain) * gyroVelocityZ;
  m_previousGyroZ = m_filteredGyroZ;

  // Correct the outputs with the calculated error values
  m_filteredGyroX -= m_gyroOffsetX;
  m_filteredGyroY -= m_gyroOffsetY;
  m_filteredGyroZ -= m_gyroOffsetZ;

  /*Serial.print(m_filteredGyroX);
  Serial.print("     ");
  Serial.print(m_filteredGyroY);
  Serial.print("     ");
  Serial.println(m_filteredGyroZ);*/
}

