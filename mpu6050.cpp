#include <Arduino.h>
#include <Wire.h>

#include "mpu6050.h"

const double COMPLEMENTARY_FILTER_GAIN = 0.96;


MPU6050::MPU6050()
{

}

void MPU6050::init()
{
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_I2C_ADDR);       // Start communication with MPU6050 // MPU=0x69
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  /*Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
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


void MPU6050::get_mpu6050_data(double dt)
{
  static double gyroAngleX = 0.0;

  static int print = 0;

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

  // Calculating Roll and Pitch from the accelerometer data
  m_angleAccX = (atan(m_filteredAcceloremeterY / sqrt(pow(m_filteredAcceloremeterX, 2) + pow(m_filteredAcceloremeterZ, 2))) * 180 / PI);
  m_angleAccY = (atan(-1 * m_filteredAcceloremeterX / sqrt(pow(m_filteredAcceloremeterY, 2) + pow(m_filteredAcceloremeterZ, 2))) * 180 / PI);

  //float accAngleX = atan(accY / accZ) * 180 / PI;
  //float accAngleY = asin(accX / G_FORCE) * 180 / PI; // asin(accX / g) => asin(accX / 1.0) => asin(accX)

  //float accAngleX = (180.0 / PI) * (atan2(-accY, -accZ) + PI);
  //float accAngleY = (180.0 / PI) * (atan2(-accX, -accZ) + PI);
  //float accAngleZ = (180.0 / PI) * (atan2(-accY, -accX) + PI);


  /*if (isnan(accAngleX))
  {
    accAngleX = 0.0;
  }
  if (isnan(accAngleY))
  {
    accAngleY = 0.0;
  }*/

  /*Serial.print(accX);
  Serial.print("     ");
  Serial.println(m_filteredAcceloremeterX);*/
  //Serial.print("     ");
  //Serial.println(accAngleZ);

  // Read gyroscope
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true);

  double gyroVelocityX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  double gyroVelocityY = (Wire.read() << 8 | Wire.read()) / 131.0;
  double gyroVelocityZ = (Wire.read() << 8 | Wire.read()) / 131.0;

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

  /*Serial.print(m_gyroVelocityX);
  Serial.print("     ");
  Serial.print(m_gyroVelocityY);
  Serial.print("     ");
  Serial.println(m_gyroVelocityZ);*/

  //gyroAngleX += (m_gyroVelocityX * dt);

  // Complementary filter
  //complementaryFilter(m_angleAccX, m_angleAccY, dt);

  //if (print == 25)
  /*{
    print = 0;

    Serial.print(m_roll);
    Serial.print("     ");
    Serial.print(m_pitch);
    Serial.print("     ");
    Serial.println(m_yaw);
  }
  print++;*/

  /*Serial.print(accAngleX);
  Serial.print("     ");
  Serial.print(gyroAngleX);
  Serial.print("     ");
  Serial.println(m_roll);*/

}


void MPU6050::complementaryFilter(double accAngleX, double accAngleY, double dt)
{
  m_roll = COMPLEMENTARY_FILTER_GAIN * (m_roll + (m_filteredGyroX * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleX;
  m_pitch = COMPLEMENTARY_FILTER_GAIN * (m_pitch + (m_filteredGyroY * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleY;
  m_yaw += m_filteredGyroZ * dt;
}