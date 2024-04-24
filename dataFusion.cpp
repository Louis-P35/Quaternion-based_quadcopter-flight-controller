#include <Arduino.h>
#include <Wire.h>

#include "ms5611.h"
#include "dataFusion.h"

#define G_FORCE 9.81


/*
This function read the altutude measured
by the barometric sensor.
Using exponential moving average as a filter.
*/
double readAltitude(MS5611* pBarometer)
{
  const double alpha = 0.1;

  int result = pBarometer->read();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
    return;
  }

  // Fetch pressure data
  double pressure = pBarometer->getPressure();

  // Exponential moving average to filter data
  static double previousEMA = 0.0;
  double currentEMA = (pressure - previousEMA) * alpha + previousEMA;
  previousEMA = currentEMA;

  //Serial.print("T: ");
  //Serial.println(g_barometer.getTemperature(), 2);
  //Serial.print("P: ");
  //Serial.println(currentEMA, 2);

  auto calculateAltitude = [](double pressure) -> double
  {
    double seaLevelPressure = 101325; // Standard atmospheric pressure at sea level in Pascals
    return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 1.0 / 5.255));
  };

  // Multiply by 100 because currentEMA is in milibar and 1 mbar = 1 hPa = 100 Pascals
  double altitude = calculateAltitude(currentEMA*100.0);

  //Serial.print("Alt: ");
  //Serial.println(altitude, 2);

  return altitude;
}


double _complementaryFilter(double fusedValue, double value, double valueDerivative, double dt, double gain)
{
  return gain * (fusedValue + (valueDerivative * dt)) + (1.0 - gain) * value;
}


double computeAltitude(double accX, double accY, double accZ, double roll_rad, double pitch_rad, double barometerAlt, double dt)
{
  /*
  Compute the Z velocity by integrating the accelerometer
  This function do not work for now
  */

  static double zVelocity = 0.0;
  static double altitude = 0.0;
  static double prevAltitude = 0.0;

  // Compute the Z acceleration in the world coordinate system
  double worldAccZ = -sin(pitch_rad) * accX + cos(pitch_rad) * sin(roll_rad) * accY + cos(pitch_rad) * cos(roll_rad) * accZ;
  // Remove the gravity vector (g) from the acceleration vector
  worldAccZ -= 1.0;
  // Convert in m/s
  worldAccZ *= G_FORCE;

  // Integrate it to get Z velocity
  //zVelocity += worldAccZ * dt;
  zVelocity = _complementaryFilter(zVelocity, 0.0, worldAccZ, dt, 0.999);

  // Remove the gravity vector from the acceleration vector
  //double accZWithoutGravity = accZ - 0.99;

  //double mesuredVelocity = (barometerAlt - prevAltitude) / dt;
  //prevAltitude = barometerAlt;

  // Integrate acceleation of the Z axis to get the Z velocity
  //zVelocity += accZWithoutGravity * dt;
  //zVelocity = _complementaryFilter(zVelocity, mesuredVelocity, accZWithoutGravity, dt, 1.0);

  // Integrate the Z velocity to get the altitude
  //altitude += zVelocity * dt;
  altitude = _complementaryFilter(altitude, barometerAlt, zVelocity, dt, 0.98);

  return altitude;
}


