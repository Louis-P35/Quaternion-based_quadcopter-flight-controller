#include <Arduino.h>
#include <Wire.h>

#include "radio.h"
#include "readPWM.h"

Radio::Radio(const double& mid, const double& expo, const double& targetAngleMax) 
: m_throttleMID(mid), m_throttleExpo(expo), m_targetAngleMax(targetAngleMax)
{
  // Calculate A and B coefficients for throttle curve
  m_a = (1.0 - m_throttleMID) / pow(m_throttleMID, m_throttleExpo);
  m_b = 1.0 - m_a;

  m_previousReadTime = millis();
}

void Radio::readRadioReceiver(const bool& isFlying)
{
  // Get the ellapsed time since the last time the receiver has been read
  // and read the data only every 20 ms
  unsigned long currentTime = millis();
  double dt = (currentTime - m_previousReadTime) / 1000.0;
  if ((currentTime - m_previousReadTime) < 20)
  {
    return;
  }
  m_previousReadTime = currentTime;

  /* Read the radio receiver */
  m_radioChannel1 = getRadioChannel(0); // Roll
  m_radioChannel2 = getRadioChannel(1); // Pitch
  m_radioChannel3 = getRadioChannel(2); // Thrust
  m_radioChannel4 = getRadioChannel(3); // Yaw
  auto printRadioChannels = [this]() -> void
  {
    Serial.print(m_radioChannel1);
    Serial.print("\t");
    Serial.print(m_radioChannel2);
    Serial.print("\t");
    Serial.print(m_radioChannel3);
    Serial.print("\t");
    Serial.println(m_radioChannel4);
  };
  //printRadioChannels();

  m_targetRoll = msToDegree(m_radioChannel1, m_targetAngleMax, true);
  m_targetPitch = msToDegree(m_radioChannel2, m_targetAngleMax, false);
  m_targetYaw = integrateTargetYaw(m_radioChannel4, dt, isFlying);
  const double rawThrottle = ((double)m_radioChannel3 - 1000.0) / 10.0;
  m_targetThrust = getThrottle(rawThrottle);
}


// Return the throttle value according to the radio receiver input
// and the defined throttle curve.
double Radio::getThrottle(const double& radioInput) const
{
  return radioInput * (m_a * pow(radioInput, m_throttleExpo) + m_b);
}


// Method to convert microseconds to angle
double Radio::msToDegree(const unsigned long& duration, const double& amplitudeMax, const bool& invertAxe)
{
  const double deadZone = 1.0;

  double tmp = (((((double)duration) - 1000.0) / 1000.0) * (amplitudeMax * 2.0)) - amplitudeMax;

  // Hysteresis to have a stable zero
  if (tmp > -deadZone && tmp < deadZone)
  {
    tmp = 0.0;
  }
  else if (tmp > 0.0)
  {
    tmp -= deadZone;
  }
  else
  {
    tmp += deadZone;
  }

  double retVal = (tmp < -amplitudeMax) ? (-amplitudeMax) : (tmp > amplitudeMax) ? amplitudeMax : tmp;

  if (invertAxe)
    return -retVal;

  return retVal;
}


// Method to integrate yaw command
double Radio::integrateTargetYaw(const unsigned long& duration, const double& dt, const bool& isFlying)
{
  const double deadZone = 0.05;
  const double speed = 400.0;

  // Update target yaw only when flying
  if (!isFlying)
    return m_targetYaw;

  double tmp = ((((double)duration) - 1000.0) / 1000.0) - 0.5;

  // Hysteresis to have a stable zero
  if (tmp > -deadZone && tmp < deadZone)
  {
    tmp = 0.0;
  }
  else if (tmp > 0.0)
  {
    tmp -= deadZone;
  }
  else
  {
    tmp += deadZone;
  }

  // Integrate target yaw
  m_targetYaw += tmp * speed * dt;

  // Clamp value to [-180;+180]
  if (m_targetYaw > 180.0)
    m_targetYaw -= 360.0;
  else if (m_targetYaw < -180.0)
    m_targetYaw += 360.0;

  return m_targetYaw;
}

