#include <Arduino.h>

#include "pid.h"


// Constructor to initialize PID gains
PID::PID(double kp, double ki, double kd, double sat) : m_kp(kp), m_ki(ki), m_kd(kd), m_saturation(sat)
{
  m_previousTime = micros();
}


double PID::computePID(double error, bool integrate)
{
  // Measure the delta time since last update
  unsigned long currentTime = micros();
  double dt = (double)(currentTime - m_previousTime) / 1000000.0; // Divide by 1000000 to get seconds
  m_previousTime = currentTime;

  // Proportionnal gain
  double p = error * m_kp;

  // Derivative gain
  double derivedError = (error - m_previousError) / dt;
  double d = derivedError * m_kd;
  m_previousError = error;

  // Integral gain
  if (integrate) // Do not integrate error if the drone is not flying
  {
    m_sommeError += error * dt;
    // Prevent windup
    if (m_sommeError > m_saturation)
    {
      m_sommeError = m_saturation;
    }
    else if (m_sommeError < -m_saturation)
    {
      m_sommeError = -m_saturation;
    }
  }
  double i = m_sommeError * m_ki;

  return p + i + d;
}