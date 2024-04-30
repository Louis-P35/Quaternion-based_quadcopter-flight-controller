#include <Arduino.h>

#include "pid.h"


// For Euler-based PID
double PID::getError(const double& current, const double& target)
{
  return target - current;
}

// For Quaternion-based PID
Quaternion PID::getError(const Quaternion& current, const Quaternion& target)
{
  // Compute error
  Quaternion q_error = target * current.inverse();
  // Ensure it is normalized
  q_error = q_error.normalize();

  /* A quaternion q and its negative -q represent the same orientation. This can result in a sudden 
  flip in the error direction when the quaternion passes through these equivalent representations.
  If the dot product is negative, it implies that the quaternion error might cross the antipodal 
  point, leading to a sign reversal. If so, we adjust the error quaternion by negating it to ensure 
  it represents the shortest path. */
  if (current.dotProduct(target) < 0.0)
  {
    return Quaternion(-q_error.m_w, -q_error.m_x, -q_error.m_y, -q_error.m_z);
  }

  return q_error;
}


double PID::computePID(const double& error, const double& dt, const bool& integrate)
{
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


