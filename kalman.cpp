#include <Arduino.h>

#include "kalman.h"


Kalman::Kalman(double Qangle, double Qbias, double Rmeasure)
{
  m_estimatedAngle = 0.0;
  m_estimatedBias = 0.0;

  // Init covarance matrix
  m_P[0][0] = 0.0; 
  m_P[0][1] = 0.0;
  m_P[1][0] = 0.0; 
  m_P[1][1] = 0.0;

  m_Qangle = Qangle;
  m_Qbias = Qbias;
  m_Rmeasure = Rmeasure;
}


double Kalman::compute(double accelAngle, double gyroRate, double dt)
{
  // Integrate the angular velocity (from gyro) to get angle
  m_estimatedAngle += dt * (gyroRate - m_estimatedBias);

  // Update error covariance matrix
  m_P[0][0] += dt * (dt * m_P[1][1] - m_P[0][1] - m_P[1][0] + m_Qangle);
  m_P[0][1] -= dt * m_P[1][1];
  m_P[1][0] -= dt * m_P[1][1];
  m_P[1][1] += m_Qbias * dt;

  // Compute Kalman gain
  double S = m_P[0][0] + m_Rmeasure;
  m_K[0] = m_P[0][0] / S;
  m_K[1] = m_P[1][0] / S;

  // Compute angular error
  double y = accelAngle - m_estimatedAngle;

  // Update of estimated angle and biais
  m_estimatedAngle += m_K[0] * y;
  m_estimatedBias += m_K[1] * y;

  // Error covariance matrix update
  double P00_temp = m_P[0][0];
  double P01_temp = m_P[0][1];

  m_P[0][0] -= m_K[0] * P00_temp;
  m_P[0][1] -= m_K[0] * P01_temp;
  m_P[1][0] -= m_K[1] * P00_temp;
  m_P[1][1] -= m_K[1] * P01_temp;

  return m_estimatedAngle;
}


