#include <Arduino.h>

#include "kalman.h"

using namespace BLA;

// With quaternion, the transition matrix is linear so no need for the extended kalman filter


Kalman::Kalman()
{
  // Init covarance matrix
  m_P[0][0] = 0.0; 
  m_P[0][1] = 0.0;
  m_P[1][0] = 0.0; 
  m_P[1][1] = 0.0;
}

Kalman1D::Kalman1D(double Qangle, double Qbias, double Rmeasure) : Kalman()
{
  m_estimatedAngle = 0.0;
  m_estimatedBias = 0.0;

  m_Qangle = Qangle;
  m_Qbias = Qbias;
  m_Rmeasure = Rmeasure;
}


double Kalman1D::compute(double accelAngle, double gyroRate, double dt)
{
  /* Extended Kalman Filter (EKF) high-level and intuitive pseudo-code: */

  // Predict step: Integrate gyro data over time to predict the current angle
  // estimated_angle += dt * gyro_rate  

  // Update error estimates
  // error_estimate: Increase with time as confidence in the prediction decreases
  // error_estimate += process_noise // process_noise is represented in the covarience matrix

  // error_measurement: Set based on how noisy the accelerometer data is
  // error_measurement = accelerometer_noise // accelerometer_noise is also represented in the covarience matrix

  // Update Kalman Gain based on the ratio of prediction uncertainty to the total uncertainty
  // kalman_gain = error_estimate / (error_estimate + error_measurement) 

  // Update step: Correct the estimated angle with accelerometer data, adjusted by the Kalman Gain
  // estimated_angle += kalman_gain * (accel_angle - estimated_angle)

  /**************************************************************************/


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
  // Error between mesurement (t) and prediction done at t-1
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


Kalman2D::Kalman2D() : Kalman()
{
  F = {1, 0.004, 0, 1};  
  G = {0.5*0.004*0.004,0.004};
  H = {1, 0};
  I = {1, 0, 0, 1};
  Q = G*~G*100.0f;
  R = {30*30};
  P = {0, 0, 0, 0};
  S = {0, 0};
}

double Kalman2D::compute(double zAltBarometer, double zAccel, double dt)
{
  // code from:
  // https://github.com/CarbonAeronautics/Part-XIX-2D-Kalman-filter/blob/main/ArduinoCode

  Acc = {zAccel};

  S = F*S+G*Acc;

  P = F*P*~F+Q;

  L=H*P*~H+R;

  Invert(L);

  K=P*~H*L;

  M={zAltBarometer};

  S=S+K*(M-H*S);

  AltitudeKalman=S(0,0);

  VelocityVerticalKalman=S(1,0);

  P=(I-K*H)*P;

  return AltitudeKalman;  // Return the updated altitude estimate
}


