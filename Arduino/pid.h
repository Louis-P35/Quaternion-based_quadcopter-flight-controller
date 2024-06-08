#pragma once

#include "quaternion.h"


/* PID class */
class PID
{
private:
  double m_kp = 0.0;
  double m_ki = 0.0;
  double m_kd = 0.0;

  double m_saturation = 100.0;

  double m_previousError = 0.0;
  double m_sommeError = 0.0;

public:
  // Constructor to initialize PID gains
  PID(double kp, double ki, double kd, double sat) : 
  m_kp(kp), m_ki(ki), m_kd(kd), m_saturation(sat)
  {
    m_previousError = 0.0;
    m_sommeError = 0.0;
  }

  // For Euler-based PID
  static double getError(const double& current, const double& target);

  // For Quaternion-based PID
  static Quaternion getError(const Quaternion& current, const Quaternion& target);

  double computePID(const double& error, const double& dt, const bool& integrate);
};



/*struct DualLoopControl
{
  PID m_angleControl;
  PID m_velocityControl;

  // Constructor to initialize PID controllers with specific gains
  DualLoopControl(float angleKp, float angleKi, float angleKd, float velocityKp, float velocityKi, float velocityKd)
  : m_angleControl(angleKp, angleKi, angleKd), m_velocityControl(velocityKp, velocityKi, velocityKd)
  {

  }

  float computeMotorPower(float targetAngle, float angle, float velocity, float dt)
  {
    float targetVelocity = angleControlLoop(targetAngle, angle, dt);

    return velocityControlLoop(targetVelocity, velocity, dt);
  }

  // Return the target velocity
  float angleControlLoop(float targetAngle, float angle, float dt)
  {
    float angleError = targetAngle - angle;

    // Outer loop
    return m_angleControl.computePID(angleError, dt);
  }

  // Return the motor power
  float velocityControlLoop(float targetVelocity, float velocity, float dt)
  {
    float velocityError = targetVelocity - velocity;

    // Inner loop
    return m_velocityControl.computePID(velocityError, dt);
  }
};*/
