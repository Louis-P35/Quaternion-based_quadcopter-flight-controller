/* PID class */
struct PID
{
  double m_kp = 0.0;
  double m_ki = 0.0;
  double m_kd = 0.0;

  double m_saturation = 100.0;

  double m_previousError = 0.0;
  double m_sommeError = 0.0;

  unsigned long m_previousTime;

  // Constructor to initialize PID gains
  PID(double kp, double ki, double kd, double sat);

  double computePID(double error, bool integrate);
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
