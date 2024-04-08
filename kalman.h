

class Kalman
{
public:
  double m_estimatedAngle;

private:
  // Error covariance matrix
  // Represent the uncertainty about the state of the system
  double m_P[2][2] = { { 0.0, 0.0 }, { 0.0, 0.0 } };

  // Kalman gain
  double m_K[2];

  double m_estimatedBias;

  // Filter control variables
  double m_Qangle = 0.001; // Angle noise
  double m_Qbias = 0.003;  // Gyroscope bias noise
  double m_Rmeasure = 0.03; // Measure noise

  unsigned long m_previousTime;

public:
  Kalman(double Qangle, double Qbias, double Rmeasure);

  double compute(double accelAngle, double gyroRate);

};


