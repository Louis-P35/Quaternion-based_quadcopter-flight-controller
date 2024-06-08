#pragma once

#include <BasicLinearAlgebra.h>


class Kalman
{
protected:
  // Error covariance matrix
  // Represent the uncertainty about the state of the system
  double m_P[2][2] = { { 0.0, 0.0 }, { 0.0, 0.0 } };

  // Kalman gain
  double m_K[2];

public:
  Kalman();

  virtual double compute(double ref, double primitive, double dt) = 0;

};

/*
One dimentional extended kalman filter
*/
class Kalman1D : public Kalman
{
public:
  double m_estimatedAngle;

private:
  double m_estimatedBias;

  // Filter control variables
  double m_Qangle = 0.001;    // Angle noise
  double m_Qbias = 0.003;     // Gyroscope bias noise
  double m_Rmeasure = 0.03;   // Measure noise

public:
  Kalman1D(double Qangle, double Qbias, double Rmeasure);

  double compute(double accelAngle, double gyroRate, double dt) override final;
};


/*
Two dimentional extended kalman filter
*/
class Kalman2D : public Kalman
{
public:
  double AltitudeKalman;
  double VelocityVerticalKalman;

private:
  BLA::Matrix<2,2> F;
  BLA::Matrix<2,1> G;
  BLA::Matrix<2,2> P;
  BLA::Matrix<2,2> Q;
  BLA::Matrix<2,1> S;
  BLA::Matrix<1,2> H;
  BLA::Matrix<2,2> I;
  BLA::Matrix<1,1> Acc;
  BLA::Matrix<2,1> K;
  BLA::Matrix<1,1> R;
  BLA::Matrix<1,1> L;
  BLA::Matrix<1,1> M;

public:
  Kalman2D();

  double compute(double zMeasure, double accel, double dt) override final;
};


