#pragma once

#include "quaternion.h"

class Radio
{
private:
  double m_throttleMID = 0.0;
  double m_throttleExpo = 0.0;
  double m_a = 0.0;
  double m_b = 0.0;

  unsigned long m_previousReadTime = 0;

  double m_targetAngleMax = 20.0;

public:
  double m_targetYaw = 0.0;
  double m_targetPitch = 0.0;
  double m_targetRoll = 0.0;
  double m_targetThrust = 0.0;

  unsigned long m_radioChannel1 = 0;
  unsigned long m_radioChannel2 = 0;
  unsigned long m_radioChannel3 = 0;
  unsigned long m_radioChannel4 = 0;

public:
  Radio(const double& mid, const double& expo, const double& targetAngleMax);
  void readRadioReceiver(const bool& isFlying);
  double getThrottle(const double& radioInput) const;
  double msToDegree(const unsigned long& duration, const double& amplitudeMax, const bool& invertAxe);
  double integrateTargetYaw(const unsigned long& duration, const double& dt, const bool& isFlying);
  Quaternion getTargetQuaternion();
};