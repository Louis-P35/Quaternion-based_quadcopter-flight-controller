/*
 * madgwick.hpp
 *
 *  Created on: Apr 26, 2025
 *      Author: louis
 */

#pragma once

#include "Utils/quaternion.hpp"

/*
The Madgwick filter utilizes quaternion representations for calculating orientations.
Quaternions are particularly well-suited for representing three-dimensional rotations because
they do not suffer from gimbal lock—a condition where the loss of one degree of freedom
in three-dimensional space causes two of the three rotation axes to align, which can
happen with Euler angles. Quaternions provide a compact, non-singular representation which
ensures stable and continuous calculations even during full 360-degree rotations.
*/


class MadgwickFilter
{
public:
  Quaternion m_qEst = Quaternion(1.0, 0.0, 0.0, 0.0);

public:
  MadgwickFilter()
  {
    m_qEst = Quaternion(1.0, 0.0, 0.0, 0.0);
  }

  void compute(
  const double& ax,
  const double& ay,
  const double& az,
  const double& gx,
  const double& gy,
  const double& gz,
  const double& dt
  );

  void getEulerAngle(double& roll, double& pitch, double& yaw);
};
