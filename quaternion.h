#pragma once

#include <Arduino.h>
#include <Wire.h>

#define RAD_TO_DEGREE (180.0 / PI)


/*
Quaternion class
*/
class Quaternion
{
public:
  // Init quaternion as unit vector
  double m_w = 1.0;
  double m_x = 0.0;
  double m_y = 0.0;
  double m_z = 0.0;

  Quaternion(double w, double x, double y, double z) : m_w(w), m_x(x), m_y(y), m_z(z)
  {

  }

  // Return the conjugated quaternion (imaginary component with the reverse sign)
  Quaternion conjugate() const
  {
    return Quaternion(m_w, -m_x, -m_y, -m_z);
  }

  // Return the norm of the quaternion
  double norm() const
  {
    return sqrt(m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z);
  }

  // Return the norm squared of the quaternion
  double normSquared() const
  {
    return m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z;
  }

  // Normalize the quaternion
  Quaternion& normalize()
  {
    double _norm = norm();

    if (_norm != 0.0)
    {
      m_w /= _norm;
      m_x /= _norm;
      m_y /= _norm;
      m_z /= _norm;
    }

    return *this;
  }

  // Inverse of the quaternion
  Quaternion inverse() const
  {
    double norm_sq = normSquared(); // TODO: norm ? norm squared ? 

    // Handle zero norm case, though it should ideally never occur
    if (norm_sq == 0.0)
      return Quaternion(0.0, 0.0, 0.0, 0.0);

    Quaternion conj = conjugate();

    return Quaternion(conj.m_w / norm_sq, conj.m_x / norm_sq, conj.m_y / norm_sq, conj.m_z / norm_sq);
  }

  // Return the dot product of two quaternion
  double dotProduct(const Quaternion& b) const
  {
    return m_w * b.m_w + m_x * b.m_x + m_y * b.m_y + m_z * b.m_z;
  }


  // Print the quaternion throught the serial port
  void print() const
  {
    Serial.print("m_w: ");
    Serial.print(m_w);
    Serial.print("\t m_x: ");
    Serial.print(m_x);
    Serial.print("\t m_y: ");
    Serial.print(m_y);
    Serial.print("\t m_z: ");
    Serial.println(m_z);
  }

  // Compute the Euler angle from the quaternion
  // Assume roll is x, pitch is y and yaw is z
  // Assume right hand system
  void toEuler(double& roll, double& pitch, double& yaw) const
  {
    // Roll
    double sinr_cosp = 2.0 * (m_w * m_x + m_y * m_z);
    double cosr_cosp = 1.0 - 2.0 * (m_x * m_x + m_y * m_y);
    roll = atan2(sinr_cosp, cosr_cosp);
    roll *= RAD_TO_DEGREE;

    // Pitch
    double sinp = 2.0 * (m_w * m_y - m_z * m_x);
    if (abs(sinp) >= 1.0)
      pitch = copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
    else
      pitch = asin(sinp);
    pitch  *= RAD_TO_DEGREE;

    // Yaw
    double siny_cosp = 2.0 * (m_w * m_z + m_x * m_y);
    double cosy_cosp = 1.0 - 2.0 * (m_y * m_y + m_z * m_z);
    yaw = atan2(siny_cosp, cosy_cosp);
    yaw *= RAD_TO_DEGREE;
  }

  // Static function to convert Euler angles to Quaternion (in ZXY order)
  static Quaternion fromEuler(const double& roll, const double& pitch, const double& yaw)
  {
      const double cy = cos(yaw * 0.5);
      const double sy = sin(yaw * 0.5);
      const double cp = cos(pitch * 0.5);
      const double sp = sin(pitch * 0.5);
      const double cr = cos(roll * 0.5);
      const double sr = sin(roll * 0.5);

      const double w = cr * cp * cy + sr * sp * sy;
      const double x = sr * cp * cy - cr * sp * sy;
      const double y = cr * sp * cy + sr * cp * sy;
      const double z = cr * cp * sy - sr * sp * cy;

      return Quaternion(w, x, y, z);
  }

  /* Operators override */

  // Quaternion multiplication override
  Quaternion operator*(const Quaternion& b) const
  {
    return Quaternion(
      m_w * b.m_w - m_x * b.m_x - m_y * b.m_y - m_z * b.m_z,  // Real part
      m_w * b.m_x + m_x * b.m_w + m_y * b.m_z - m_z * b.m_y,  // i
      m_w * b.m_y - m_x * b.m_z + m_y * b.m_w + m_z * b.m_x,  // j
      m_w * b.m_z + m_x * b.m_y - m_y * b.m_x + m_z * b.m_w   // k
    );
  }

  // Assignment and Quaternion multiplication override
  Quaternion& operator*=(const Quaternion& b)
  {
    Quaternion tmp = *this * b;
    
    m_w = tmp.m_w;
    m_x = tmp.m_x;
    m_y = tmp.m_y;
    m_z = tmp.m_z;

    return *this;
  }

  // Scalar multiplictaion override
  Quaternion operator*(const double& scalar) const
  {
    return Quaternion(m_w * scalar, m_x * scalar, m_y * scalar, m_z * scalar);
  }

  // Assignment and scalar multiplication override
  Quaternion& operator*=(const double& scalar)
  {
    m_w *= scalar;
    m_x *= scalar;
    m_y *= scalar;
    m_z *= scalar;

    return *this;
  }

  // Quaternion addition override
  Quaternion operator+(const Quaternion& b) const
  {
    return Quaternion(m_w + b.m_w, m_x + b.m_x, m_y + b.m_y, m_z + b.m_z);
  }

  // Assignment and quaternion addition override
  Quaternion& operator+=(const Quaternion& b)
  {
    m_w += b.m_w;
    m_x += b.m_x;
    m_y += b.m_y;
    m_z += b.m_z;

    return *this;
  }

  // Quaternion substraction override
  Quaternion operator-(const Quaternion& b) const
  {
    return Quaternion(m_w - b.m_w, m_x - b.m_x, m_y - b.m_y, m_z - b.m_z);
  }

  // Assignment and quaternion substraction override
  Quaternion& operator-=(const Quaternion& b)
  {
    m_w -= b.m_w;
    m_x -= b.m_x;
    m_y -= b.m_y;
    m_z -= b.m_z;

    return *this;
  }
};



