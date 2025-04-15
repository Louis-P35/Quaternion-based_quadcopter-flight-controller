#include "AHRS/Quaternion.h"

namespace IMU_EKF
{

	Quaternion<float> operator*(float l, const Quaternion<float> &r)
	{
		return Quaternion<float>(r[v1] * l, r[v2] * l, r[v3] * l, r[w] * l);
	}

	Quaternion<double> operator*(double l, const Quaternion<double> &r)
	{
		return Quaternion<double>(r[v1] * l, r[v2] * l, r[v3] * l, r[w] * l);
	}

}
