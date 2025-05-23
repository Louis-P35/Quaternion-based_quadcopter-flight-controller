/*
 * IMU.hpp
 *
 *  Created on: May 12, 2025
 *      Author: louis
 */

#pragma once

// Include from project
#include "Utils/vector.hpp"
#include "Filters/lowpassFilter.hpp"
#include "Filters/notchFilter.hpp"
#include "Filters/lpfBiquadButterworth.hpp"


/*
 * This class is all float because IMU sensor data are on 16 bits
 */
class IMU
{
private:
	Vector3<float> m_magBias = Vector3<float>(-40.05f, 12.15f, 29.025f);
	Vector3<float> m_accelOffset = Vector3<float>(0.0f, 0.0f, 0.0f);
	Vector3<float> m_gyroOffset = Vector3<float>(0.0f, 0.0f, 0.0f);

	static constexpr float m_lpf_gyro_gain = 0.05f;
	static constexpr float m_lpf_acc_gain = 0.1f;

	LPF<float> m_lpfAccelX;
	LPF<float> m_lpfAccelY;
	LPF<float> m_lpfAccelZ;

	LPF<float> m_lpfGyroX;
	LPF<float> m_lpfGyroY;
	LPF<float> m_lpfGyroZ;

	LPF<float> m_lpfGyroX2;
	LPF<float> m_lpfGyroY2;
	LPF<float> m_lpfGyroZ2;

	NotchFilter<float> m_notchGyroX;
	NotchFilter<float> m_notchGyroY;
	NotchFilter<float> m_notchGyroZ;

	NotchFilter<float> m_notchGyroX2;
	NotchFilter<float> m_notchGyroY2;
	NotchFilter<float> m_notchGyroZ2;

	NotchFilter<float> m_notchAccelX;
	NotchFilter<float> m_notchAccelY;
	NotchFilter<float> m_notchAccelZ;

public:
	Vector3<float> m_accel = {0.0f};
	Vector3<float> m_gyro = {0.0f};
	Vector3<float> m_gyroRaw = {0.0f};

	Vector3<float> m_gyroDebug[5000];
	int m_gyroDebugIndex = 0;

public:
	IMU() = default;

	void init(
			const float& dataAcquisitionRate,
			const float& gyroNotchF0,
			const float& gyroNotchQ,
			const float& accelNotchF0,
			const float& accelNotchQ
			);

	void setAccelOffset(const Vector3<float>& offset);
	void setGyroOffset(const Vector3<float>& offset);

	void readAndFilterIMU_gdps();
	void gyroAccelCalibration();
	void magnetometerCalibration();
	void calibrateHoverOffset();
};
