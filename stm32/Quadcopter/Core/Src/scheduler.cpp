/*
 * scheduler.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

// Includes from STL
#include <string.h>  // Include for memcpy

// Includes from Project
#include "scheduler.hpp"
#include "logManager.hpp"
#include "PID/controlStrategy.hpp"
#include "Utils/utilsAlgebra.hpp"
#include "PWM/readRadio.hpp"
#include "stateMachine.hpp"

// screen /dev/tty.usbserial-14220 115200


// SCHEDULER
// Loops' frequency definitions
#define AHRS_FREQUENCY_LOOP 6000.0
#define AHRS_FREQUENCY_LOOP_PERIODE (1.0/AHRS_FREQUENCY_LOOP)

#define PID_RATE_FREQUENCY_LOOP 2000.0
#define PID_RATE_FREQUENCY_LOOP_PERIODE (1.0/PID_RATE_FREQUENCY_LOOP)

#define PID_ANGLE_FREQUENCY_LOOP 500.0
#define PID_ANGLE_FREQUENCY_LOOP_PERIODE (1.0/PID_ANGLE_FREQUENCY_LOOP)

#define PID_POS_FREQUENCY_LOOP 50.0
#define PID_POS_FREQUENCY_LOOP_PERIODE (1.0/PID_POS_FREQUENCY_LOOP)

#define ESCs_FREQUENCY_LOOP 480.0
#define ESCs_FREQUENCY_LOOP_PERIODE (1.0/ESCs_FREQUENCY_LOOP)

#define RADIO_FREQUENCY_LOOP 50.0
#define RADIO_FREQUENCY_LOOP_PERIODE (1.0/RADIO_FREQUENCY_LOOP)



#define ROLLPITCH_ATT_KP 1.0f
#define ROLLPITCH_ATT_KI 1.0f
#define ROLLPITCH_ATT_KD 1.0f

#define YAW_ATT_KP 1.0f
#define YAW_ATT_KI 1.0f
#define YAW_ATT_KD 1.0f

// Radio control
#define THROTTLE_HOVER_OFFSET 0.1 // Around hover point
#define THROTTLE_EXPO 0.99
#define TARGET_ANGLE_MAX 45.0


// TODOs:
// Offset hover quaternion
// Set hover thrust offset precisely
// Vector control
// PIDs & map to motors
// Rename Quadcopter -> Scheduler
// Remove Eigen


Scheduler::Scheduler(
		SPI_HandleTypeDef hspi,
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port,
		UART_HandleTypeDef uart_ext,
		TIM_HandleTypeDef& htim1
		) :
		m_huart_ext(uart_ext),
		m_htim1(htim1),
		m_radio(THROTTLE_HOVER_OFFSET, THROTTLE_EXPO, TARGET_ANGLE_MAX)
{

}



/*
 * Called once at the beginning of the software
 */
void Scheduler::mainSetup()
{
	// Setup the serial print
	LogManager::getInstance().setup(m_huart_ext);

	// Setup the IMU (ICM20948)
	icm20948_init(); // Accelerometer & gyroscope
	//ak09916_init();  // Magnetometer

	// Init AHRS
	m_madgwickFilter = MadgwickFilter();

	// Calibrate IMU
	gyroAccelCalibration();

	// Setup PWM reading for radio receiver
	setupRadio();

	// Set Startup state
	StateMachine::getInstance().setState(StateMachine::getInstance().getStartupSequenceState());
}


/*
 * Called indefinitely in a loop
 * This is the main loop of this software
 */
void Scheduler::mainLoop(const double dt)
{
	// Highest frequency loop (~6khz): IMU read & AHRS quaternion update (Madgwick)
	// Medium frequency loop (~2khz): Rate or attitude PID
	// Low frequency loop (~500hz): PWM motor update
	// Very low frequency loop: (~50hz): Position hold PID

	m_pidRateLoopDt += dt;
	m_pidAngleLoopDt += dt;
	m_pidPosLoopDt += dt;
	m_escsLoopDt += dt;
	m_radioLoopDt += dt;

	// Fast loop (6khz)

	// Read IMU
	icm20948_gyro_read_dps(&m_gyro);
	icm20948_accel_read_g(&m_accel);
	//bool readMag = ak09916_mag_read_uT(&m_mag);

	Eigen::Vector3f gyro;
	Eigen::Vector3f acc;
	gyro << m_gyro.x, m_gyro.y, m_gyro.z;
	acc << m_accel.x, m_accel.y, m_accel.z;
	gyro -= m_gyroOffset;
	acc -= m_accelOffset;

	// Debug print IMU data
	/*LogManager::getInstance().serialPrint(
		acc.x(), acc.y(), acc.z(),
		gyro.x(), gyro.y(), gyro.z(),
		0.0, 0.0, 0.0
		//calibratedMag.x(), calibratedMag.y(), calibratedMag.z()
		);*/

	// AHRS, Madgwick filter
	m_madgwickFilter.compute(
		acc.x(), // Acceleration vector will be normalized
		acc.y(),
		acc.z(),
		gyro.x() * DEGREE_TO_RAD,
		gyro.y() * DEGREE_TO_RAD,
		gyro.z() * DEGREE_TO_RAD,
		dt
		);

	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst);

	// Read radio
	if (m_radioLoopDt > RADIO_FREQUENCY_LOOP_PERIODE)
	{
		const bool signalLost = m_radio.readRadioReceiver(true, m_radioLoopDt);

		if (!signalLost)
		{
			//LogManager::getInstance().serialPrint(m_radio.m_targetRoll, m_radio.m_targetPitch, m_radio.m_targetYaw, m_radio.m_targetThrust);

			// Compute target quaternion
			m_targetAttitude = Quaternion::fromEuler(m_radio.m_targetRoll * DEGREE_TO_RAD, m_radio.m_targetPitch * DEGREE_TO_RAD, m_radio.m_targetYaw * DEGREE_TO_RAD);
			// Debug print AHRS result
			//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_targetAttitude);
		}
		else
		{
			// Signal lost, target quaternion is horizon
		}

		m_radioLoopDt = 0.0;
	}

	// Enable angle loop (that will run in the state machine)
	if (m_pidAngleLoopDt > PID_ANGLE_FREQUENCY_LOOP_PERIODE)
	{
		if(m_ctrlStrat.m_flightMode == StabilizationMode::STAB ||
				m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
		{
			m_angleLoop = true;
		}
		m_pidAngleLoopDt = 0.0;
	}

	// Enable position hold loop (that will run in the state machine)
	if (m_pidPosLoopDt > PID_POS_FREQUENCY_LOOP_PERIODE)
	{
		if (m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
		{
			m_posLoop = true;
		}
		m_pidPosLoopDt = 0.0;
	}

	// Handle drone behavior according to the current state
	if (m_pidRateLoopDt > PID_RATE_FREQUENCY_LOOP_PERIODE)
	{
		StateMachine::getInstance().run(m_pidRateLoopDt, *this);
		m_pidRateLoopDt = 0.0;

		// Reset angle & position hold flag here because they are executed in the state machine
		m_angleLoop = false;
		m_posLoop = false;
	}

	// Motors update
	if (m_escsLoopDt > ESCs_FREQUENCY_LOOP_PERIODE)
	{
		// PWM update
		m_motorMixer.mixThrustTorque(m_thrust, m_torqueX, m_torqueY, m_torqueZ);
		m_motorMixer.clampRescale();

		setMotorPower(Motor::eMotor1, m_motorMixer.m_powerMotor[0]);
		setMotorPower(Motor::eMotor2, m_motorMixer.m_powerMotor[1]);
		setMotorPower(Motor::eMotor3, m_motorMixer.m_powerMotor[2]);
		setMotorPower(Motor::eMotor4, m_motorMixer.m_powerMotor[3]);

		m_escsLoopDt = 0.0;
	}
}


void Scheduler::gyroAccelCalibration()
{
	Eigen::Vector3f gyro = Eigen::Vector3f(0.0, 0.0, 0.0);
	Eigen::Vector3f accel = Eigen::Vector3f(0.0, 0.0, 0.0);

	const int nbIteration = 200;

	for (int i = 0; i < nbIteration; ++i)
	{
		// Read IMU
		icm20948_gyro_read_dps(&m_gyro);
		icm20948_accel_read_g(&m_accel);

		gyro += Eigen::Vector3f(m_gyro.x, m_gyro.y, m_gyro.z);
		accel += Eigen::Vector3f(m_accel.x, m_accel.y, m_accel.z);

		HAL_Delay(10);
	}

	m_gyroOffset = gyro / (float)nbIteration;
	accel /= (float)nbIteration;

	// Find the g vector and normalize it
	float norm = accel.norm();
	Eigen::Vector3f g = accel / norm;

	// Remove the gravity vector from the offset
	m_accelOffset = accel - g;
}


/*
 * Set PWM's high time to control ESCs.
 * power = [0.0, 1.0]
 */
void Scheduler::setMotorPower(const Motor motor, const double power)
{
	constexpr double pwmRes = 500.0;
	constexpr int pwmResMin = static_cast<int>(pwmRes);
	constexpr int pwmResMax = 2*pwmResMin;

	int high = static_cast<int>(pwmRes + power * pwmRes);
	if (high < pwmResMin)
	{
		high = pwmResMin;
	}
	else if (high > pwmResMax)
	{
		high = pwmResMax;
	}

	switch(motor)
	{
	case Motor::eMotor1:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_1, high);
		break;

	case Motor::eMotor2:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_2, high);
		break;

	case Motor::eMotor3:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_3, high);
		break;

	case Motor::eMotor4:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_4, high);
		break;

	default:
		break;
	}
}


/*
 * Calibrate magnetometer.
 * Print the "center of the sphere" over UART.
 * Run this function and move the IMU in every direction.
 * Function to run once.
 */
void magnetometerCalibration()
{
	axises mag;
	Eigen::Vector3f magMin( 1e6,  1e6,  1e6);
	Eigen::Vector3f magMax(-1e6, -1e6, -1e6);

	while(1)
	{
		HAL_Delay(10);

		bool readMag = ak09916_mag_read_uT(&mag);
		if (!readMag)
		{
			continue;
		}

		Eigen::Vector3f m = Eigen::Vector3f(mag.x, mag.y, mag.z);
		magMin = magMin.cwiseMin(m);
		magMax = magMax.cwiseMax(m);

		// Compute of the center of the sphere
		Eigen::Vector3f bias = (magMax + magMin) * 0.5f;
		float B = (magMax - magMin).norm() * 0.5f;

		// Print result
		char pBuffer[256];
		sprintf(pBuffer,
			"%4.4f, %4.4f, %4.4f, %4.4f\r\n",
			bias.x(), bias.y(), bias.z(), B);
		LogManager::getInstance().serialPrint(pBuffer);
	}
}


