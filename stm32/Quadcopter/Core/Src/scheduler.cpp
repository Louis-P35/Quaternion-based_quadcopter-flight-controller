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
#include "PWM/readRadio.hpp"
#include "stateMachine.hpp"

// screen /dev/tty.usbserial-14220 115200

#define DEGREE_TO_RAD (M_PI/180.0)


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


/*
 * PIDs coeff
 */

#define SATURATION 1000.0
#define MAX_OUT 1000.0
#define MIN_OUT -1000.0

#define ROLLPITCH_ANGLE_KP 0.0
#define ROLLPITCH_ANGLE_KI 0.0
#define ROLLPITCH_ANGLE_KD 0.0

#define YAW_ANGLE_KP 0.0
#define YAW_ANGLE_KI 0.0
#define YAW_ANGLE_KD 0.0

#define ROLLPITCH_RATE_KP 0.001
#define ROLLPITCH_RATE_KI 0.0
#define ROLLPITCH_RATE_KD 0.0

#define YAW_RATE_KP 0.001
#define YAW_RATE_KI 0.0
#define YAW_RATE_KD 0.0

#define ROLLPITCH_POS_KP 0.0
#define ROLLPITCH_POS_KI 0.0
#define ROLLPITCH_POS_KD 0.0

#define YAW_POS_KP 0.0
#define YAW_POS_KI 0.0
#define YAW_POS_KD 0.0

// Radio control
#define THROTTLE_HOVER_OFFSET 0.1 // Around hover point
#define THROTTLE_EXPO 0.99
#define TARGET_ANGLE_MAX 45.0
#define TARGET_RATE_MAX 200.0


//#define COMPUTE_HOVER_OFFSET 1

// Uncomment this to disable motors
//#define DEBUG_DISABLE_MOTORS 1


// TODOs:
// Set hover thrust offset precisely
// PIDs & map to motors
// Remove Eigen
// Define PID values
// Keep yaw in rates
// Compute target quaternion only in STAB/HORIZON(& stick 0) mode


Scheduler::Scheduler(
		SPI_HandleTypeDef hspi,
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port,
		UART_HandleTypeDef uart_ext,
		TIM_HandleTypeDef& htim1
		) :
		m_huart_ext(uart_ext),
		m_htim1(htim1),
		m_radio(THROTTLE_HOVER_OFFSET, THROTTLE_EXPO, TARGET_ANGLE_MAX, TARGET_RATE_MAX)
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
	m_madgwickFilter = MadgwickFilter<double>();

	// Calibrate IMU
	gyroAccelCalibration();

	// Setup PWM reading for radio receiver
	setupRadio();

	// Set Startup state
	StateMachine::getInstance().setState(StateMachine::getInstance().getStartupSequenceState());

	// Set PIDs coefficients
	m_ctrlStrat.setPIDsatMinMax(SATURATION, MIN_OUT, MAX_OUT);

	m_ctrlStrat.setRatePIDcoefsRoll(ROLLPITCH_RATE_KP, ROLLPITCH_RATE_KI, ROLLPITCH_RATE_KD);
	m_ctrlStrat.setRatePIDcoefsPitch(ROLLPITCH_RATE_KP, ROLLPITCH_RATE_KI, ROLLPITCH_RATE_KD);
	m_ctrlStrat.setRatePIDcoefsYaw(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD);

	m_ctrlStrat.setAnglePIDcoefsRoll(ROLLPITCH_ANGLE_KP, ROLLPITCH_ANGLE_KI, ROLLPITCH_ANGLE_KD);
	m_ctrlStrat.setAnglePIDcoefsPitch(ROLLPITCH_ANGLE_KP, ROLLPITCH_ANGLE_KI, ROLLPITCH_ANGLE_KD);
	m_ctrlStrat.setAnglePIDcoefsYaw(YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_KD);

	m_ctrlStrat.setPosPIDcoefsRoll(ROLLPITCH_POS_KP, ROLLPITCH_POS_KI, ROLLPITCH_POS_KD);
	m_ctrlStrat.setPosPIDcoefsPitch(ROLLPITCH_POS_KP, ROLLPITCH_POS_KI, ROLLPITCH_POS_KD);
	m_ctrlStrat.setPosPIDcoefsYaw(YAW_POS_KP, YAW_POS_KI, YAW_POS_KD);
}


/*
 * Called indefinitely in a loop
 * This is the main loop of this software
 */
void Scheduler::mainLoop(const double dt)
{
	// Highest frequency loop (~6khz): IMU read & AHRS quaternion update (Madgwick)
	// Medium frequency loop (~2khz): Rate PID (att PID at 500 hz)
	// Low frequency loop (~500hz): PWM motor update
	// Very low frequency loop: (~50hz): Position hold PID & read radio

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

	// Remove accel & gyro's offset
	m_calibratedGyro = {static_cast<double>(m_gyro.x), static_cast<double>(m_gyro.y), static_cast<double>(m_gyro.z)};
	m_calibratedGyro -= m_gyroOffset;
	m_calibratedAccel = {static_cast<double>(m_accel.x), static_cast<double>(m_accel.y), static_cast<double>(m_accel.z)};
	m_calibratedAccel -= m_accelOffset;

	m_sumGyro += m_calibratedGyro;
	m_nbSumGyro++;

	// AHRS, Madgwick filter
	m_madgwickFilter.compute(
			m_calibratedAccel.m_x, // Acceleration vector will be normalized
			m_calibratedAccel.m_y,
			m_calibratedAccel.m_z,
			m_calibratedGyro.m_x * DEGREE_TO_RAD,
			m_calibratedGyro.m_y * DEGREE_TO_RAD,
			m_calibratedGyro.m_z * DEGREE_TO_RAD,
			dt
		);

	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_madgwickFilter.m_qEst);
	//m_qAttitudeCorrected = m_qHoverOffset * m_madgwickFilter.m_qEst;
	//m_qAttitudeCorrected.normalize();
	//LogManager::getInstance().serialPrint(m_qAttitudeCorrected, m_madgwickFilter.m_qEst);


#ifdef COMPUTE_HOVER_OFFSET
	// Compute the hover offset (must be done once after each teardown/build of the drone)
	calibrateHoverOffset();
#endif

	// Read radio
	if (m_radioLoopDt > RADIO_FREQUENCY_LOOP_PERIODE)
	{
		const bool signalLost = m_radio.readRadioReceiver(true, m_radioLoopDt);

		if (!signalLost)
		{
			//LogManager::getInstance().serialPrint(m_radio.m_targetRoll, m_radio.m_targetPitch, m_radio.m_targetYaw, m_radio.m_targetThrust);
			//LogManager::getInstance().serialPrint(m_radio.m_targetRateRoll, m_radio.m_targetRatePitch, m_radio.m_targetRateYaw, m_radio.m_targetThrust);

			// Compute target quaternion
			m_targetAttitude = Quaternion<double>::fromEuler(m_radio.m_targetRoll * DEGREE_TO_RAD, m_radio.m_targetPitch * DEGREE_TO_RAD, m_radio.m_targetYaw * DEGREE_TO_RAD);
			// Debug print AHRS result
			//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_targetAttitude);
		}
		else
		{
			// Signal lost, target quaternion is horizon
		}

		//LogManager::getInstance().serialPrint(m_motorMixer.m_powerMotor[1], m_motorMixer.m_powerMotor[2], 0.0, 0.0);
		//LogManager::getInstance().serialPrint(m_motorMixer.m_powerMotor[0], m_motorMixer.m_powerMotor[3], 0.0, 0.0);
		//LogManager::getInstance().serialPrint("\n\r");

		//LogManager::getInstance().serialPrint(m_averagedGyro.m_x, m_averagedGyro.m_y, m_averagedGyro.m_z, 0.0);
		// Reset dt
		m_radioLoopDt = 0.0;
	}

	// Enable PID angle loop (that will run in the state machine)
	if (m_pidAngleLoopDt > PID_ANGLE_FREQUENCY_LOOP_PERIODE)
	{
		// Enable it only in certain flight mode
		if(m_ctrlStrat.m_flightMode == StabilizationMode::STAB ||
				m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
		{
			m_angleLoop = true;
		}
	}

	// Enable PID position hold loop (that will run in the state machine)
	if (m_pidPosLoopDt > PID_POS_FREQUENCY_LOOP_PERIODE)
	{
		// Enable it only in certain flight mode
		if (m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
		{
			m_posLoop = true;
		}
	}

	// Handle drone behavior according to the current state (state machine)
	// Run all the PID loops
	if (m_pidRateLoopDt > PID_RATE_FREQUENCY_LOOP_PERIODE)
	{
		// Compute the average gyro value between 2 rate loop
		m_averagedGyro = m_sumGyro / static_cast<double>(m_nbSumGyro);
		m_nbSumGyro = 0;
		m_sumGyro = {0.0, 0.0, 0.0};

		// Run the state machine
		StateMachine::getInstance().run(m_pidRateLoopDt, *this);

		// Reset dt
		m_pidRateLoopDt = 0.0;
		if (m_posLoop)
		{
			m_pidPosLoopDt = 0.0;
		}
		if (m_angleLoop)
		{
			m_pidAngleLoopDt = 0.0;
		}

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

#ifndef DEBUG_DISABLE_MOTORS
		setMotorPower(Motor::eMotor1, m_motorMixer.m_powerMotor[0]);
		setMotorPower(Motor::eMotor2, m_motorMixer.m_powerMotor[1]);
		setMotorPower(Motor::eMotor3, m_motorMixer.m_powerMotor[2]);
		setMotorPower(Motor::eMotor4, m_motorMixer.m_powerMotor[3]);
#endif

		// Reset dt
		m_escsLoopDt = 0.0;
	}
}


/*
 * Find the gyroscope and accelerometer offsets for each axis.
 */
void Scheduler::gyroAccelCalibration()
{
	Vector3<double> gyro = Vector3<double>(0.0, 0.0, 0.0);
	Vector3<double> accel = Vector3<double>(0.0, 0.0, 0.0);

	const int nbIteration = 200;

	for (int i = 0; i < nbIteration; ++i)
	{
		// Read IMU
		icm20948_gyro_read_dps(&m_gyro);
		icm20948_accel_read_g(&m_accel);

		gyro += Vector3<double>(m_gyro.x, m_gyro.y, m_gyro.z);
		accel += Vector3<double>(m_accel.x, m_accel.y, m_accel.z);

		HAL_Delay(10);
	}

	m_gyroOffset = gyro / static_cast<double>(nbIteration);
	accel /= static_cast<double>(nbIteration);

	// Find the g vector and normalize it
	double norm = accel.norm();
	Vector3<double> g = accel / norm;

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

	// Mapping enum to real motors
	switch(motor)
	{
	case Motor::eMotor1:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_4, high);
		break;

	case Motor::eMotor2:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_1, high);
		break;

	case Motor::eMotor3:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_2, high);
		break;

	case Motor::eMotor4:
		__HAL_TIM_SET_COMPARE(&m_htim1, TIM_CHANNEL_3, high);
		break;

	default:
		break;
	}
}

/*
 * Calibrate the IMU orientation.
 * Because the IMU is never solder and mounted perfectly flat on the drone.
 * Just print out the result over UART.
 */
void Scheduler::calibrateHoverOffset()
{
	static constexpr int nbPassMinInitAhrs = 30000;
	static constexpr int nbIterMax = 500;
	static bool computeDone = false;
	static bool print = true;
	static double sumRoll = 0.0;
	static double sumPitch = 0.0;
	static int nbIter = 0;
	static int nbPass = 0;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;

	// Let some time for the AHRS to stabilize
	nbPass++;
	if (nbPass < nbPassMinInitAhrs)
	{
		return;
	}

	if (nbIter < nbIterMax)
	{
		m_madgwickFilter.m_qEst.toEuler(roll, pitch, yaw);
		sumRoll += roll;
		sumPitch += pitch;
		nbIter++;
	}
	else if (!computeDone)
	{
		double avgRoll = sumRoll / static_cast<double>(nbIter);
		double avgPicth = sumPitch / static_cast<double>(nbIter);
		LogManager::getInstance().serialPrint("Roll, Pitch (degree):\n\r");
		LogManager::getInstance().serialPrint(avgRoll, avgPicth, 0.0, 0.0);
		Quaternion<double> qAverage;
		qAverage = Quaternion<double>::fromEuler(
				avgRoll * DEGREE_TO_RAD,
				avgPicth * DEGREE_TO_RAD,
				0.0
				);

		// Compute the offset to add to m_madgwickFilter.m_qEst
		m_qHoverOffset = qAverage.inverse();
		m_qHoverOffset.normalize();

		computeDone = true;
	}
	else if (print)
	{
		LogManager::getInstance().serialPrint("m_qHoverOffset:\n\r");
		LogManager::getInstance().serialPrint(m_qHoverOffset);
		print = false;
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


