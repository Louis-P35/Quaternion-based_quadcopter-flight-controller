/*
 * flightCore.cpp
 *
 *  Created on: Jul 10, 2025
 *      Author: louis
 */


// Includes from project
#include "config.h"
#include "flightCore.hpp"
#include "Scheduler/scheduler.hpp"
#include "logManager.hpp"
#include "PID/controlStrategy.hpp"
#include "PID/pid.hpp"
#include "Radio/radio.hpp"
#include "FSM/stateMachine.hpp"
#include "Sensors/mtf01WrapperC.h"

#ifndef DISABLE_UNIT_TESTING
#include "UnitTests/schedulerUnitTests.hpp"
#endif

// Includes from STL
#include <algorithm>


#define DEGREE_TO_RAD (M_PI/180.0)
#define RAD_TO_DEG (180.0/M_PI)


/*
 * PIDs coeff
 */

#define SATURATION (75.0f)
#define MAX_OUT (1000.0f)
#define MIN_OUT (-1000.0f)

#define ROLL_PITCH_RATE_MAX_D_PERCENT (0.75f) // < 1 for stability, [1, 2] for aggresivity

// Attitude loop PIDs coefficients
#define ROLLPITCH_ANGLE_KP (8.0f)
#define ROLLPITCH_ANGLE_KI (0.0f)
#define ROLLPITCH_ANGLE_KD (0.0f)

#define YAW_ANGLE_KP (ROLLPITCH_ANGLE_KP / 2.0f)
#define YAW_ANGLE_KI (0.0f)
#define YAW_ANGLE_KD (0.0f)

// Rate loop PIDs coefficients
#define ROLLPITCH_RATE_KP (0.4f)
#define ROLLPITCH_RATE_KI (0.4f)
#define ROLLPITCH_RATE_KD (0.04f)

#define YAW_RATE_KP (0.3f)
#define YAW_RATE_KI (0.01f)
#define YAW_RATE_KD (0.0f)

// Position loop PIDs coefficients
#define ROLLPITCH_POS_KP (0.0f)
#define ROLLPITCH_POS_KI (0.0f)
#define ROLLPITCH_POS_KD (0.0f)

#define YAW_POS_KP (0.0f)
#define YAW_POS_KI (0.0f)
#define YAW_POS_KD (0.0f)

// Radio control
#define THROTTLE_HOVER_OFFSET (0.1f) // Around hover point
#define THROTTLE_EXPO (0.99f)
#define TARGET_ANGLE_MAX (45.0f)
#define TARGET_RATE_MAX (200.0f)
#define THRUST_IS_FLYING_THRESHOLD_UP (340.0f)
#define THRUST_IS_FLYING_THRESHOLD_DOWN (250.0f)


extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern ADC_HandleTypeDef hadc3;


extern Scheduler g_scheduler;
extern FlightCore* g_pFlightCore;


volatile bool g_enableRadioLoop = false;
volatile bool g_start = false;

volatile bool g_startRecord = false;
volatile bool g_startPrint = false;

//#define CALIBRATE_IMU (1)

// 0.45 (x4) = take off thrust

//#define COMPUTE_HOVER_OFFSET (1)

// Uncomment this to disable motors
//#define DEBUG_DISABLE_MOTORS (1)
//#define PID_TESTING_MODE (1)





// TODOs:
// GYRO à 6khz => voir ce que ça donne de monter les fréquences de coupure des 2 LPF (moins de latences)
// 		=> Test avec un biquad Butterworth 2ᵉ ordre à la place des 2 LPF chainé (moins de latences)

// PID d terme chainer 2 LPF et monter la fréquence de coupure (moins de latence)

// PID D terd on derived gyro instead of derived error
// Compute frequency and do not /dt in pid
// Set PID coefs at hover point
// Set hover thrust offset precisely
// Remove Eigen
// Keep yaw in rates
// Compute target quaternion only in STAB/HORIZON(& stick 0) mode



FlightCore::FlightCore(uint16_t spi_cs_pin, GPIO_TypeDef* spi_cs_gpio_port,
		uint16_t esp_cs_pin, GPIO_TypeDef* esp_cs_gpio_port) :
		m_radio(THROTTLE_HOVER_OFFSET, THROTTLE_EXPO, TARGET_ANGLE_MAX, TARGET_RATE_MAX),
		m_espInterface(hspi1, esp_cs_gpio_port, esp_cs_pin)
{

}


/*
 * Called once at the beginning of the software.
 */
void FlightCore::mainSetup()
{
	constexpr float pidAngleOutputCutOffFreq = 15.0f;

	// Setup the serial print
	LogManager::getInstance().setup();

	// Setup the IMU (ICM20948)
	m_imu.init(4000.0f); // TODO hardcoded... // CODE PLANTE ICI sans etre branché ??
	//escLoop();

	// Init AHRS
	m_madgwickFilter = MadgwickFilter<float>();

	// Calibrate IMU
#ifdef CALIBRATE_IMU
	m_imu.gyroAccelCalibration();
#else
	m_imu.setGyroOffset(Vector3<float>(0.0010f, -0.0005f, 0.0196f));
	m_imu.setAccelOffset(Vector3<float>(0.0067, 0.0277, -0.0146));
#endif

	// Setup PWM reading for radio receiver
	//setupRadio();

#if SENSOR_MTF01_ENABLED
	// Setup optical flow sensor
	mtf01WrapperSetInstance(static_cast<void*>(&m_opticalflow));
#endif

	// Set Startup state
	MainStateMachine::getInstance().setState(MainStateMachine::getInstance().getStartupSequenceState());

	// Configure PIDs
	m_ctrlStrat.setRatePIDderivativeMode(DerivativeMode::OnMeasurement);
	m_ctrlStrat.setRatePIDcoefsRoll(ROLLPITCH_RATE_KP, ROLLPITCH_RATE_KI, ROLLPITCH_RATE_KD);
	m_ctrlStrat.setRatePIDcoefsPitch(ROLLPITCH_RATE_KP, ROLLPITCH_RATE_KI, ROLLPITCH_RATE_KD);
	m_ctrlStrat.setRatePIDcoefsYaw(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD);
	m_ctrlStrat.setPIDsatMinMaxRate(SATURATION, MIN_OUT, MAX_OUT, ROLL_PITCH_RATE_MAX_D_PERCENT);

	m_ctrlStrat.setAnglePIDderivativeMode(DerivativeMode::OnError);
	m_ctrlStrat.setAnglePIDcoefsRoll(ROLLPITCH_ANGLE_KP, ROLLPITCH_ANGLE_KI, ROLLPITCH_ANGLE_KD);
	m_ctrlStrat.setAnglePIDcoefsPitch(ROLLPITCH_ANGLE_KP, ROLLPITCH_ANGLE_KI, ROLLPITCH_ANGLE_KD);
	m_ctrlStrat.setAnglePIDcoefsYaw(YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_KD);
	m_ctrlStrat.setPIDsatMinMaxAngle(SATURATION, MIN_OUT, MAX_OUT);

	m_ctrlStrat.setPosPIDderivativeMode(DerivativeMode::OnError);
	m_ctrlStrat.setPosPIDcoefsRoll(ROLLPITCH_POS_KP, ROLLPITCH_POS_KI, ROLLPITCH_POS_KD);
	m_ctrlStrat.setPosPIDcoefsPitch(ROLLPITCH_POS_KP, ROLLPITCH_POS_KI, ROLLPITCH_POS_KD);
	m_ctrlStrat.setPosPIDcoefsYaw(YAW_POS_KP, YAW_POS_KI, YAW_POS_KD);
	m_ctrlStrat.setPIDsatMinMaxPos(SATURATION, MIN_OUT, MAX_OUT);

	// D term filters for PID rate loop
	constexpr float rateLoopFreq = 2000.0f; // TODO hardcoded...
	constexpr float angleLoopFreq = 1000.0f; // TODO hardcoded...
	constexpr float paramsPIDAngleCutOff[1] = {pidAngleOutputCutOffFreq};

	for (size_t i = 0; i < 3; ++i)
	{
		m_ctrlStrat.m_rateLoop[i].m_dTermLpf.init(rateLoopFreq, 10.0f);
		m_ctrlStrat.m_rateLoop[i].m_dTermLpf2.init(rateLoopFreq, 10.0f);
		m_ctrlStrat.m_rateLoop[i].m_ffTermLpf.init(rateLoopFreq, 50.0f);

		m_ctrlStrat.m_angleLoop[i].m_filteredOutput.init(angleLoopFreq, paramsPIDAngleCutOff);
	}

	// Set the control mode
	m_ctrlStrat.m_flightMode = StabilizationMode::STAB;

	// Enable PID angle loop
	// Enable it only in certain flight mode
	if(m_ctrlStrat.m_flightMode == StabilizationMode::STAB ||
			m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
	{
		m_angleLoopEnable = true; // TODO: Need to be done in the state machine
	}

	// Enable PID position hold loop (that will run in the state machine)
	// Enable it only in certain flight mode
	if (g_pFlightCore->m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
	{
		g_pFlightCore->m_posLoopEnable = true; // TODO: Need to be done in the state machine
	}


	// Setup the tasks
	// addAllTasks(g_scheduler); // Unit test

	bool taskAddSuccess = true;

	// 4 KHz tasks
	taskAddSuccess &= g_scheduler.addTask(TaskType::eRead_IMU, 0, readIMU_task, FREQUENCY_SLOT::e_4KHZ);

	// 2 KHz tasks
	taskAddSuccess &= g_scheduler.addTask(TaskType::ePID_rate, 0, pidRate_task, FREQUENCY_SLOT::e_2KHZ);

	// 1 KHz tasks
	taskAddSuccess &= g_scheduler.addTask(TaskType::eAHRS, 0, AHRS_task, FREQUENCY_SLOT::e_1KHZ);
	taskAddSuccess &= g_scheduler.addTask(TaskType::ePID_att, 1, pidAtt_task, FREQUENCY_SLOT::e_1KHZ);

	// 500 Hz tasks
	taskAddSuccess &= g_scheduler.addTask(TaskType::eESCs, 0, ESCs_task, FREQUENCY_SLOT::e_500HZ);

	// 100 Hz tasks
#if SENSOR_MTF01_ENABLED
	taskAddSuccess &= g_scheduler.addTask(TaskType::eRead_opticalFlow, 0, readOpticalFlow_task, FREQUENCY_SLOT::e_100HZ);
#endif
	taskAddSuccess &= g_scheduler.addTask(TaskType::ePID_pos, 1, pidPos_task, FREQUENCY_SLOT::e_100HZ);

	// 50 Hz tasks
	taskAddSuccess &= g_scheduler.addTask(TaskType::eRead_radio, 0, readRadio_task, FREQUENCY_SLOT::e_50HZ);
	taskAddSuccess &= g_scheduler.addTask(TaskType::eMain_fsm, 1, mainFSM_task, FREQUENCY_SLOT::e_50HZ);

	// 10 Hz tasks
	taskAddSuccess &= g_scheduler.addTask(TaskType::eRead_battery, 0, readBattery_task, FREQUENCY_SLOT::e_10HZ);
	taskAddSuccess &= g_scheduler.addTask(TaskType::eDebugPrint, 1, debugPrint_task, FREQUENCY_SLOT::e_10HZ);

	// Error
	if (!taskAddSuccess)
	{
		// TODO: Handle task add error
	}

	// Start the loop
	g_start = true;
}


/*
 * Task read and filter IMU.
 * C wrapper function.
 */
void readIMU_task(const float& dt)
{
	g_pFlightCore->m_imu.readAndFilterIMU_gdps();
}


/*
 * Compute the AHRS (Madgwick filter).
 * C wrapper function.
 */
void AHRS_task(const float& dt)
{
	g_pFlightCore->ahrsLoop(dt);
}


/*
 * Send command signals to ESCs.
 * C wrapper function.
 */
void ESCs_task(const float& dt)
{
	g_pFlightCore->escLoop();
}


/*
 * Run the PID position (xyz).
 * C wrapper function.
 */
void pidPos_task(const float& dt)
{
	g_pFlightCore->pidPosLoop(dt);
}


/*
 * Run the PID attitude (angle).
 * C wrapper function.
 */
void pidAtt_task(const float& dt)
{
	g_pFlightCore->pidAttLoop(dt);
}


/*
 * Run the PID rate.
 * C wrapper function.
 */
void pidRate_task(const float& dt)
{
	g_pFlightCore->pidRateLoop(dt);
}


/*
 * Run the main finite state machine.
 * C wrapper function.
 */
void mainFSM_task(const float& dt)
{
	MainStateMachine::getInstance().run(dt);
}


/*
 *
 * C wrapper function.
 */
void subFSM_task(const float& dt)
{

}


/*
 * Read battery task.
 * Read the battery voltage with ADC.
 * C wrapper function.
 */
void readBattery_task(const float& dt)
{
	g_pFlightCore->batteryLoop();
}


/*
 * Read radio receiver task.
 * C wrapper function.
 */
void readRadio_task(const float& dt)
{
	g_pFlightCore->radioLoop(dt);
}


/*
 * Read the optical flow and lidar sensor (MTF-01).
 * C wrapper function.
 */
void readOpticalFlow_task(const float& dt)
{
	g_pFlightCore->m_opticalflow.readSensor();
}


/*
 * Print over UART.
 * C wrapper function.
 */
void debugPrint_task(const float& dt)
{
	g_pFlightCore->debugPrintLoop();
}



/*
 * Read battery.
 * Read the battery voltage with ADC.
 */
void FlightCore::batteryLoop()
{
	// Read battery voltage
	// It is a blocking function !!
	m_batteryVoltage = readBatteryVoltage();

	// Compute voltage compensation
	m_motorMixer.computeVoltageCompensation(m_batteryVoltage);
}


/*
 * Print over UART.
 */
void FlightCore::debugPrintLoop()
{
	static uint8_t phase = 0;

	const Quaternion<float>& q = m_madgwickFilter.m_qEst;
	const Vector3<float>&   g = m_imu.m_gyroFilterAhrs;
	const Vector3<float>&   a = m_imu.m_accelFilterAhrs;

	switch (phase)
	{
		case 0:
			m_espInterface.sendAttitude(
				q.m_w, q.m_x, q.m_y, q.m_z,
				g.m_x, g.m_y, g.m_z,
				a.m_x, a.m_y, a.m_z);
			break;

		case 1:
		{
			uint8_t motorPct[4];
			for (int i = 0; i < 4; ++i)
				motorPct[i] = static_cast<uint8_t>(m_motorMixer.m_powerMotor[i] / 10.0f);
			m_espInterface.sendStatus(
				MainStateMachine::getInstance().getStateName(),
				m_batteryVoltage, 0.0f, 0, motorPct, 4);
			break;
		}

		case 2:
		{
			uint16_t rcChannels[16] = {};
			for (int i = 0; i < 16; ++i)
				rcChannels[i] = m_radio.m_radioProtocole.getChannelUs(i);
			m_espInterface.sendRc(rcChannels, 16);
			break;
		}
	}

	phase = (phase + 1) % 3;
}


/*
 * Run Madgwick filter.
 */
void FlightCore::ahrsLoop(const float& dt)
{
	// AHRS, Madgwick filter
	m_madgwickFilter.compute(
			m_imu.m_accelFilterAhrs.m_x, // Acceleration vector will be normalized
			m_imu.m_accelFilterAhrs.m_y,
			m_imu.m_accelFilterAhrs.m_z,
			m_imu.m_gyroFilterAhrs.m_x * DEGREE_TO_RAD,
			m_imu.m_gyroFilterAhrs.m_y * DEGREE_TO_RAD,
			m_imu.m_gyroFilterAhrs.m_z * DEGREE_TO_RAD,
			dt
		);

	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_madgwickFilter.m_qEst);


#ifdef COMPUTE_HOVER_OFFSET
	// Compute the hover offset (must be done once after each teardown/build of the drone)
	calibrateHoverOffset();
#endif
}


/*
 * Run the PID rate.
 */
void FlightCore::pidRateLoop(const float& dt)
{
	if (!m_rateLoopEnable)
	{
		return;
	}

	// Run rate PID
	m_ctrlStrat.rateControlLoop(
			dt,
			m_imu.m_gyroFilterRates,
			m_setPoint
			);

	m_thrust = m_radio.m_targetThrust * 4.0f;
	m_torqueX = m_ctrlStrat.m_rateLoop[0].m_output;
	m_torqueY = m_ctrlStrat.m_rateLoop[1].m_output;
	m_torqueZ = m_ctrlStrat.m_rateLoop[2].m_output;
}


/*
 * Run the attitude (angle) PID.
 */
void FlightCore::pidAttLoop(const float& dt)
{
	if (!m_angleLoopEnable)
	{
		return;
	}

	// Correct the physical offset IMU -> drone
	m_qAttitudeCorrected = m_qHoverOffset * m_madgwickFilter.m_qEst;
	m_qAttitudeCorrected.normalize();

	// A quaternion q and -q represent the same rotation.
	// Here, canonical() make a sign choice (q.w >= 0).
	Quaternion<float> qEst = Quaternion<float>::canonical(m_qAttitudeCorrected);
	Quaternion<float> qTarget = Quaternion<float>::canonical(m_setPoint.m_targetQuaternion);

	// Get attitude error
	Quaternion<float> qError = PID::getError(qEst, qTarget);

	// Test
	//Quaternion qTest = qError * qEst;
	//qTest.normalize();

	// Get the angle and axis of rotation
	Vector3<float> rotAxis;
	float angleRad = 0.0f;
	qError.toAxisAngle(rotAxis, angleRad);

	// Projection of the rotation axis onto the 3 axis of the drone
	// It is NOT Euler angles here, so no singularity
	std::array<float, 3> error;
	error[0] = rotAxis.m_x * angleRad * RAD_TO_DEG;
	error[1] = rotAxis.m_y * angleRad * RAD_TO_DEG;
	error[2] = rotAxis.m_z * angleRad * RAD_TO_DEG;

	// Run angle PID
	m_ctrlStrat.angleControlLoop(
			dt,
			m_imu.m_gyroFilterRates,
			error,
			m_isFlying
			);
}


/*
 * Run the position (xyz) PID.
 */
void FlightCore::pidPosLoop(const float& dt)
{
	if (!m_posLoopEnable)
	{
		return;
	}

	// TODO
}


/*
 * Motors update
 */
void FlightCore::escLoop()
{
	m_motorMixer.mixThrustTorque(m_thrust, m_torqueX, m_torqueY, m_torqueZ);
	m_motorMixer.applyVoltageCompensation();
	m_motorMixer.clampRescale();

	// PWM update
#ifndef DEBUG_DISABLE_MOTORS
	setMotorPower(Motor::eMotor1, m_motorMixer.m_powerMotor[0]);
	setMotorPower(Motor::eMotor2, m_motorMixer.m_powerMotor[1]);
	setMotorPower(Motor::eMotor3, m_motorMixer.m_powerMotor[2]);
	setMotorPower(Motor::eMotor4, m_motorMixer.m_powerMotor[3]);
#endif
}


/*
 * Read radio's PWM signals
 */
void FlightCore::radioLoop(const float& dt)
{
#if RADIO_SOURCE_SPI
	{
		const EspSpi::SbusFromMiso& sbus = m_espInterface.getSbusData();
		m_radio.m_radioProtocole.feedSpiData(
			sbus.channels, sbus.frame_lost, sbus.failsafe, sbus.valid);
	}
#endif
	const bool signalLost = m_radio.readRadioReceiver(true, dt);

	// Handle is flying detection
	if (!m_isFlying && m_radio.m_targetThrust > THRUST_IS_FLYING_THRESHOLD_UP)
	{
		m_isFlying = true;
	}
	else if (m_isFlying && m_radio.m_targetThrust < THRUST_IS_FLYING_THRESHOLD_DOWN)
	{
		m_isFlying = false;
	}
}


/*
 * Called indefinitely in a loop.
 * Run the tasks scheduler.
 */
void mainLoop(const double dt)
{
	static float timeSinceBoot = 0.0f;
	timeSinceBoot += static_cast<float>(dt);

	// Run the scheduler
	g_scheduler.runTasks(timeSinceBoot);


#ifdef PID_TESTING_MODE
	g_flightCore.pidDebugStream();
#endif

	//HAL_Delay(20);


	/* DEBUG PRINT */

	/*LogManager::getInstance().serialPrint("Battery voltage: ");
	LogManager::getInstance().serialPrint(m_motorMixer.m_voltageCompensation);
	LogManager::getInstance().serialPrint("\n\r");*/

	//LogManager::getInstance().serialPrint(pidRateTicks, ahrsTicks, escTicks, radioTicks);
	//LogManager::getInstance().serialPrint("\n\r");

	//LogManager::getInstance().serialPrint(m_ctrlStrat.m_rateLoop[1].m_target);
	//LogManager::getInstance().serialPrint("\n\r");
	//LogManager::getInstance().serialPrint("COUCOU\n\r");

	//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_madgwickFilter.m_qEst);
	//LogManager::getInstance().serialPrint("\n\r");

	//m_radio.m_radioProtocole.print();
	//LogManager::getInstance().serialPrint(m_radio.m_targetRateRoll, m_radio.m_targetRatePitch, m_radio.m_targetRateYaw, m_radio.m_targetThrust);
	//LogManager::getInstance().serialPrint(m_radio.m_targetRoll, m_radio.m_targetPitch, m_radio.m_targetYaw, m_radio.m_targetThrust);

	/*if (g_startPrint)
	{
		setMotorPower(Motor::eMotor1, 0.0f);
		setMotorPower(Motor::eMotor2, 0.0f);
		setMotorPower(Motor::eMotor3, 0.0f);
		setMotorPower(Motor::eMotor4, 0.0f);

		HAL_Delay(500);

		for (int i = 0; i < 5000; ++i)
		{
			LogManager::getInstance().serialPrint(m_imu.m_gyroDebug[i].m_x, m_imu.m_gyroDebug[i].m_y, m_imu.m_gyroDebug[i].m_z, false);
			HAL_Delay(20);
		}

		g_startPrint = false;
	}*/

	//HAL_Delay(50);
}


/*
 * Set PWM's high time to control ESCs.
 * power = [0.0, 1000.0]
 */
void FlightCore::setMotorPower(const Motor& motor, const float& power)
{
	constexpr float pwmRes = 500.0;
	constexpr int pwmResMin = static_cast<int>(pwmRes);
	constexpr int pwmResMax = 2*pwmResMin;

	// 500 -> 1ms
	// 1000 -> 2ms

	int high = static_cast<int>(pwmRes + power * 0.5f);
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
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, high);
		break;

	case Motor::eMotor2:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, high);
		break;

	case Motor::eMotor3:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, high);
		break;

	case Motor::eMotor4:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, high);
		break;

	default:
		break;
	}
}

/*
 * Read the ADC and convert it to 3S battery voltage.
 * Return a value in Volt
 */
float FlightCore::readBatteryVoltage()
{
	constexpr float divider = 1.0f / 0.14826f; // Voltage divider bridge value
	constexpr float vref = 3.3f;
	constexpr float scale = (vref * divider) / 65535.0f;

	HAL_ADC_Start(&hadc3);

	if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
	{
		uint32_t raw = HAL_ADC_GetValue(&hadc3); // 0–65535 as it is a 16-bits resolution
		HAL_ADC_Stop(&hadc3);

		return raw * scale;
	}

	HAL_ADC_Stop(&hadc3);

	return 12.6; // No effect (assume full battery) if an error occur
}

/*
 * Calibrate the IMU orientation.
 * Because the IMU is never solder and mounted perfectly flat on the drone.
 * Just print out the result over UART.
 */
void FlightCore::calibrateHoverOffset()
{
	static constexpr int nbPassMinInitAhrs = 30000;
	static constexpr int nbIterMax = 500;
	static bool computeDone = false;
	static bool print = true;
	static float sumRoll = 0.0f;
	static float sumPitch = 0.0f;
	static int nbIter = 0;
	static int nbPass = 0;
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;

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
		float avgRoll = sumRoll / static_cast<float>(nbIter);
		float avgPicth = sumPitch / static_cast<float>(nbIter);
		LogManager::getInstance().serialPrint("Roll, Pitch (degree):\n\r");
		LogManager::getInstance().serialPrint(avgRoll, avgPicth, 0.0f, 0.0f);
		Quaternion<float> qAverage;
		qAverage = Quaternion<float>::fromEuler(
				avgRoll * DEGREE_TO_RAD,
				avgPicth * DEGREE_TO_RAD,
				0.0f
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
 * Must be called at 50 hz
 */
void FlightCore::pidDebugStream()
{
	static size_t callCount = 0;
	constexpr bool rate = true;

	auto formatString = [](const float& val) -> std::string
	{
		char buffer[16];

		snprintf(buffer, sizeof(buffer), "%.1f", val);
		std::string str(buffer);
		std::replace(str.begin(), str.end(), '.', ',');

		return str;
	};

	if (callCount == 0)
	{
		if (rate)
		{
			LogManager::getInstance().serialPrint("GyroPitch;TargetPitch;P;I;D;VC\r\n");
		}
		else
		{
			LogManager::getInstance().serialPrint("Pitch;TargetPitch;P;D;VC\r\n");
		}
	}

	std::string tmp = "";
	std::string vc = formatString(m_motorMixer.m_voltageCompensation * 100.0f);

	if (rate)
	{
		std::string gyro = formatString(m_imu.m_gyroFilterRates.m_x);
		std::string targetRate = formatString(m_ctrlStrat.m_rateLoop[0].m_target);//m_radio.m_targetRateRoll);
		std::string pTerm = formatString(m_ctrlStrat.m_rateLoop[0].m_pTerm);
		std::string iTerm = formatString(m_ctrlStrat.m_rateLoop[0].m_iTerm);
		std::string dTerm = formatString(m_ctrlStrat.m_rateLoop[0].m_dTerm);

		tmp = gyro + ";" + targetRate + ";" + pTerm + ";" + iTerm + ";" + dTerm + ";" + vc + "\r\n";
	}
	else
	{
		float roll;
		float pitch;
		float yaw;

		m_madgwickFilter.getEulerAngle(roll, pitch, yaw);
		std::string angle = formatString(pitch);
		std::string targetAngle = formatString(m_radio.m_targetPitch);
		std::string pTerm = formatString(m_ctrlStrat.m_angleLoop[1].m_pTerm);
		std::string dTerm = formatString(m_ctrlStrat.m_angleLoop[1].m_dTerm);

		tmp = angle + ";" + targetAngle + ";" + pTerm + ";" + dTerm + ";" + vc + "\r\n";
	}

	LogManager::getInstance().serialPrint((char*)tmp.c_str());

	callCount++;
}

