/*
 * scheduler.cpp
 *
 *  Created on: Jun 11, 2024
 *      Author: louis
 */

// Includes from STL
#include <orchestrator.h>
#include <string.h>  // Include for memcpy

// Includes from Project
#include "scheduler.hpp"
#include "logManager.hpp"
#include "PID/controlStrategy.hpp"
#include "PID/pid.hpp"
#include "PWM/readRadio.hpp"
#include "stateMachine.hpp"

// screen /dev/tty.usbserial-14220 115200

#define DEGREE_TO_RAD (M_PI/180.0)


/*
 * Filters
 */

//#define GYRO_NOTCH_F0 87.5f
//#define GYRO_NOTCH_Q 1.0f
//#define ACCEL_NOTCH_F0 200.0f
//#define ACCEL_NOTCH_Q 15.0f



/*
 * PIDs coeff
 */

#define SATURATION 75.0f
#define MAX_OUT 1000.0f
#define MIN_OUT -1000.0f

#define ROLL_PITCH_RATE_MAX_D_PERCENT 0.75f // < 1 for stability, [1, 2] for aggresivity

#define ROLLPITCH_ANGLE_KP 0.0f
#define ROLLPITCH_ANGLE_KI 0.0f
#define ROLLPITCH_ANGLE_KD 0.0f

#define YAW_ANGLE_KP 0.0f
#define YAW_ANGLE_KI 0.0f
#define YAW_ANGLE_KD 0.0f

#define ROLLPITCH_RATE_KP 0.5f
#define ROLLPITCH_RATE_KI 0.3f
#define ROLLPITCH_RATE_KD 0.06f

#define YAW_RATE_KP 0.0f
#define YAW_RATE_KI 0.0f
#define YAW_RATE_KD 0.0f

#define ROLLPITCH_POS_KP 0.0f
#define ROLLPITCH_POS_KI 0.0f
#define ROLLPITCH_POS_KD 0.0f

#define YAW_POS_KP 0.0f
#define YAW_POS_KI 0.0f
#define YAW_POS_KD 0.0f

// Radio control
#define THROTTLE_HOVER_OFFSET 0.1f // Around hover point
#define THROTTLE_EXPO 0.99f
#define TARGET_ANGLE_MAX 45.0f
#define TARGET_RATE_MAX 200.0f


extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;
extern Scheduler g_scheduler;
volatile bool g_enableRadioLoop = false;
volatile bool g_start = false;

constexpr float Scheduler::m_rateDt;
constexpr float Scheduler::m_ahrsDt;
constexpr float Scheduler::m_angleDt;
constexpr float Scheduler::m_posDt;
constexpr float Scheduler::m_radioDt;

//#define CALIBRATE_IMU 1

// 0.45 (x4) = take off thrust

//#define COMPUTE_HOVER_OFFSET 1

// Uncomment this to disable motors
#define DEBUG_DISABLE_MOTORS 1
#define PID_TESTING_MODE 1


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


Scheduler::Scheduler(
		uint16_t spi_cs_pin,
		GPIO_TypeDef* spi_cs_gpio_port
		) :
		m_radio(THROTTLE_HOVER_OFFSET, THROTTLE_EXPO, TARGET_ANGLE_MAX, TARGET_RATE_MAX)
{

}


/*
 * Called once at the beginning of the software
 */
void Scheduler::mainSetup()
{
	// Setup the serial print
	LogManager::getInstance().setup();

	// Setup the IMU (ICM20948)
	m_imu.init(IMU_SAMPLE_FREQUENCY/*, GYRO_NOTCH_F0, GYRO_NOTCH_Q, ACCEL_NOTCH_F0, ACCEL_NOTCH_Q*/);

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
	setupRadio();

	// Set Startup state
	StateMachine::getInstance().setState(StateMachine::getInstance().getStartupSequenceState());

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


	// Gyro filters for PID rate loop
	const float rateLoopFreq = static_cast<float>(IMU_SAMPLE_FREQUENCY) / static_cast<float>(RATE_DIVIDER);
	for (size_t i = 0; i < 3; ++i)
	{
		m_ctrlStrat.m_rateLoop[i].m_dTermLpf.init(rateLoopFreq, 20.0f);
		m_ctrlStrat.m_rateLoop[i].m_ffTermLpf.init(rateLoopFreq, 50.0f);
	}

	// Start the loop
	g_start = true;
}


uint32_t ahrsTicks = 0;
uint32_t pidRateTicks = 0;
uint32_t escTicks = 0;
uint32_t radioTicks = 0;


/*
 * Called at 4khz by timer 2 overflow interrupt
 * This is the main loop of this software
 */
void orchestrator_highestFrequencyLoop()
{
	static uint32_t ticks = 0;

	if (g_start)
	{
		ticks++;

		// 4 khz loop
		g_scheduler.m_imu.readAndFilterIMU_gdps();

		// 1 khz loop
		// AHRS
		if ((ticks % AHRS_DIVIDER) == 0) // 1khz (4khz / 4)
		{
			g_scheduler.ahrsLoop();
			ahrsTicks++;
		}

		// 500 hz loop
		// ESCs
		if ((ticks % ESC_DIVIDER) == 0) // 500hz (6khz / 12)
		{
			g_scheduler.escLoop();
			escTicks++;
		}

		// 100 hz loop
		// PID position hold
		if ((ticks % POS_HOLD_DIVIDER) == 0) // 100hz (6khz / 60)
		{
			// Enable PID position hold loop (that will run in the state machine)
			// Enable it only in certain flight mode
			if (g_scheduler.m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
			{
				g_scheduler.m_posLoop = true;
			}
		}

		// 50 hz loop
		// Radio
		if ((ticks % RADIO_DIVIDER) == 0) // 50hz (6khz / 120)
		{
			g_scheduler.radioLoop();
			radioTicks++;
		}

		// 2 khz loop
		// PID rate loop
		// The last called because it run the state machine
		if ((ticks % RATE_DIVIDER) == 0) // 2khz (4khz / 2)
		{
			g_scheduler.pidRateLoop();
			pidRateTicks++;
		}
	}
}


/*
 * Handle drone behavior according to the current state (state machine)
 * Run all the PID loops
 */
void Scheduler::pidRateLoop()
{
	// Run the state machine
	StateMachine::getInstance().run(*this);

	// Reset angle & position hold flag here because they are executed in the state machine
	m_angleLoop = false;
	m_posLoop = false;


	/*if (startRecord && m_imu.m_gyroDebugIndex < 5000)
	{
		m_imu.m_gyroDebug[m_imu.m_gyroDebugIndex] = m_gyroCopyRaw;
		m_imu.m_gyroDebugIndex++;
	}
	else if (m_imu.m_gyroDebugIndex == 5000)
	{
		m_imu.m_gyroDebugIndex++;
		setMotorPower(Motor::eMotor1, 0.0f);
		setMotorPower(Motor::eMotor2, 0.0f);
		setMotorPower(Motor::eMotor3, 0.0f);
		setMotorPower(Motor::eMotor4, 0.0f);
		for (int i = 0; i < 5000; ++i)
		{
			LogManager::getInstance().serialPrint(m_imu.m_gyroDebug[i].m_x, m_imu.m_gyroDebug[i].m_y, false);
			HAL_Delay(20);
		}
	}*/
}


/*
 * Run Madgwick filter
 */
void Scheduler::ahrsLoop()
{
	// AHRS, Madgwick filter
	m_madgwickFilter.compute(
			m_imu.m_accel.m_x, // Acceleration vector will be normalized
			m_imu.m_accel.m_y,
			m_imu.m_accel.m_z,
			m_imu.m_gyro.m_x * DEGREE_TO_RAD,
			m_imu.m_gyro.m_y * DEGREE_TO_RAD,
			m_imu.m_gyro.m_z * DEGREE_TO_RAD,
			m_ahrsDt
		);

	// Debug print AHRS result
	//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_madgwickFilter.m_qEst);


#ifdef COMPUTE_HOVER_OFFSET
	// Compute the hover offset (must be done once after each teardown/build of the drone)
	calibrateHoverOffset();
#endif

	// Enable PID angle loop (that will run in the state machine)
	// Enable it only in certain flight mode
	if(m_ctrlStrat.m_flightMode == StabilizationMode::STAB ||
			m_ctrlStrat.m_flightMode == StabilizationMode::POSHOLD)
	{
		m_angleLoop = true;
	}
}


/*
 * Motors update
 */
void Scheduler::escLoop()
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
}


/*
 * Read radio's PWM signals
 */
void Scheduler::radioLoop()
{
	static bool startRecord = false;
	const bool signalLost = m_radio.readRadioReceiver(true, m_radioDt);

	//LogManager::getInstance().serialPrint(m_rateDt);

	//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_madgwickFilter.m_qEst);

	if (!signalLost)
	{
		//LogManager::getInstance().serialPrint(m_radio.m_targetRoll, m_radio.m_targetPitch, m_radio.m_targetYaw, m_radio.m_targetThrust);
		//LogManager::getInstance().serialPrint(m_radio.m_targetRateRoll, m_radio.m_targetRatePitch, m_radio.m_targetRateYaw, m_radio.m_targetThrust);

		// Compute target quaternion
		m_targetAttitude = Quaternion<float>::fromEuler(m_radio.m_targetRoll * DEGREE_TO_RAD, m_radio.m_targetPitch * DEGREE_TO_RAD, m_radio.m_targetYaw * DEGREE_TO_RAD);
		// Debug print AHRS result
		//LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_targetAttitude);
	}
	else
	{
		// Signal lost, target quaternion is horizon
	}

#ifdef PID_TESTING_MODE
	m_radio.m_targetRateRoll = 0.0;
	m_radio.m_targetRateYaw = 0.0;
	//m_radio.m_targetRatePitch = (m_radio.m_targetRatePitch / 172.0) * 50.0;

	if (m_radio.m_targetThrust > 350.0f)
	{
		m_radio.m_targetThrust = 350.0f;
		startRecord = true;
	}

	if (m_radio.m_targetRatePitch > 50.0f)
	{
		m_radio.m_targetRatePitch = 250.0f;
	}
	else if (m_radio.m_targetRatePitch < -50.0f)
	{
		m_radio.m_targetRatePitch = -250.0f;
	}

	//pidDebugStream();
#endif

	/*static int ttt = 0;
	ttt++;
	if ((ttt % 10) == 0)
	{
		LogManager::getInstance().serialPrint(m_radio.m_radioChannel1, m_radio.m_radioChannel2, m_radio.m_radioChannel3, m_radio.m_radioChannel4);
	}*/
	/*std::string tototmp = "POWER: \r\n";
	tototmp += std::to_string(m_motorMixer.m_powerMotor[1]);
	tototmp += " ";
	tototmp += std::to_string(m_motorMixer.m_powerMotor[2]);
	tototmp += "\r\n";
	tototmp += std::to_string(m_motorMixer.m_powerMotor[0]);
	tototmp += " ";
	tototmp += std::to_string(m_motorMixer.m_powerMotor[3]);
	tototmp += "\r\nTORQUE: \n\r";
	tototmp += std::to_string(m_thrust);
	tototmp += " ";
	tototmp += std::to_string(m_torqueX);
	tototmp += " ";
	tototmp += std::to_string(m_torqueY);
	tototmp += " ";
	tototmp += std::to_string(m_torqueZ);
	tototmp += "\r\n";
	LogManager::getInstance().serialPrint((char*)tototmp.c_str());*/

	/*LogManager::getInstance().serialPrint("POWER: \n\r");
	LogManager::getInstance().serialPrint(m_motorMixer.m_powerMotor[1], m_motorMixer.m_powerMotor[2], 0.0, 0.0);
	LogManager::getInstance().serialPrint(m_motorMixer.m_powerMotor[0], m_motorMixer.m_powerMotor[3], 0.0, 0.0);
	LogManager::getInstance().serialPrint("TORQUE: \n\r");
	LogManager::getInstance().serialPrint(m_thrust, m_torqueX, m_torqueY, m_torqueZ);
	LogManager::getInstance().serialPrint("\n\r");*/
}


/*
 * Called indefinitely in a loop
 */
void Scheduler::mainLoop(const double dt)
{
	//LogManager::getInstance().serialPrint(pidRateTicks, ahrsTicks, escTicks, radioTicks);
	//LogManager::getInstance().serialPrint("\n\r");

	//LogManager::getInstance().serialPrint(m_imu.m_gyro.m_y);
	//LogManager::getInstance().serialPrint("\n\r");
	LogManager::getInstance().serialPrint(m_madgwickFilter.m_qEst, m_madgwickFilter.m_qEst);
	LogManager::getInstance().serialPrint("\n\r");

	HAL_Delay(50);
}


/*
 * Set PWM's high time to control ESCs.
 * power = [0.0, 1000.0]
 */
void Scheduler::setMotorPower(const Motor& motor, const float& power)
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
void Scheduler::pidDebugStream()
{
	static size_t callCount = 0;

	// Logging at 25 hz
	/*if ((callCount%2) != 0)
	{
		return;
	}*/

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
		LogManager::getInstance().serialPrint("GyroPitch;TargetPitch;P;I;D\r\n");
	}

	std::string gyro = formatString(m_imu.m_gyro.m_y);
	std::string targetRate = formatString(m_radio.m_targetRatePitch);
	std::string pTerm = formatString(m_ctrlStrat.m_rateLoop[1].m_pTerm);
	std::string iTerm = formatString(m_ctrlStrat.m_rateLoop[1].m_iTerm);
	std::string dTerm = formatString(m_ctrlStrat.m_rateLoop[1].m_dTerm);

	std::string tmp = gyro + ";" + targetRate + ";" + pTerm + ";" + iTerm + ";" + dTerm + "\r\n";
	LogManager::getInstance().serialPrint((char*)tmp.c_str());

	callCount++;
}

