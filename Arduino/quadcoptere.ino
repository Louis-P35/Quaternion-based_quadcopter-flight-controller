#include <Arduino.h>
#include <Wire.h>

#include "readPWM.h"
#include "pid.h"
#include "kalman.h"
#include "utils.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "ms5611.h"
#include "pwm.h"
#include "dataFusion.h"
#include "madgwick.h"
#include "radio.h"


#define DEGREE_TO_RAD (PI / 180.0)
#define RAD_TO_DEGREE (180.0 / PI)

#define USE_KALMAN_FILTER
//#define CALIBRATE_IMU_HOVER_OFFSET

/* Pins mapping for PWM input (from the radio receiver) */
#define READ_PWM_CHANNEL_0_PIN 2
#define READ_PWM_CHANNEL_1_PIN 3
#define READ_PWM_CHANNEL_2_PIN 18
#define READ_PWM_CHANNEL_3_PIN 19

/* PID coefficients */
#define ANGLE_KP 40.0
#define ANGLE_KI 0.4
#define ANGLE_KD 12.0

//#define VELOCITY_KP 0.6
//0.6
//#define VELOCITY_KI 0.0
//3.5
//#define VELOCITY_KD 0.03
//0.03

#define SATURATION 10000.0

#define TARGET_ANGLE_MAX 45.0

// Motors power
#define PWM_MOTOR_FULL_STOP 16000
#define PWM_MOTOR_MIN_POWER 17500
#define PWM_MOTOR_MAX_POWER 31500
#define POWER_SCALE ((PWM_MOTOR_MAX_POWER - PWM_MOTOR_MIN_POWER) / 100.0)

// Radio control
#define THROTTLE_MID 0.1 // Around hover point
#define THROTTLE_EXPO 0.75

enum class DroneState
{
  SAFE_MODE,
  READY_TO_TAKE_OFF,
  FLYING
};

// TODO: Dual loop PID control
// TODO: Interrupt when MPU6050 has new data available
// TODO: Battery level check and scale throttle accordingly
// TODO: Scale accAngle values to match -90 +90 (calibration)
// TODO: MPU9250 SPI (+ rapide)
// TODO: Read magnetometer sensor
// TODO: Accelerometer odometry

// TODO: pid calibration (grater I term)

// Done:
// DONE: Kalman filters
// DONE: Calibration at startup
// DONE: -180, +180 range from acc and IMU
// DONE: 500Hz high res PWM signals
// DONE: Disable PID I term until take off
// DONE: Read barometer sensor
// DONE: Madgwick filters
// DONE: Full quaternion
// DONE: Throttle curve (throttle MID & throttle EXPO)

// Why use quaternion:
//    - No gimbal lock
//    - Interpolation between two quaternion is spherical, always take the shortest path
//    - No need to worry about rotation order
//    - Faster to compute than rotation matrix


// mpu6050 IMU (accelerometer + gyroscope)
MPU6050 g_imu = MPU6050();

// ms5611 barometer
MS5611 g_barometer = MS5611();

// Madgwick filter
MadgwickFilter g_madgwickFilter = MadgwickFilter();

// PIDs to perform the motors control feedback loop
PID attitudeControl_roll = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_pitch = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_yaw = PID(ANGLE_KP, 0.0, ANGLE_KD, SATURATION);

// Radio control logic
Radio g_radio = Radio(THROTTLE_MID, THROTTLE_EXPO, TARGET_ANGLE_MAX);

// Current state of the drone
DroneState g_state = DroneState::SAFE_MODE;

// Hover attitude calibration
Quaternion g_zeroRollPitch = Quaternion(0.99955, 0.005, -0.0065, 0.0);
Quaternion g_calibratedAttitude = Quaternion(1.0, 0.0, 0.0, 0.0);


void calibrateIMU();
void readRadioReceiver();
void compute_ahrs(const double& dt);
void complementaryFilter(
  const double& accAngleX, 
  const double& accAngleY, 
  const double& gyroX, 
  const double& gyroY, 
  const double& gyroZ, 
  const double& dt
  );
void handleFlyingState();
void motorsControl(const double& dt);
Quaternion correctImuMissAlignment(const Quaternion& zeroOffset, const Quaternion& imuRotation);



void setup()
{
  // Setup serial communication for debugging
  Serial.begin(19200);

  // I2C 400 KHz
  TWBR = 12;

  // Init the mpu6050 IMU
  g_imu.init();

  // Calibrate the accelerometer and gyroscope offset
  g_imu.calibrate();

  // Setup of the 3 axis magnetometer
  //magnetometerSetup();

  // Setup the ms5611 barometer
  if (g_barometer.begin() == true)
  {
    Serial.println("MS5611 found.");
  }
  else
  {
    Serial.println("MS5611 not found.");
  }

  // Configure interrupt for radio receiver reading
  setup_PWM_reader(
    READ_PWM_CHANNEL_0_PIN, 
    READ_PWM_CHANNEL_1_PIN, 
    READ_PWM_CHANNEL_2_PIN, 
    READ_PWM_CHANNEL_3_PIN
    );

  // Setup timer 1 and 5 for PWM mode
  setupPWM();

  // Init motors at full stop for 2 secs
  // for ESCs to exit safe mode.
  setPwmPin11(PWM_MOTOR_FULL_STOP);
  setPwmPin12(PWM_MOTOR_FULL_STOP);
  setPwmPin44(PWM_MOTOR_FULL_STOP);
  setPwmPin45(PWM_MOTOR_FULL_STOP);
  delay(2000);

  //madgwickUnitTest();

#ifdef CALIBRATE_IMU_HOVER_OFFSET
  // Calibrate IMU hover offset
  Serial.println("IMU hover offset calibration...");
#endif
}


void loop()
{
  static unsigned long previousTime = micros();
  
  // Get the ellapsed time since the last loop cycle
  unsigned long currentTime = micros();
  double elapsedTime = (currentTime - previousTime) / 1000000.0; // Divide by 1000000 to get seconds
  previousTime = currentTime;
  if (elapsedTime == 0.0)
  {
    elapsedTime = 1.0; // Avoid dividing by zero
  }

  // Print dt in milliseconds
  /*static bool bb = true;
  if (bb)
  {
    Serial.println(elapsedTime * 1000.0);
  }
  bb = !bb;*/

  // Read the radio receiver at a 50 hz maximum frequency
  g_radio.readRadioReceiver(g_state == DroneState::FLYING);
  printRadio(g_radio);


  // Handle the state of the drone
  // Needed for safety
  handleFlyingState();
  auto printState = []() -> void
  {
    Serial.print("Drone state: ");
    if (g_state == DroneState::SAFE_MODE)
      Serial.println("SAFE_MODE");
    else if (g_state == DroneState::READY_TO_TAKE_OFF)
      Serial.println("READY_TO_TAKE_OFF");
    else if (g_state == DroneState::FLYING)
      Serial.println("FLYING");
  };
  //printState();



  // Fetch the data from IMU (accel and gyro) and filter it
  // Also compute attitude with accelerometer only
  g_imu.get_mpu6050_data();

  // Fetch the data from the magnetometer
  // and compute the angle (yaw) in degree
  //g_magnetometerAngle = fetch_magnetometer_data();

  // Fuse the data (accelerometer and gyroscope) to compute attitude
  compute_ahrs(elapsedTime);
  //printAttitude(g_calibratedAttitude);
  //g_madgwickFilter.m_qEst.print();

#ifdef CALIBRATE_IMU_HOVER_OFFSET
  // Calibrate IMU hover offset
  calibrateImuHoverOffset();
#endif

  // Run the PID's and update PWM signal to the ESCss
  motorsControl(elapsedTime);
}



void motorsControl(const double& dt)
{
  // Get the target quaternion from the target Euler angles
  Quaternion qTarget = Quaternion::fromEuler(
    g_radio.m_targetRoll * DEGREE_TO_RAD, 
    g_radio.m_targetPitch * DEGREE_TO_RAD, 
    g_radio.m_targetYaw * DEGREE_TO_RAD
    );

  // Compute angular errors in the Quaternion space
  // This give the rotation error in the bodyframe
  Quaternion qError = PID::getError(g_calibratedAttitude, qTarget);

  // Use directly the vector part of the error quaternion as inputs of the PIDs
  double error_roll = qError.m_x;
  double error_pitch = qError.m_y;
  double error_yaw = qError.m_z;

  //printError(qError);

  // Compute PIDs
  double pidRoll = attitudeControl_roll.computePID(error_roll, dt, g_state == DroneState::FLYING);
  double pidPitch = attitudeControl_pitch.computePID(error_pitch, dt, g_state == DroneState::FLYING);
  double pidYaw = attitudeControl_yaw.computePID(error_yaw, dt, g_state == DroneState::FLYING);

  // Motors map:
  //
  //            FRONT
  // Moror 3              Motror 4
  //              Roll
  //               ^
  //               |
  //                --> Pitch
  //
  // Moror 1              Motror 2
  //            BACK

  // Apply PID adjustments to each motor
  double motor1Power = g_radio.m_targetThrust - pidPitch - pidRoll - pidYaw;
  double motor2Power = g_radio.m_targetThrust - pidPitch + pidRoll + pidYaw;
  double motor3Power = g_radio.m_targetThrust + pidPitch - pidRoll + pidYaw;
  double motor4Power = g_radio.m_targetThrust + pidPitch + pidRoll - pidYaw;

  // Scale values to match PWM's resolution
  motor1Power *= POWER_SCALE;
  motor2Power *= POWER_SCALE;
  motor3Power *= POWER_SCALE;
  motor4Power *= POWER_SCALE;

  // Set PWMs to control ESCs
  // The ESCs are fed with a 500hz PWM signal
  // They will update their duty cycle only once they have completed their current cycle
  if (g_state == DroneState::FLYING || g_state == DroneState::READY_TO_TAKE_OFF)
  {
    int pwm1 = constrain(PWM_MOTOR_FULL_STOP + (int)motor1Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER);
    int pwm2 = constrain(PWM_MOTOR_FULL_STOP + (int)motor2Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER);
    int pwm3 = constrain(PWM_MOTOR_FULL_STOP + (int)motor3Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER);
    int pwm4 = constrain(PWM_MOTOR_FULL_STOP + (int)motor4Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER);

    setPwmPin11(pwm1);
    setPwmPin12(pwm2);
    setPwmPin44(pwm3);
    setPwmPin45(pwm4);

    //printMotorsPower(pwm1, pwm2, pwm3, pwm4, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER);
  }
  else // DroneState::SAFE_MODE
  {
    setPwmPin11(PWM_MOTOR_FULL_STOP);
    setPwmPin12(PWM_MOTOR_FULL_STOP);
    setPwmPin44(PWM_MOTOR_FULL_STOP);
    setPwmPin45(PWM_MOTOR_FULL_STOP);
  }
}


void handleFlyingState()
{
  switch(g_state)
  {
    case DroneState::SAFE_MODE:
    {
      // Wait for sticks to be down and toward inside
      if (g_radio.m_radioChannel3 < 1150 && 
          g_radio.m_radioChannel2 > 1800 &&
          g_radio.m_radioChannel1 < 1150 &&
          g_radio.m_radioChannel4 > 1800)
      {
        g_state = DroneState::READY_TO_TAKE_OFF;
      }
      break;
    }
    case DroneState::READY_TO_TAKE_OFF:
    {
      if (g_radio.m_radioChannel3 > 1150)
      {
        g_state = DroneState::FLYING;

        // Resetting target yaw because it has been affected
        // by the stick when exiting safe mode.
        g_radio.m_targetYaw = 0.0;
      }
      break;
    }
    case DroneState::FLYING:
    {
      break;
    }
    default:
    {
      break;
    }
  }
}



// Legacy code ...
void complementaryFilter(
  const double& accAngleX, 
  const double& accAngleY, 
  const double& gyroX, 
  const double& gyroY, 
  const double& gyroZ, 
  const double& dt
  )
{
  static const double COMPLEMENTARY_FILTER_GAIN = 0.98;
  static double roll = 0.0;
  static double pitch = 0.0;
  static double yaw = 0.0;

  roll = COMPLEMENTARY_FILTER_GAIN * (roll + (gyroX * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleX;
  pitch = COMPLEMENTARY_FILTER_GAIN * (pitch + (gyroY * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleY;
  yaw += gyroZ * dt;
  if (yaw > 180.0) yaw -= 360.0;
  else if (yaw < -180.0) yaw += 360.0;
}


void compute_ahrs(const double& dt)
{
/*#ifdef USE_KALMAN_FILTER
  // Fuse the data (accelerometer and gyroscope) with Kalman filter to compute attitude
  g_roll = g_kalman_roll.compute(g_imu.m_angleAccX, g_imu.m_filteredGyroX, dt);
  g_pitch = g_kalman_pitch.compute(g_imu.m_angleAccY, g_imu.m_filteredGyroY, dt);
  g_yaw += g_imu.m_filteredGyroZ * dt; //g_kalman_yaw.compute(g_imu.m_angleAccY, g_imu.m_filteredGyroY, dt);
  if (g_yaw > 180.0) g_yaw -= 360.0;
  else if (g_yaw < -180.0) g_yaw += 360.0;
#else
  // Complementary filter
  complementaryFilter(g_imu.m_angleAccX, g_imu.m_angleAccY, g_imu.m_filteredGyroX, g_imu.m_filteredGyroY, g_imu.m_filteredGyroZ, dt);
#endif*/
  // Madgwick filter
  g_madgwickFilter.compute(
    g_imu.m_filteredAcceloremeterX, // Acceleration vector will be normalized
    g_imu.m_filteredAcceloremeterY, 
    g_imu.m_filteredAcceloremeterZ, 
    g_imu.m_filteredGyroX * DEGREE_TO_RAD, 
    g_imu.m_filteredGyroY * DEGREE_TO_RAD, 
    g_imu.m_filteredGyroZ * DEGREE_TO_RAD, 
    dt
  );
  //g_madgwickFilter.getEulerAngle(g_roll, g_pitch, g_yaw);
  //g_madgwickFilter.m_qEst.print();

  // Correct the miss alignment of the IMU
  g_calibratedAttitude = correctImuMissAlignment(g_zeroRollPitch, g_madgwickFilter.m_qEst);
}

/*
This function apply a correction to the measured quaternion attitude
to correct for the IMU missalignment.
*/
Quaternion correctImuMissAlignment(const Quaternion& zeroOffset, const Quaternion& imuRotation)
{
  return zeroOffset.inverse() * imuRotation;
}



/* Calibrate the IMU orientation
Because the IMU is never solder and mounted perfectly flat on the drone
This function must not be build on release */
void calibrateImuHoverOffset()
{
  // Calibrate IMU hover offset
  static Quaternion qSum = Quaternion(1.0, 0.0, 0.0, 0.0);
  static unsigned int nbPass = 0;
  const int nbLoop = 1000;
  
  if (nbPass < nbLoop)
  {
    qSum += g_madgwickFilter.m_qEst;
    nbPass++;
  }
  else
  {
    Quaternion qTmp = qSum * (1.0/(double)nbLoop);
    Serial.print("IMU world orientation: ");
    qTmp.print();
  }
}



