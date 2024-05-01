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


#define DEGREE_TO_RAD (PI / 180.0)
#define RAD_TO_DEGREE (180.0 / PI)

#define USE_KALMAN_FILTER

/* Pins mapping for PWM input (from the radio receiver) */
#define READ_PWM_CHANNEL_0_PIN 2
#define READ_PWM_CHANNEL_1_PIN 3
#define READ_PWM_CHANNEL_2_PIN 18
#define READ_PWM_CHANNEL_3_PIN 19

/* PID coefficients */
#define ANGLE_KP 10.0
#define ANGLE_KI 0.0
#define ANGLE_KD 0.0

//#define VELOCITY_KP 0.6
//0.6
//#define VELOCITY_KI 0.0
//3.5
//#define VELOCITY_KD 0.03
//0.03

#define SATURATION 100.0

#define TARGET_ANGLE_MAX 45.0

// Motors power
#define PWM_MOTOR_FULL_STOP 16000
#define PWM_MOTOR_MIN_POWER 17500
#define PWM_MOTOR_MAX_POWER 31500

enum class DroneState
{
  SAFE_MODE,
  READY_TO_TAKE_OFF,
  FLYING
};

// TODO: Implement a panic recovery mode, by detecting the free fall with accelerometer
// TODO: Dual loop PID control
// TODO: DMP
// TODO: Interrupt when MPU6050 has new data available
// TODO: Battery level check
// TODO: Scale accAngle values to match -90 +90 (calibration)
// TODO: MPU9250 SPI (+ rapide)
// TODO: Read magnetometer sensor
// TODO: Accelerometer odometry

// Done:
// DONE: Kalman filters
// DONE: Calibration at startup
// DONE: -180, +180 range from acc and IMU
// DONE: 500Hz high res PWM signals
// DONE: Disable PID I term until take off
// DONE: Read barometer sensor
// DONE: Madgwick filters
// DONE: Full quaternion


// mpu6050 IMU (accelerometer + gyroscope)
MPU6050 g_imu = MPU6050();

// ms5611 barometer
MS5611 g_barometer = MS5611();

// Kalman filters, to perform the data fusion between the gyroscope and accelerometer
//Kalman1D g_kalman_roll = Kalman1D(0.001, 0.003, 0.03);
//Kalman1D g_kalman_pitch = Kalman1D(0.001, 0.003, 0.03);

// Two dimentional Kalman filter to perform data fusion between the barometer and accelerometer
//Kalman2D g_kalman2d_altitude = Kalman2D();

// Madgwick filter
MadgwickFilter g_madgwickFilter = MadgwickFilter();

// PIDs to perform the motors control feedback loop
PID attitudeControl_roll = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_pitch = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_yaw = PID(ANGLE_KP, 0.0, ANGLE_KD, SATURATION);


unsigned long g_radioChannel1 = 0;
unsigned long g_radioChannel2 = 0;
unsigned long g_radioChannel3 = 0;
unsigned long g_radioChannel4 = 0;

double g_magnetometerAngle = 0.0;

double g_targetRoll = 0.0;
double g_targetPitch = 0.0;
double g_targetYaw = 0.0;
double g_targetThrust = 0.0;

/* Final attitude */
double g_roll = 0.0;
double g_pitch = 0.0;
double g_yaw = 0.0;

// Current state of the drone
DroneState g_state = DroneState::SAFE_MODE;


void calibrateIMU();
void readRadioReceiver();
void imuDataFusion(const double& dt);
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



void setup()
{
  // Setup serial communication for debugging
  Serial.begin(19200);

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
  setup_PWM_reader(READ_PWM_CHANNEL_0_PIN, READ_PWM_CHANNEL_1_PIN, READ_PWM_CHANNEL_2_PIN, READ_PWM_CHANNEL_3_PIN);

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



  // Compute altitude using barometer and accelerometer
  // double barometerAltitude = readAltitude(&g_barometer);

  /*double worldAccZ = computeAltitude(
    g_imu.m_filteredAcceloremeterX, 
    g_imu.m_filteredAcceloremeterY, 
    g_imu.m_filteredAcceloremeterZ, 
    g_roll * DEGREE_TO_RAD, 
    g_pitch * DEGREE_TO_RAD, 
    barometerAltitude, 
    elapsedTime
    );
  double altitude = g_kalman2d_altitude.compute(barometerAltitude, worldAccZ, elapsedTime);*/
  //Serial.print("Alt: ");
  //Serial.println(barometerAltitude);
  //Serial.print(" AltAcc: ");
  /*Serial.print(-8.0);
  Serial.print(",");
  Serial.print(-12.0);
  Serial.print(",");
  Serial.print(barometerAltitude);
  Serial.print(",");
  Serial.println(worldAccZ);*/

  // Read the radio receiver at a 50 hz maximum frequency
  readRadioReceiver();
  auto printRadio = []() -> void
  {
    Serial.print("Roll: ");
    Serial.print(g_targetRoll);
    Serial.print("\t Pitch: ");
    Serial.print(g_targetPitch);
    Serial.print("\t Yaw: ");
    Serial.print(g_targetYaw);
    Serial.print("\t Thrust: ");
    Serial.println(g_targetThrust);
  };
  //printRadio();


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
  imuDataFusion(elapsedTime);

  auto printAttitude = []() -> void
  {
    double roll, pitch, yaw = 0.0;
    g_madgwickFilter.m_qEst.toEuler(roll, pitch, yaw);
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("\t Pitch:");
    Serial.print(pitch);
    Serial.print("\t Yaw:");
    Serial.println(yaw);
  };
  //printAttitude();

  // Run the PID's and update PWM signal to the ESCss
  motorsControl(elapsedTime);

}



void motorsControl(const double& dt)
{
  // Get the target quaternion from the target Euler angles
  Quaternion qTarget = Quaternion::fromEuler(
    g_targetRoll * DEGREE_TO_RAD, 
    g_targetPitch * DEGREE_TO_RAD, 
    g_targetYaw * DEGREE_TO_RAD
    );

  // Compute angular errors in the Quaternion space
  // This give the rotation error in the bodyframe
  Quaternion qError = PID::getError(g_madgwickFilter.m_qEst, qTarget);

  // Use directly the vector part of the error quaternion as inputs of the PIDs
  double error_roll = qError.m_x;
  double error_pitch = qError.m_y;
  double error_yaw = qError.m_z;

  // Print for debug
  auto printError = [](const Quaternion& qErr) -> void
  {
    double errRoll = 0.0;
    double errPitch = 0.0;
    double errYaw = 0.0;

    // Switching to Euler representation can lead to gimbal lock
    qErr.toEuler(errRoll, errPitch, errYaw);

    Serial.print("Error roll: ");
    Serial.print(errRoll);
    Serial.print("\t Error pitch:");
    Serial.print(errPitch);
    Serial.print("\t Error yaw:");
    Serial.println(errYaw);
  };
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
  double motor1Power = g_targetThrust - pidPitch - pidRoll + pidYaw;
  double motor2Power = g_targetThrust - pidPitch + pidRoll - pidYaw;
  double motor3Power = g_targetThrust + pidPitch - pidRoll - pidYaw;
  double motor4Power = g_targetThrust + pidPitch + pidRoll + pidYaw;

  // Scale values to match PWM's resolution
  const double scale = (PWM_MOTOR_MAX_POWER - PWM_MOTOR_MIN_POWER) / 100.0;
  motor1Power *= scale;
  motor2Power *= scale;
  motor3Power *= scale;
  motor4Power *= scale;

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

    auto printMotorsPower = [pwm1, pwm2, pwm3, pwm4]() -> void
    {
      Serial.print(pwm1);
      Serial.print("\t");
      Serial.print(pwm2);
      Serial.print("\t");
      Serial.print(pwm3);
      Serial.print("\t");
      Serial.print(pwm4);
      Serial.print("\t");
      Serial.print(PWM_MOTOR_MIN_POWER);
      Serial.print("\t");
      Serial.println(PWM_MOTOR_MAX_POWER);
    };
    printMotorsPower();
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
      if (g_radioChannel3 < 1150 && 
          g_radioChannel2 > 1800 &&
          g_radioChannel1 < 1150 &&
          g_radioChannel4 > 1800)
      {
        g_state = DroneState::READY_TO_TAKE_OFF;
      }
      break;
    }
    case DroneState::READY_TO_TAKE_OFF:
    {
      if (g_radioChannel3 > 1150)
      {
        g_state = DroneState::FLYING;

        // Resetting target yaw because it has been affected
        // by the stick when exiting safe mode.
        g_targetYaw = 0.0;
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

  g_roll = COMPLEMENTARY_FILTER_GAIN * (g_roll + (gyroX * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleX;
  g_pitch = COMPLEMENTARY_FILTER_GAIN * (g_pitch + (gyroY * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleY;
  g_yaw += gyroZ * dt;
  if (g_yaw > 180.0) g_yaw -= 360.0;
  else if (g_yaw < -180.0) g_yaw += 360.0;
}


void imuDataFusion(const double& dt)
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
}


void readRadioReceiver()
{
  static unsigned long previousTime = millis();
  
  // Get the ellapsed time since the last time the receiver has been read
  // and read the data only every 20 ms
  unsigned long currentTime = millis();
  double dt = (currentTime - previousTime) / 1000.0;
  if ((currentTime - previousTime) < 20)
  {
    return;
  }
  previousTime = currentTime;

  /* Lambda to convert microseconds to angle */
  auto convertToAngle = [](const unsigned long& duration, const double& amplitudeMax, const bool& invertAxe) -> double
  {
    const double deadZone = 1.0;

    double tmp = (((((double)duration) - 1000.0) / 1000.0) * (amplitudeMax * 2.0)) - amplitudeMax;

    // Hysteresis to have a stable zero
    if (tmp > -deadZone && tmp < deadZone)
    {
      tmp = 0.0;
    }
    else if (tmp > 0.0)
    {
      tmp -= deadZone;
    }
    else
    {
      tmp += deadZone;
    }

    double retVal = (tmp < -amplitudeMax) ? (-amplitudeMax) : (tmp > amplitudeMax) ? amplitudeMax : tmp;

    if (invertAxe)
      return -retVal;

    return retVal;
  };

  /* Lambda to integrate yaw command */
  auto convertYawToAngle = [](const unsigned long& duration, const double& dt) -> double
  {
    static double yawAngle = 0.0;

    const double deadZone = 0.05;
    const double speed = 200.0;

    double tmp = ((((double)duration) - 1000.0) / 1000.0) - 0.5;

    // Hysteresis to have a stable zero
    if (tmp > -deadZone && tmp < deadZone)
    {
      tmp = 0.0;
    }
    else if (tmp > 0.0)
    {
      tmp -= deadZone;
    }
    else
    {
      tmp += deadZone;
    }

    // Integrate target yaw
    yawAngle += tmp * speed * dt;

    // Clamp value to [-180;+180]
    if (yawAngle > 180.0)
      yawAngle -= 360.0;
    else if (yawAngle < -180.0)
      yawAngle += 360.0;

    return yawAngle;
  };

  /* Read the radio receiver */
  g_radioChannel1 = getRadioChannel(0); // Roll
  g_radioChannel2 = getRadioChannel(1); // Pitch
  g_radioChannel3 = getRadioChannel(2); // Thrust
  g_radioChannel4 = getRadioChannel(3); // Yaw
  auto printRadioChannels = []() -> void
  {
    Serial.print(g_radioChannel1);
    Serial.print("\t");
    Serial.print(g_radioChannel2);
    Serial.print("\t");
    Serial.print(g_radioChannel3);
    Serial.print("\t");
    Serial.println(g_radioChannel4);
  };
  //printRadioChannels();

  g_targetRoll = convertToAngle(g_radioChannel1, TARGET_ANGLE_MAX, true);
  g_targetPitch = convertToAngle(g_radioChannel2, TARGET_ANGLE_MAX, false);
  g_targetYaw = convertYawToAngle(g_radioChannel4, dt);
  g_targetThrust = ((double)g_radioChannel3 - 1000.0) / 10.0;
}


