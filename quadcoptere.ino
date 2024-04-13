#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#include "readPWM.h"
#include "pid.h"
#include "kalman.h"
#include "utils.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "pwm.h"


const int MS5611_I2C_ADDR = 0x77;               // MS5611 I2C address (barometer)

#define USE_KALMAN_FILTER

/* Pins mapping for PWM output (to control ESCs) */
#define PWM_PIN_1 8
#define PWM_PIN_2 9
#define PWM_PIN_3 10
#define PWM_PIN_4 11

/* Pins mapping for PWM input (from the radio receiver) */
#define READ_PWM_CHANNEL_0_PIN 2
#define READ_PWM_CHANNEL_1_PIN 3
#define READ_PWM_CHANNEL_2_PIN 18
#define READ_PWM_CHANNEL_3_PIN 19

/* PID coefficients */
#define ANGLE_KP 1.0
#define ANGLE_KI 0.0
#define ANGLE_KD 0.0

//#define VELOCITY_KP 0.6
//0.6
//#define VELOCITY_KI 0.0
//3.5
//#define VELOCITY_KD 0.03
//0.03

#define SATURATION 100.0

#define TARGET_ANGLE_MAX 20.0

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
// TODO: Scale accAngle values to match -90 +90
// TODO: MPU9250 SPI (+ rapide)
// TODO: Read barometer & magnetometer sensor

// Done:
// DONE: Kalman filters
// DONE: Calibration at startup
// DONE: -180, +180 range from acc and IMU
// DONE: 500Hz high res PWM signals
// DONE: Disable PID I term until take off


// mpu6050 IMU (accelerometer + gyroscope)
MPU6050 g_imu = MPU6050();

// Kalman filters, to perform the data fusion between the gyroscope and accelerometer
Kalman g_kalman_roll = Kalman(0.001, 0.003, 0.03);
Kalman g_kalman_pitch = Kalman(0.001, 0.003, 0.03);

// PIDs to perform the motors control feedback loop
PID attitudeControl_roll = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_pitch = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_yaw = PID(ANGLE_KP, 0.0, ANGLE_KD, SATURATION);



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

  // Read the radio receiver at a 50 hz maximum frequency
  readRadioReceiver();


  // Handle the state of the drone
  // Needed for safety
  handleFlyingState();


  // Fetch the data from IMU (accel and gyro) and filter it
  // Also compute attitude with accelerometer only
  g_imu.get_mpu6050_data();

  // Fetch the data from the magnetometer
  // and compute the angle (yaw) in degree
  //g_magnetometerAngle = fetch_magnetometer_data();

  // Fuse the data (accelerometer and gyroscope) to compute attitude
  imuDataFusion(elapsedTime);


  Serial.print("Roll:");
  Serial.print(g_roll);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.print(g_pitch);
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.println(g_yaw);

  // Run the PID's and update PWM signal to the ESCss
  motorsControl(elapsedTime);

}



void motorsControl(double dt)
{
 
  // Compute angular errors
  double error_roll = g_targetRoll - g_roll;
  double error_pitch = g_targetPitch - g_pitch;
  double error_yaw = g_targetYaw - g_yaw;

  // Compute PIDs
  double pidRoll = attitudeControl_roll.computePID(error_roll, dt, g_state == DroneState::FLYING);
  double pidPitch = attitudeControl_pitch.computePID(error_pitch, dt, g_state == DroneState::FLYING);
  double pidYaw = attitudeControl_yaw.computePID(error_yaw, dt, g_state == DroneState::FLYING);


  // Scale PIDs values to their desired range
  // TODO

  // Calculate motors power
  //            FRONT
  // Moror 1              Motror 2
  //  
  //
  //
  //
  //
  // Moror 3              Motror 4
  //            BACK

  // Motor 1: Front left
  // Motor 2: Front right
  // Motor 3: Back left
  // Motor 4: Back right
  // Apply PID adjustments to each motor
  double motor1Power = g_targetThrust - pidPitch - pidRoll + pidYaw;
  double motor2Power = g_targetThrust - pidPitch + pidRoll - pidYaw;
  double motor3Power = g_targetThrust + pidPitch - pidRoll - pidYaw;
  double motor4Power = g_targetThrust + pidPitch + pidRoll + pidYaw;

  // Set PWMs to control ESCs
  // The ESCs are fed with a 500hz PWM signal
  // They will update their duty cycle only once they have completed their current cycle
  if (g_state == DroneState::FLYING || g_state == DroneState::READY_TO_TAKE_OFF)
  {
    setPwmPin11(constrain(PWM_MOTOR_FULL_STOP + (int)motor1Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER));
    setPwmPin12(constrain(PWM_MOTOR_FULL_STOP + (int)motor2Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER));
    setPwmPin44(constrain(PWM_MOTOR_FULL_STOP + (int)motor3Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER));
    setPwmPin45(constrain(PWM_MOTOR_FULL_STOP + (int)motor4Power, PWM_MOTOR_MIN_POWER, PWM_MOTOR_MAX_POWER));
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
      // Wait for thrust and pitch stick to be down
      if (g_targetThrust < 5.0 && g_targetPitch < (-TARGET_ANGLE_MAX + 3.0)) // TODO change value
      {
        g_state = DroneState::READY_TO_TAKE_OFF;
      }
      break;
    }
    case DroneState::READY_TO_TAKE_OFF:
    {
      if (g_targetThrust > 6) // TODO change value
      {
        g_state = DroneState::READY_TO_TAKE_OFF;
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



void complementaryFilter(double accAngleX, double accAngleY, double gyroX, double gyroY, double gyroZ, double dt)
{
  static const double COMPLEMENTARY_FILTER_GAIN = 0.98;

  g_roll = COMPLEMENTARY_FILTER_GAIN * (g_roll + (gyroX * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleX;
  g_pitch = COMPLEMENTARY_FILTER_GAIN * (g_pitch + (gyroY * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleY;
  g_yaw += gyroZ * dt;
  if (g_yaw > 180.0) g_yaw -= 360.0;
  else if (g_yaw < -180.0) g_yaw += 360.0;
}


void imuDataFusion(double dt)
{
#ifdef USE_KALMAN_FILTER
  // Fuse the data (accelerometer and gyroscope) with Kalman filter to compute attitude
  g_roll = g_kalman_roll.compute(g_imu.m_angleAccX, g_imu.m_filteredGyroX, dt);
  g_pitch = g_kalman_pitch.compute(g_imu.m_angleAccY, g_imu.m_filteredGyroY, dt);
  g_yaw += g_imu.m_filteredGyroZ * dt; //g_kalman_yaw.compute(g_imu.m_angleAccY, g_imu.m_filteredGyroY, dt);
  if (g_yaw > 180.0) g_yaw -= 360.0;
  else if (g_yaw < -180.0) g_yaw += 360.0;
#else
  // Complementary filter
  complementaryFilter(g_imu.m_angleAccX, g_imu.m_angleAccY, g_imu.m_filteredGyroX, g_imu.m_filteredGyroY, g_imu.m_filteredGyroZ, dt);
#endif
}


void readRadioReceiver()
{
  static unsigned long previousTime = millis();
  
  // Get the ellapsed time since the last time the receiver has been read
  // and read the data only every 20 ms
  unsigned long currentTime = millis();
  if ((currentTime - previousTime) < 20)
  {
    return;
  }
  previousTime = currentTime;

  /* Lambda to convert microseconds to angle */
  auto convertToAngle = [](unsigned long duration, double amplitudeMax) -> double
  {
    double tmp = (((((double)duration) - 1000.0) / 1000.0) * (amplitudeMax * 2.0)) - amplitudeMax;

    return (tmp < -amplitudeMax) ? (-amplitudeMax) : (tmp > amplitudeMax) ? amplitudeMax : tmp;
  };

  /* Read the radio receiver */
  g_targetRoll = convertToAngle(getRadioChannel(0), TARGET_ANGLE_MAX);
  g_targetPitch = convertToAngle(getRadioChannel(1), TARGET_ANGLE_MAX);
  g_targetYaw = 0.0;
  g_targetThrust = 0.0;
}








