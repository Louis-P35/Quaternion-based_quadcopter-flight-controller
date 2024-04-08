#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#include "readPWM.h"
#include "pid.h"
#include "kalman.h"
#include "utils.h"
#include "mpu6050.h"


const int HMC5883L_I2C_ADDR = 0x1E;             // HMC5883L I2C address (magnetometer)
const int MS5611_I2C_ADDR = 0x77;               // MS5611 I2C address (barometer)

#define RAD_TO_DEGREE 57.2957795

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

enum class DroneState
{
  CALIBRATING,
  IDLE,
  FLYING
};

// TODO: Implement a panic recovery mode, by detecting the free fall with accelerometer
// TODO: Disable PID I term until take off
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


// mpu6050 IMU (accelerometer + gyroscope)
MPU6050 g_imu = MPU6050();

// Kalman filters, to perform the data fusion between the gyroscope and accelerometer
Kalman g_kalman_roll = Kalman(0.001, 0.003, 0.03);
Kalman g_kalman_pitch = Kalman(0.001, 0.003, 0.03);

// PIDs to perform the motors control feedback loop
PID attitudeControl_roll = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_pitch = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_yaw = PID(ANGLE_KP, 0.0, ANGLE_KD, SATURATION);

// Create Servo objects to control the 4 ESCs
Servo g_esc_motor_1; // Servo object to control ESC 1
Servo g_esc_motor_2; // Servo object to control ESC 2
Servo g_esc_motor_3; // Servo object to control ESC 3
Servo g_esc_motor_4; // Servo object to control ESC 4





double g_magnetometerAngle = 0.0;

double g_targetRoll = 0.0;
double g_targetPitch = 0.0;
double g_targetYaw = 0.0;
double g_targetThrust = 0.0;

// Current state of the drone
DroneState g_state = DroneState::CALIBRATING;


void calibrateIMU();
void readRadioReceiver();


void magnetometerSetup()
{
  // HMC5883L configuration
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x00); // Configuration register A
  Wire.write(0x70); // 8 averaged samples, 15Hz frequency, normal mode
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x01); // Configuration register B
  Wire.write(0xA0); // Gain = 5
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x02); // Mode register
  Wire.write(0x00); // Continus measure mode
  Wire.endTransmission();
}


void setup()
{
  delay(250);

  Serial.begin(19200);

  // Init the mpu6050 IMU
  g_imu.init();

  // Calibrate the accelerometer and gyroscope offset
  g_imu.calibrate();
  
  //calculate_IMU_error();
  //delay(20);

  /* Attach the Servo objects to their corresponding PWM pin */
  g_esc_motor_1.attach(PWM_PIN_1);
  g_esc_motor_2.attach(PWM_PIN_2);
  g_esc_motor_3.attach(PWM_PIN_3);
  g_esc_motor_4.attach(PWM_PIN_4);
  g_esc_motor_1.writeMicroseconds(1000); // initialize the signal to a low value
  g_esc_motor_2.writeMicroseconds(1000); // initialize the signal to a low value
  g_esc_motor_3.writeMicroseconds(1000); // initialize the signal to a low value
  g_esc_motor_4.writeMicroseconds(1000); // initialize the signal to a low value

  // Configure interrupt for radio receiver reading
  setup_PWM_reader(READ_PWM_CHANNEL_0_PIN, READ_PWM_CHANNEL_1_PIN, READ_PWM_CHANNEL_2_PIN, READ_PWM_CHANNEL_3_PIN);
  
  delay(1000); // wait for a second

  // Setup of the 3 axis magnetometer
  magnetometerSetup();
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

  // Fetch the data from IMU (accel and gyro) and filter it
  // Also compute attitude with accelerometer only
  g_imu.get_mpu6050_data(elapsedTime);

  // Fetch the data from the magnetometer
  // and compute the angle (yaw) in degree
  //fetch_magnetometer_data();

  // Fuse the data (accelerometer and gyroscope) with Kalman filter to compute attitude
  double kalmanRoll = g_kalman_roll.compute(g_imu.m_angleAccX, g_imu.m_filteredGyroX);
  double kalmanPitch = g_kalman_pitch.compute(g_imu.m_angleAccY, g_imu.m_filteredGyroY);
  double kalmanYaw = 0.0;//g_kalman_yaw.compute(g_imu.m_angleAccY, g_imu.m_filteredGyroY);


  /*Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.println(pitch);*/

  //Serial.print(",");
  //Serial.print("Yaw:");
  //Serial.println(g_magnetometerAngle);

  /*Serial.print("Kalman:");
  Serial.print(roll);
  Serial.print(",");
  Serial.print("Complementary_filter:");
  Serial.print(g_roll);
  Serial.print(" :");*/
  //Serial.println(elapsedTime*1000.0);

  // Compute angular errors
  double error_roll = g_targetRoll - kalmanRoll;
  double error_pitch = g_targetPitch - kalmanPitch;
  double error_yaw = g_targetYaw - kalmanYaw;

  // Compute PIDs
  double pidRoll = attitudeControl_roll.computePID(error_roll, g_state == DroneState::FLYING);
  double pidPitch = attitudeControl_pitch.computePID(error_pitch, g_state == DroneState::FLYING);
  double pidYaw = attitudeControl_yaw.computePID(error_yaw, g_state == DroneState::FLYING);

  /*float aa = 500.0;
  float bb = -500.0;
  Serial.print(aa);
  Serial.print("  ");
  Serial.print(bb);
  Serial.print("  ");
  Serial.println(pidRoll);*/

  // Scale PIDs values to their desired range


  // Calculate motors power
  // Motor 1: Front left
  // Motor 2: Front right
  // Motor 3: Back right
  // Motor 4: Back left
  // Apply PID adjustments to each motor
  double motor1Power = g_targetThrust - pidPitch + pidYaw; // Front left FRONT
  double motor2Power = g_targetThrust + pidRoll - pidYaw;  // Front right RIGHT
  double motor3Power = g_targetThrust + pidPitch + pidYaw; // Back right BACK
  double motor4Power = g_targetThrust - pidRoll - pidYaw;  // Back left LEFT

  Serial.print(motor1Power);
  Serial.print("  ");
  Serial.print(motor2Power);
  Serial.print("  ");
  Serial.print(motor3Power);
  Serial.print("  ");
  Serial.println(motor4Power);


  // Set PWMs to control ESCs
  g_esc_motor_1.writeMicroseconds(1000 + (int)motor1Power); // Send speed to ESC
  g_esc_motor_2.writeMicroseconds(1000 + (int)motor2Power); // Send speed to ESC
  g_esc_motor_3.writeMicroseconds(1000 + (int)motor3Power); // Send speed to ESC
  g_esc_motor_4.writeMicroseconds(1000 + (int)motor4Power); // Send speed to ESC


  //delay(15); // wait for a refresh cycle

  //for (int i = 0; i < 4; i++)
  /*{
    int i = 0;
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": ");
    
    Serial.println(g_targetRoll);
  }
  delay(1000); // Delay for readability*/

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
  g_targetRoll = 0.0; //convertToAngle(getRadioChannel(0), TARGET_ANGLE_MAX);
  g_targetPitch = 0.0;//convertToAngle(getRadioChannel(1), TARGET_ANGLE_MAX);
  g_targetYaw = 0.0;
  g_targetThrust = 0.0;
}



void fetch_magnetometer_data()
{
  static unsigned long previousFetchTime = micros();
  
  // Get the ellapsed time since the last time data was fetched
  unsigned long currentTime = micros();
  double elapsedTime = (currentTime - previousFetchTime) / 1000000.0; // Divide by 1000000 to get seconds
  
  // Read data throught I2C only at 15 Hz
  if (elapsedTime < (1.0/15.0))
  {
    return;
  }
  previousFetchTime = currentTime;
  
  // Start reading from data register on X MSB axis
  Wire.beginTransmission(HMC5883L_I2C_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  
  // Request 6 bytes of data : 2 for each axis
  Wire.requestFrom(HMC5883L_I2C_ADDR, 6);
  if (6 <= Wire.available())
  {
    int16_t x = Wire.read() << 8 | Wire.read(); // X axis
    int16_t y = Wire.read() << 8 | Wire.read(); // Z axis (careful about axis order)
    int16_t z = Wire.read() << 8 | Wire.read(); // Y axis

    // Compute the angle in radians
    double angleRadians = atan2((double)y, (double)x);

    // Convert it in degrees
    g_magnetometerAngle = angleRadians * RAD_TO_DEGREE;
  }
}







