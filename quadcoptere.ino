#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

const int MPU_I2C_ADDR = 0x69; // MPU6050 I2C address
const float COMPLEMENTARY_FILTER_GAIN = 0.96;

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
#define ANGLE_KP 2.0
#define ANGLE_KI 0.0
#define ANGLE_KD 0.0

//#define VELOCITY_KP 0.6
//0.6
//#define VELOCITY_KI 0.0
//3.5
//#define VELOCITY_KD 0.03
//0.03

#define SATURATION 100.0

// TODO: Implement a panic recovery mode, by detecting the free fall with accelerometer

// Store the start and duration time of the high signal for the 4 channel
volatile unsigned long g_pulseStart[4] = {0, 0, 0, 0};
volatile unsigned long g_pulseDuration[4] = {0, 0, 0, 0};

/* functions to read PWM signal, called at any state change (high to low or low to high) */
void handlePWMread(int pin, int channel)
{
  if (digitalRead(pin) == HIGH)
  {
    g_pulseStart[channel] = micros();
  }
  else
  {
    g_pulseDuration[channel] = micros() - g_pulseStart[channel];
  }
}

void readPWM0()
{
  handlePWMread(READ_PWM_CHANNEL_0_PIN, 0);
}

void readPWM1()
{
  handlePWMread(READ_PWM_CHANNEL_1_PIN, 1);
}

void readPWM2()
{
  handlePWMread(READ_PWM_CHANNEL_2_PIN, 2);
}

void readPWM3()
{
  handlePWMread(READ_PWM_CHANNEL_3_PIN, 3);
}


/* PID class */
struct PID
{
  float m_kp = 0.0;
  float m_ki = 0.0;
  float m_kd = 0.0;

  float m_saturation = 100.0;

  float m_previousError = 0.0;
  float m_sommeError = 0.0;

  // Constructor to initialize PID gains
  PID(float kp, float ki, float kd, float sat) : m_kp(kp), m_ki(ki), m_kd(kd), m_saturation(sat)
  {

  }

  float computePID(float error, float dt)
  {
    // Proportionnal gain
    float p = error * m_kp;

    // Derivative gain
    float derivedError = (error - m_previousError) / dt;
    float d = derivedError * m_kd;
    m_previousError = error;

    // Integral gain
    m_sommeError += error * dt;
    // Prevent windup
    if (m_sommeError > m_saturation)
    {
      m_sommeError = m_saturation;
    }
    else if (m_sommeError < -m_saturation)
    {
      m_sommeError = -m_saturation;
    }
    float i = m_sommeError * m_ki;

    return p + i + d;
  }
};


/*struct DualLoopControl
{
  PID m_angleControl;
  PID m_velocityControl;

  // Constructor to initialize PID controllers with specific gains
  DualLoopControl(float angleKp, float angleKi, float angleKd, float velocityKp, float velocityKi, float velocityKd)
  : m_angleControl(angleKp, angleKi, angleKd), m_velocityControl(velocityKp, velocityKi, velocityKd)
  {

  }

  float computeMotorPower(float targetAngle, float angle, float velocity, float dt)
  {
    float targetVelocity = angleControlLoop(targetAngle, angle, dt);

    return velocityControlLoop(targetVelocity, velocity, dt);
  }

  // Return the target velocity
  float angleControlLoop(float targetAngle, float angle, float dt)
  {
    float angleError = targetAngle - angle;

    // Outer loop
    return m_angleControl.computePID(angleError, dt);
  }

  // Return the motor power
  float velocityControlLoop(float targetVelocity, float velocity, float dt)
  {
    float velocityError = targetVelocity - velocity;

    // Inner loop
    return m_velocityControl.computePID(velocityError, dt);
  }
};


// 
DualLoopControl attitudeControl_roll(ANGLE_KP, ANGLE_KI, ANGLE_KD, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD);
DualLoopControl attitudeControl_pitch(ANGLE_KP, ANGLE_KI, ANGLE_KD, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD);
DualLoopControl attitudeControl_yaw(ANGLE_KP, ANGLE_KI, ANGLE_KD, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD);
*/

PID attitudeControl_roll(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_pitch(ANGLE_KP, ANGLE_KI, ANGLE_KD, SATURATION);
PID attitudeControl_yaw(ANGLE_KP, 0.0, ANGLE_KD, SATURATION);

/* Create Servo objects to control the 4 ESCs */
Servo g_esc_motor_1; // Servo object to control ESC 1
Servo g_esc_motor_2; // Servo object to control ESC 2
Servo g_esc_motor_3; // Servo object to control ESC 3
Servo g_esc_motor_4; // Servo object to control ESC 4


float g_gyroVelocityX = 0.0;
float g_gyroVelocityY = 0.0;
float g_gyroVelocityZ = 0.0;

float g_roll = 0.0;
float g_pitch = 0.0;
float g_yaw = 0.0;


void setup()
{
  delay(250);

  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_I2C_ADDR);       // Start communication with MPU6050 // MPU=0x69
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  /*Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
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

  /* Configure interrupt for radio receiver reading */
  pinMode(READ_PWM_CHANNEL_0_PIN, INPUT);
  pinMode(READ_PWM_CHANNEL_1_PIN, INPUT);
  pinMode(READ_PWM_CHANNEL_2_PIN, INPUT);
  pinMode(READ_PWM_CHANNEL_3_PIN, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(READ_PWM_CHANNEL_0_PIN), readPWM0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(READ_PWM_CHANNEL_1_PIN), readPWM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(READ_PWM_CHANNEL_2_PIN), readPWM2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(READ_PWM_CHANNEL_3_PIN), readPWM3, CHANGE);
  
  delay(1000); // wait for a second
}

void loop()
{
  static unsigned long previousTime = micros();
  
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - previousTime) / 1000000.0; // Divide by 1000000 to get seconds
  previousTime = currentTime;
  if (elapsedTime == 0.0)
  {
    elapsedTime = 1.0; // Avoid dividing by zero
  }

  // Get data from IMU and fuse it to compute attitude
  get_mpu6050_data(elapsedTime);
  //calculate_IMU_error();

  // Read radio receiver
  float targetRoll = 0.0;
  float targetPitch = 0.0;
  float targetYaw = 0.0;
  float targetThrust = 0.0;

  // Compute angular errors
  float error_roll = targetRoll - g_roll;
  float error_pitch = targetPitch - g_pitch;
  float error_yaw = targetYaw - g_yaw;

  // Compute PIDs
  float pidRoll = attitudeControl_roll.computePID(error_roll, elapsedTime);
  float pidPitch = attitudeControl_pitch.computePID(error_pitch, elapsedTime);
  float pidYaw = attitudeControl_yaw.computePID(error_yaw, elapsedTime);

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
  float motor1Power = targetThrust - pidPitch + pidYaw; // Front left
  float motor2Power = targetThrust + pidRoll - pidYaw;  // Front right
  float motor3Power = targetThrust + pidPitch + pidYaw; // Back right
  float motor4Power = targetThrust - pidRoll - pidYaw;  // Back left

  // Set PWMs

  //delay(20);

  int speed = 1500; // Set speed (1000 = off, 2000 = full speed)
  g_esc_motor_1.writeMicroseconds(1000); // Send speed to ESC
  g_esc_motor_2.writeMicroseconds(1300); // Send speed to ESC
  g_esc_motor_3.writeMicroseconds(1700); // Send speed to ESC
  g_esc_motor_4.writeMicroseconds(2000); // Send speed to ESC
  delay(15); // wait for a refresh cycle

  noInterrupts(); // Temporarily disable interrupts to ensure consistent readings
  for (int i = 0; i < 4; i++)
  {
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(g_pulseDuration[i]);
  }
  interrupts(); // Re-enable interrupts
  delay(1000); // Delay for readability

}






void get_mpu6050_data(float dt)
{
  const float accOffsetX = 0.0;
  const float accOffsetY = 0.0;
  const float accOffsetZ = 0.0;

  const float gyroOffsetX = 1.56;
  const float gyroOffsetY = 1.75;
  const float gyroOffsetZ = -0.12;

  //static float accVelocityX = 0.0;
  //static float accVelocityY = 0.0;
  //static float accVelocityZ = 0.0;

  //static float accPosX = 0.0;
  //static float accPosY = 0.0;
  //static float accPosZ = 0.0;

  static float gyroAngleX = 0.0;

  static int print = 0;

  // Read acceleromter
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true);

  // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  //So for a range of +-8g, we need to divide the raw values by 4096
  float accX = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float accY = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float accZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
  // Remove offset
  accX -= accOffsetX;
  accY -= accOffsetY;
  accZ -= accOffsetZ;

  /*Serial.print(accX);
  Serial.print("     ");
  Serial.print(accY);
  Serial.print("     ");
  Serial.println(accZ);*/

  // Calculating Roll and Pitch from the accelerometer data
  float accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI);
  float accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI);

  /*if (isnan(accAngleX))
  {
    accAngleX = 0.0;
  }
  if (isnan(accAngleY))
  {
    accAngleY = 0.0;
  }*/

  /*Serial.print(accAngleX);
  Serial.print("     ");
  Serial.println(accAngleY);*/

  // Read gyroscope
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR, 6, true);

  g_gyroVelocityX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  g_gyroVelocityY = (Wire.read() << 8 | Wire.read()) / 131.0;
  g_gyroVelocityZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Correct the outputs with the calculated error values
  g_gyroVelocityX -= gyroOffsetX;
  g_gyroVelocityY -= gyroOffsetY;
  g_gyroVelocityZ -= gyroOffsetZ;

  /*Serial.print(g_gyroVelocityX);
  Serial.print("     ");
  Serial.print(g_gyroVelocityY);
  Serial.print("     ");
  Serial.println(g_gyroVelocityZ);*/

  gyroAngleX += (g_gyroVelocityX * dt);

  // Complementary filter
  g_roll = COMPLEMENTARY_FILTER_GAIN * (g_roll + (g_gyroVelocityX * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleX;
  g_pitch = COMPLEMENTARY_FILTER_GAIN * (g_pitch + (g_gyroVelocityY * dt)) + (1.0 - COMPLEMENTARY_FILTER_GAIN) * accAngleY;
  g_yaw += g_gyroVelocityZ * dt;

  /*if (print == 25)
  {
    print = 0;

    Serial.print(g_roll);
    Serial.print("     ");
    Serial.print(g_pitch);
    Serial.print("     ");
    Serial.println(g_yaw);
  }
  print++;*/

  /*Serial.print(accAngleX);
  Serial.print("     ");
  Serial.print(gyroAngleX);
  Serial.print("     ");
  Serial.println(g_roll);*/

}






void calculate_IMU_error()
{
  float accX = 0;
  float accY = 0;
  float accZ = 0;

  for (int i = 0; i < 200; ++i)
  {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, 6, true);

    accX += (Wire.read() << 8 | Wire.read()) / 16384.0;
    accY += (Wire.read() << 8 | Wire.read()) / 16384.0;
    accZ += (Wire.read() << 8 | Wire.read()) / 16384.0;
  }

  //Divide the sum by 200 to get the error value
  accX /= 200.0;
  accY /= 200.0;
  accZ /= 200.0;
  accZ -= 1.0;

  Serial.print("AccErrorX: ");
  Serial.println(accX);
  Serial.print("AccErrorY: ");
  Serial.println(accY);
  Serial.print("AccErrorZ: ");
  Serial.println(accZ);


  float gyroErrorX = 0.0;
  float gyroErrorY = 0.0;
  float gyroErrorZ = 0.0;
  
  // Read gyro values 200 times
  for (int i = 0; i < 200; ++i)
  {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, 6, true);

    int gyroX = Wire.read() << 8 | Wire.read();
    int gyroY = Wire.read() << 8 | Wire.read();
    int gyroZ = Wire.read() << 8 | Wire.read();

    // Sum all readings
    gyroErrorX += (gyroX / 131.0);
    gyroErrorY += (gyroY / 131.0);
    gyroErrorZ += (gyroZ / 131.0);
  }

  //Divide the sum by 200 to get the error value
  gyroErrorX /= 200.0;
  gyroErrorY /= 200.0;
  gyroErrorZ /= 200.0;

  // Print the error values on the Serial Monitor
  Serial.print("GyroErrorX: ");
  Serial.println(gyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(gyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(gyroErrorZ);
}






