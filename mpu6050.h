

class MPU6050
{
private:
  // For a range of +-8g, we need to divide the raw values by 4096
  const double m_accScale = 4096.0;

  double m_accOffsetX = 0.0;
  double m_accOffsetY = 0.0;
  double m_accOffsetZ = 0.0;

  double m_gyroOffsetX = 1.56;
  double m_gyroOffsetY = 1.75;
  double m_gyroOffsetZ = -0.12;

public:
  double m_filteredAcceloremeterX = 0.0;
  double m_filteredAcceloremeterY = 0.0;
  double m_filteredAcceloremeterZ = 0.0;

  const double m_lpf_acc_gain = 0.1;

  double m_previousAccX = 0.0;
  double m_previousAccY = 0.0;
  double m_previousAccZ = 0.0;

  double m_angleAccX = 0.0;
  double m_angleAccY = 0.0;

  double m_filteredGyroX = 0.0;
  double m_filteredGyroY = 0.0;
  double m_filteredGyroZ = 0.0;

  double m_previousGyroX = 0.0;
  double m_previousGyroY = 0.0;
  double m_previousGyroZ = 0.0;

  const double m_lpf_gyro_gain = 0.01;

public:
  MPU6050();
  void init();
  void calibrate();
  void get_mpu6050_data();
};



