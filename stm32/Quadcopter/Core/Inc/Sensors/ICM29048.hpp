/*
 * ICM29048.hpp
 *
 *  Created on: Jul 20, 2024
 *      Author: Louis
 */

#pragma once


// STL
#include "stdint.h"

// Driver
#include "stm32h7xx_hal.h"

// Project
#include "Utils/Eigen/Dense"
#include "Sensors/mpu9250.hpp"

// TODO: header for common mpu9250 & ICM29048


// User Bank 0 Registers
#define ICM29048_WHO_AM_I           0x00 // Device ID register
#define ICM29048_USER_CTRL          0x03 // User control register
#define ICM29048_LP_CONFIG          0x05 // Low Power mode configuration
#define ICM29048_PWR_MGMT_1         0x06 // Power management 1
#define ICM29048_PWR_MGMT_2         0x07 // Power management 2
#define ICM29048_INT_PIN_CFG        0x0F // Interrupt pin configuration
#define ICM29048_INT_ENABLE         0x10 // Interrupt enable
#define ICM29048_INT_ENABLE_1       0x11 // Interrupt enable 1
#define ICM29048_INT_ENABLE_2       0x12 // Interrupt enable 2
#define ICM29048_INT_ENABLE_3       0x13 // Interrupt enable 3
#define ICM29048_I2C_MST_STATUS     0x17 // I2C master status
#define ICM29048_INT_STATUS         0x19 // Interrupt status
#define ICM29048_INT_STATUS_1       0x1A // Interrupt status 1
#define ICM29048_INT_STATUS_2       0x1B // Interrupt status 2
#define ICM29048_INT_STATUS_3       0x1C // Interrupt status 3
#define ICM29048_DELAY_TIMEH        0x28 // Delay time high byte
#define ICM29048_DELAY_TIMEL        0x29 // Delay time low byte
#define ICM29048_ACCEL_XOUT_H       0x2D // Accelerometer X-axis high byte
#define ICM29048_ACCEL_XOUT_L       0x2E // Accelerometer X-axis low byte
#define ICM29048_ACCEL_YOUT_H       0x2F // Accelerometer Y-axis high byte
#define ICM29048_ACCEL_YOUT_L       0x30 // Accelerometer Y-axis low byte
#define ICM29048_ACCEL_ZOUT_H       0x31 // Accelerometer Z-axis high byte
#define ICM29048_ACCEL_ZOUT_L       0x32 // Accelerometer Z-axis low byte
#define ICM29048_GYRO_XOUT_H        0x33 // Gyroscope X-axis high byte
#define ICM29048_GYRO_XOUT_L        0x34 // Gyroscope X-axis low byte
#define ICM29048_GYRO_YOUT_H        0x35 // Gyroscope Y-axis high byte
#define ICM29048_GYRO_YOUT_L        0x36 // Gyroscope Y-axis low byte
#define ICM29048_GYRO_ZOUT_H        0x37 // Gyroscope Z-axis high byte
#define ICM29048_GYRO_ZOUT_L        0x38 // Gyroscope Z-axis low byte
#define ICM29048_TEMP_OUT_H         0x39 // Temperature high byte
#define ICM29048_TEMP_OUT_L         0x3A // Temperature low byte
#define ICM29048_EXT_SLV_SENS_DATA_00 0x3B // External sensor data 00
#define ICM29048_EXT_SLV_SENS_DATA_01 0x3C // External sensor data 01
#define ICM29048_EXT_SLV_SENS_DATA_02 0x3D // External sensor data 02
#define ICM29048_EXT_SLV_SENS_DATA_03 0x3E // External sensor data 03
#define ICM29048_EXT_SLV_SENS_DATA_04 0x3F // External sensor data 04
#define ICM29048_EXT_SLV_SENS_DATA_05 0x40 // External sensor data 05
#define ICM29048_EXT_SLV_SENS_DATA_06 0x41 // External sensor data 06
#define ICM29048_EXT_SLV_SENS_DATA_07 0x42 // External sensor data 07
#define ICM29048_EXT_SLV_SENS_DATA_08 0x43 // External sensor data 08
#define ICM29048_EXT_SLV_SENS_DATA_09 0x44 // External sensor data 09
#define ICM29048_EXT_SLV_SENS_DATA_10 0x45 // External sensor data 10
#define ICM29048_EXT_SLV_SENS_DATA_11 0x46 // External sensor data 11
#define ICM29048_EXT_SLV_SENS_DATA_12 0x47 // External sensor data 12
#define ICM29048_EXT_SLV_SENS_DATA_13 0x48 // External sensor data 13
#define ICM29048_EXT_SLV_SENS_DATA_14 0x49 // External sensor data 14
#define ICM29048_EXT_SLV_SENS_DATA_15 0x4A // External sensor data 15
#define ICM29048_EXT_SLV_SENS_DATA_16 0x4B // External sensor data 16
#define ICM29048_EXT_SLV_SENS_DATA_17 0x4C // External sensor data 17
#define ICM29048_EXT_SLV_SENS_DATA_18 0x4D // External sensor data 18
#define ICM29048_EXT_SLV_SENS_DATA_19 0x4E // External sensor data 19
#define ICM29048_EXT_SLV_SENS_DATA_20 0x4F // External sensor data 20
#define ICM29048_EXT_SLV_SENS_DATA_21 0x50 // External sensor data 21
#define ICM29048_EXT_SLV_SENS_DATA_22 0x51 // External sensor data 22
#define ICM29048_EXT_SLV_SENS_DATA_23 0x52 // External sensor data 23
#define ICM29048_FIFO_EN_1          0x66 // FIFO enable 1
#define ICM29048_FIFO_EN_2          0x67 // FIFO enable 2
#define ICM29048_FIFO_RST           0x68 // FIFO reset
#define ICM29048_FIFO_MODE          0x69 // FIFO mode
#define ICM29048_FIFO_COUNTH        0x70 // FIFO count high byte
#define ICM29048_FIFO_COUNTL        0x71 // FIFO count low byte
#define ICM29048_FIFO_R_W           0x72 // FIFO read/write
#define ICM29048_DATA_RDY_STATUS    0x74 // Data ready status
#define ICM29048_FIFO_CFG           0x76 // FIFO configuration
#define ICM29048_REG_BANK_SEL       0x7F // Register bank select

// User Bank 1 Registers
#define ICM29048_SELF_TEST_X_GYRO   0x02 // Self-test for X gyroscope
#define ICM29048_SELF_TEST_Y_GYRO   0x03 // Self-test for Y gyroscope
#define ICM29048_SELF_TEST_Z_GYRO   0x04 // Self-test for Z gyroscope
#define ICM29048_SELF_TEST_X_ACCEL  0x0E // Self-test for X accelerometer
#define ICM29048_SELF_TEST_Y_ACCEL  0x0F // Self-test for Y accelerometer
#define ICM29048_SELF_TEST_Z_ACCEL  0x10 // Self-test for Z accelerometer
#define ICM29048_XA_OFFS_H          0x14 // X accelerometer offset high byte
#define ICM29048_XA_OFFS_L          0x15 // X accelerometer offset low byte
#define ICM29048_YA_OFFS_H          0x17 // Y accelerometer offset high byte
#define ICM29048_YA_OFFS_L          0x18 // Y accelerometer offset low byte
#define ICM29048_ZA_OFFS_H          0x1A // Z accelerometer offset high byte
#define ICM29048_ZA_OFFS_L          0x1B // Z accelerometer offset low byte
#define ICM29048_TIMEBASE_CORRECTION_PLL 0x28 // Timebase correction PLL

// User Bank 2 Registers
#define ICM29048_GYRO_SMPLRT_DIV    0x00 // Gyro sample rate divider
#define ICM29048_GYRO_CONFIG_1      0x01 // Gyro configuration 1
#define ICM29048_GYRO_CONFIG_2      0x02 // Gyro configuration 2
#define ICM29048_XG_OFFS_USRH       0x03 // X gyro offset high byte
#define ICM29048_XG_OFFS_USRL       0x04 // X gyro offset low byte
#define ICM29048_YG_OFFS_USRH       0x05 // Y gyro offset high byte
#define ICM29048_YG_OFFS_USRL       0x06 // Y gyro offset low byte
#define ICM29048_ZG_OFFS_USRH       0x07 // Z gyro offset high byte
#define ICM29048_ZG_OFFS_USRL       0x08 // Z gyro offset low byte
#define ICM29048_ODR_ALIGN_EN       0x09 // Output data rate alignment enable
#define ICM29048_ACCEL_SMPLRT_DIV_1 0x10 // Accel sample rate divider high byte
#define ICM29048_ACCEL_SMPLRT_DIV_2 0x11 // Accel sample rate divider low
#define ICM29048_ACCEL_CONFIG_1     0x14 // Accel configuration 1
#define ICM29048_ACCEL_CONFIG_2     0x15 // Accel configuration 2

// User Bank 3 Registers
#define ICM29048_I2C_MST_ODR_CONFIG   0x00
#define ICM20948_I2C_MST_CTRL         0x01  // I2C master control register
#define ICM20948_I2C_SLV0_ADDR        0x03  // I2C slave 0 address
#define ICM20948_I2C_SLV0_REG         0x04  // I2C slave 0 register address
#define ICM20948_I2C_SLV0_CTRL        0x05  // I2C slave 0 control
#define ICM20948_I2C_SLV0_DO          0x06  // I2C slave 0 data out

// Magnetometer (AK09916) Register Addresses
#define AK09916_WHO_AM_I              0x01  // Device ID register
#define AK09916_STATUS_1              0x10  // Data status 1
#define AK09916_HXL                   0x11  // X-axis magnetic field data lower byte
#define AK09916_HXH                   0x12  // X-axis magnetic field data upper byte
#define AK09916_HYL                   0x13  // Y-axis magnetic field data lower byte
#define AK09916_HYH                   0x14  // Y-axis magnetic field data upper byte
#define AK09916_HZL                   0x15  // Z-axis magnetic field data lower byte
#define AK09916_HZH                   0x16  // Z-axis magnetic field data upper byte
#define AK09916_STATUS_2              0x18  // Data status 2
#define AK09916_CNTL2                 0x31  // Control 2
#define AK09916_CNTL3                 0x32  // Control 3

// ICM-20948 Registers for I2C master mode
#define ICM20948_BIT_I2C_MST_EN       0x20
#define ICM20948_BIT_I2C_IF_DIS       0x10



#define ICM20948_INT_PIN_CFG 0x0F


#define AK09916_I2C_ADDR 0x0C





/*
 * This class handle the MPU9250 IMU
 * through the SPI interface.
 * It configure it, read and filter the raw value.
 */
class ICM29048
{
private:
	// SPI
	SPI_HandleTypeDef m_hspi;
	uint16_t m_spi_cs_pin;
	GPIO_TypeDef* m_spi_cs_gpio_port;

	// Config
	AccScale m_accScaleConf = AccScale::_2G; // 2g is the default value
	GyroScale m_gyroScaleConf = GyroScale::DPS250; // 250 dps is the default value

	uint8_t m_currentBank = 4; // Init current bank to a out of bound value

	// For a range of +-2g, we need to divide the raw values by 16384
	double m_accScale = 16384.0;
	// For 250 dps (default), we need to divide the raw values by 131
	double m_gyroScale = 131.0;

	double m_magScale = 1.0;

	Eigen::Vector<int16_t, 3> m_rawAcc = {0, 0, 0};
	Eigen::Vector<int16_t, 3> m_rawGyro = {0, 0, 0};
	Eigen::Vector<int16_t, 3> m_rawMag = {0, 0, 0};
	int16_t m_rawTemp = 0;

	// Raw value scaled according to setup
	Eigen::Vector3d m_rawScaledAcc = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_rawScaledGyro = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_rawScaledMag = {0.0, 0.0, 0.0};

	// Sensor offset that need to be substracted
	Eigen::Vector3d m_accOffset = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_gyroOffset = {1.56, 1.75, -0.12};

public:
	// Low pass filter for accelerometer
	Eigen::Vector3d m_filteredAcceloremeter = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_previousAcc = {0.0, 0.0, 0.0};
	const double m_lpf_acc_gain = 0.1;

	// Low pass filter for gyroscope
	Eigen::Vector3d m_filteredGyro = {0.0, 0.0, 0.0};
	Eigen::Vector3d m_previousGyro = {0.0, 0.0, 0.0};
	const double m_lpf_gyro_gain = 0.01;

	double m_angleAccX = 0.0;
	double m_angleAccY = 0.0;

public:
	ICM29048(SPI_HandleTypeDef hspi, uint16_t spi_cs_pin, GPIO_TypeDef* spi_cs_gpio_port);
	void init(const AccScale& accScaleConf, const GyroScale& gyroScaleConf);
	void resetDevice();
	void calibrate();
	void read_gyro_acc_data();
	void read_magnetometer_data();
	void filter_and_calibrate_data();

private:
	void configureAccelerometer(const AccScale& accScaleConf);
	void configureGyroscope(const GyroScale& gyroScaleConf);
	bool verifyAccelerometerConfig();
	void select_user_bank(uint8_t bank);
	void configurePowerManagement();
	void ak09916_write(uint8_t reg, uint8_t data);
	void ak09916_read(uint8_t reg, uint8_t len);
	void reset_i2c_master_magnetometer();


	void read_register(const uint8_t bank, uint8_t regAddr, uint8_t* pData, uint8_t len);
	void write_register(const uint8_t bank, uint8_t regAddr, uint8_t data);
	void handle_spi_error(HAL_StatusTypeDef result);
	void init_magnetometer();
	void setup_i2c_slave(uint8_t address, uint8_t reg, uint8_t length, bool read);
	uint8_t read_icm29048_who_am_i();
	uint8_t read_ak8963_who_am_i();
};








