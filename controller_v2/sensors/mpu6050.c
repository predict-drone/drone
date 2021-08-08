#include "lib/i2c_master.h"
#include "mpu6050.h"

// Default I2C address for the MPU-6050 is 0x68.
#define MPU6050_I2C_ADDRESS 0x68
//
//MCU6050 registers
#define MPU6050_WHO_AM_I        0x75  // R
#define MPU6050_CONFIG          0x1A  // R/W
#define MPU6050_GYRO_CONFIG     0x1B  // R/w
#define MPU6050_ACCEL_CONFIG    0x1C  // R/W
#define MPU6050_PWR_MGMT_1      0x6B  // R/W
#define MPU6050_PWR_MGMT_2      0x6C  // R/W
#define MPU6050_ACCEL_XOUT_H    0x3B  // R
#define MPU6050_ACCEL_XOUT_L    0x3C  // R
#define MPU6050_ACCEL_YOUT_H    0x3D  // R
#define MPU6050_ACCEL_YOUT_L    0x3E  // R
#define MPU6050_ACCEL_ZOUT_H    0x3F  // R
#define MPU6050_ACCEL_ZOUT_L    0x40  // R
#define MPU6050_TEMP_OUT_H      0x41  // R
#define MPU6050_TEMP_OUT_L      0x42  // R
#define MPU6050_GYRO_XOUT_H     0x43  // R
#define MPU6050_GYRO_XOUT_L     0x44  // R
#define MPU6050_GYRO_YOUT_H     0x45  // R
#define MPU6050_GYRO_YOUT_L     0x46  // R
#define MPU6050_GYRO_ZOUT_H     0x47  // R
#define MPU6050_GYRO_ZOUT_L     0x48  // R


void mpu6050_init(void)
{
	// Reset device and registers
	i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x10);
	// Disable sleep and configure gyro_x clock source
	i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);
	// Filter (faster)
	i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_CONFIG, 0x00);
	// Gyro range 1000 dps
	i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0x10);
	// Accel range 8g
	i2c_reg8_write8(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10);
}

mpu6050_accel_t mpu6050_accel_read(void)
{
	mpu6050_accel_t data;
	i2c_reg8_read_stream(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H,
			(uint8_t*) &data, sizeof(data));
	return data;
}

mpu6050_gyro_t mpu6050_gyro_read(void)
{
	mpu6050_gyro_t data;
	i2c_reg8_read_stream(MPU6050_I2C_ADDRESS, MPU6050_GYRO_XOUT_H,
			(uint8_t*) &data, sizeof(data));
	return data;
}
