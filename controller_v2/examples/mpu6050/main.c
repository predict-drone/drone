#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>

#include "lib/timer.h"
#include "sensors/mpu6050.h"
#include "lib/i2c_master.h"


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


// Shared variables
static volatile mpu6050_accel_t accel_data;
static volatile mpu6050_gyro_t gyro_data;
static pthread_mutex_t data_mtx = PTHREAD_MUTEX_INITIALIZER;

// Thread 0 vairables
static volatile bool print_flag;

// Thread 1 vairables
static volatile bool read_flag;


static void print_flag_cb(void);
static void read_flag_cb(void);


static void print_flag_cb(void)
{
	print_flag = true;
}

static void read_flag_cb(void)
{
	read_flag = true;
}

static void* thread1_main(void* attr)
{
	printf("MPU6050 example\n");
	//mpu6050_init();
	
	


	int err = 0;
	err = i2c_probe(MPU6050_I2C_ADDRESS);
	printf("MPU probe %d\n", err);
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

	uint64_t time = micros();
	int period = 250000;
	time += period;

	while (true) {
		if (time < micros()) {
			pthread_mutex_lock(&data_mtx);
			accel_data = mpu6050_accel_read();
			gyro_data = mpu6050_gyro_read();
			pthread_mutex_unlock(&data_mtx);
			time += period;
		}
	}
}

int main(void)
{

	timer_init();

	// Init threads
	pthread_t thread1;
	pthread_create(&thread1, NULL, thread1_main, NULL);

	app_timer_t* timer = create_timer(print_flag_cb, 500, 1000);


	while (true) {
		if (print_flag) {
			print_flag = false;

			pthread_mutex_lock(&data_mtx);
			printf("Accel: %d, %d, %d\n", accel_data.x, accel_data.y, accel_data.z);
			printf("Gyro: %d, %d, %d\n", gyro_data.x, gyro_data.y, gyro_data.z);
			pthread_mutex_unlock(&data_mtx);

		}
	}

	return 0;
}
