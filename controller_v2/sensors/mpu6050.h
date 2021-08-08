#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>


typedef struct mpu6050_accel_t {
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((packed)) mpu6050_accel_t;

typedef struct mpu6050_gyro_t {
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((packed)) mpu6050_gyro_t;


void mpu6050_init(void);

mpu6050_accel_t mpu6050_accel_read(void);

mpu6050_gyro_t mpu6050_gyro_read(void);

#endif // __MPU6050_H
