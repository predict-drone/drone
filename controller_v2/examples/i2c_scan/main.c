#include <stdio.h>
#include <stdint.h>
#include <machine/patmos.h>
#include <machine/spm.h>

#include "i2c_master.h"

#define MPU6050_ADDR               0x68
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_PWR_MGMT_1         0x6B   // R/W

int main()
{
	printf("Hello wrold!\n");

	for (uint8_t addr = 1; addr < 128; addr++) {
		if (i2c_probe(addr) == 0) {
			printf("DEVICE at address %02x\n", addr);
		} else {
			printf("No device at address %02x\n", addr);
		}
	}
	return 0;


	int signature = i2c_reg8_read8(MPU6050_ADDR, MPU6050_WHO_AM_I);
	printf("Signature = 0x%.2X\n", signature);


	printf("PWR MGMT = 0x%.2X\n", i2c_reg8_read8(MPU6050_ADDR, MPU6050_PWR_MGMT_1));
	i2c_reg8_write8(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
	printf("PWR MGMT = 0x%.2X\n", i2c_reg8_read8(MPU6050_ADDR, MPU6050_PWR_MGMT_1));


	int16_t accel_x;
	while (1) {
		accel_x = i2c_reg8_read16b(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H);
		printf("Accel x: %d\n", accel_x);
		for (int i = 0; i < 500000; i++);
	}

		
	return 0;
}
