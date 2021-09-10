#include <stdint.h>
#include <stdio.h>
#include <machine/fastmath.h>
#include <math.h>

#include "angles.h"
#include "main.h"
#include "lib/timer.h"
#include "util.h"

#define SETUP_MEASUREMENTS 500


/** Data structures --------------------------------------------------------- */
typedef struct angles_axis_t {
	double x;
	double y;
	double z;
} angles_axis_t;

/** Static variables -------------------------------------------------------- */
static bool init;
static double pitch_avg = 0;
static double roll_avg = 0;
static double yaw_avg = 0;
static double gyro_x_avg = 0;
static double gyro_y_avg = 0;
static double gyro_z_avg = 0;
static double alpha = 0.8;

static pthread_mutex_t init_mtx = PTHREAD_MUTEX_INITIALIZER;
static angles_axis_t gyro_err;
static angles_axis_t accel_err;
//static angles_axis_t gyro;
//static angles_axis_t accel;

static angles_t position;
static angles_t angle_vel;
static uint64_t last_time;
static pthread_mutex_t position_mtx = PTHREAD_MUTEX_INITIALIZER;

/** Function prototypes ----------------------------------------------------- */
//static void calculate_angles_gyro(void);
//static void calculate_angles_accel(void);

/** Function definitions ---------------------------------------------------- */
static angles_axis_t calculate_angles_gyro(void)
{
	mpu6050_gyro_t gyro_data = mpu6050_gyro_read();
	double x_dps = gyro_data.x / 32.8 - gyro_err.x;
	double y_dps = gyro_data.y / 32.8 - gyro_err.y;
	double z_dps = gyro_data.z / 32.8 - gyro_err.z;

	uint64_t current_time = micros();
	double elapsed_time = (current_time - last_time) / 1000000.0;
	last_time = current_time;

	angles_axis_t gyro;
	gyro.x = x_dps * elapsed_time;
	gyro.y = y_dps * elapsed_time;
	gyro.z = z_dps * elapsed_time;
	return gyro;
}

static angles_axis_t calculate_angle_vel_gyro(void)
{
	mpu6050_gyro_t gyro_data = mpu6050_gyro_read();
	double x_dps = gyro_data.x / 32.8 - gyro_err.x;
	double y_dps = gyro_data.y / 32.8 - gyro_err.y;
	double z_dps = gyro_data.z / 32.8 - gyro_err.z;

	angles_axis_t gyro;
	gyro.x = x_dps;
	gyro.y = y_dps;
	gyro.z = z_dps;
	return gyro;
}

static angles_axis_t calculate_angles_accel(void)
{
	mpu6050_accel_t accel_data = mpu6050_accel_read();
	double x = accel_data.x;
	double y = accel_data.y;
	double z = accel_data.z;
	angles_axis_t accel;
	accel.x = -(atan(x / (sqrt(pow(y, 2) + pow(z, 2)))) - accel_err.x)*180/M_PI;		//Converts the accelerometer data to angle position [degrees].
	accel.y = (atan(y / (sqrt(pow(x, 2) + pow(z, 2)))) - accel_err.y)*180/M_PI;
	accel.z = (atan(z / (sqrt(pow(y, 2) + pow(x, 2)))) - accel_err.z)*180/M_PI;
	return accel;
}

/** Public functions -------------------------------------------------------- */
void angles_init(void)
{
	pthread_mutex_lock(&print_mtx);
	printf("Calibrating MPU\n");
	pthread_mutex_unlock(&print_mtx);

	mpu6050_init();
	delay(202);

	int32_t gyro_x = 0;
	int32_t gyro_y = 0;
	int32_t gyro_z = 0;
	int32_t accel_x = 0;
	int32_t accel_y = 0;
	int32_t accel_z = 0;

	for (int i = 0; i < SETUP_MEASUREMENTS; i++) {
		mpu6050_gyro_t gyro_data = mpu6050_gyro_read();
		gyro_x += gyro_data.x;
		gyro_y += gyro_data.y;
		gyro_z += gyro_data.z;

		mpu6050_accel_t accel_data = mpu6050_accel_read();
		accel_x += accel_data.x;
		accel_y += accel_data.y;
		accel_z += accel_data.z;

		delay(10);
	}

	gyro_err.x = gyro_x / 32.8 / (double) SETUP_MEASUREMENTS;
	gyro_err.y = gyro_y / 32.8 / (double) SETUP_MEASUREMENTS;
	gyro_err.z = gyro_z / 32.8 / (double) SETUP_MEASUREMENTS;

	double x = accel_x / (double) SETUP_MEASUREMENTS;
	double y = accel_y / (double) SETUP_MEASUREMENTS;
	double z = accel_z / (double) SETUP_MEASUREMENTS;
	accel_err.x = atan(x / (sqrt(pow(y, 2) + pow(z, 2))));
	accel_err.y = atan(y / (sqrt(pow(x, 2) + pow(z, 2))));
	accel_err.z = atan(z / (sqrt(pow(y, 2) + pow(x, 2))));

	pthread_mutex_lock(&print_mtx);
	printf("Gyro err: %.2f, %.2f, %.2f\n", gyro_err.y, gyro_err.x, gyro_err.z);
	printf("Accel err: %.2f, %.2f, %.2f\n", accel_err.z, accel_err.y, accel_err.z);
	pthread_mutex_unlock(&print_mtx);
	last_time = micros();

	pthread_mutex_lock(&init_mtx);
	init = true;
	pthread_mutex_unlock(&init_mtx);
}

void calculate_angles(void) {
	angles_axis_t gyro = calculate_angles_gyro();
	angles_axis_t gyro_vel = calculate_angle_vel_gyro();
	angles_axis_t accel = calculate_angles_accel();

	double pitch_acc = 0;
	double roll_acc = 0;
	double yaw_acc = 0;

	pthread_mutex_lock(&position_mtx);

	pitch_acc = 0.95 * (position.pitch + gyro.y) + 0.05 * accel.x;		//Complementary filtering: Mixing acc and gyro data to minimise both noise and drift. [degrees]
	roll_acc = 0.95 * (position.roll + gyro.x) + 0.05 * accel.y;
	yaw_acc = 0.95 * (position.yaw + gyro.z) + 0.05 * accel.z;

	pitch_avg += alpha * (pitch_acc - pitch_avg);
	roll_avg += alpha * (roll_acc - roll_avg);
	yaw_avg += alpha * (yaw_acc - yaw_avg);

	gyro_x_avg += alpha * (gyro_vel.x - gyro_x_avg);
	gyro_y_avg += alpha * (gyro_vel.y - gyro_y_avg);
	gyro_z_avg += alpha * (gyro_vel.z - gyro_z_avg);

	position.pitch = pitch_avg;
	position.roll = roll_avg;
	position.yaw = yaw_avg;
	
	angle_vel.pitch = gyro_y_avg;
	angle_vel.roll = gyro_x_avg;
	angle_vel.yaw = gyro_z_avg;

	if (position.yaw > 90) position.yaw = 90;		//We should set limits to it
	if (position.yaw < -90) position.yaw = -90;
	pthread_mutex_unlock(&position_mtx);
}

angles_t get_angles(void)
{
	pthread_mutex_lock(&position_mtx);
	angles_t aux = position;
	pthread_mutex_unlock(&position_mtx);
	return aux;
}

angles_t get_angle_vel(void)
{
	pthread_mutex_lock(&position_mtx);
	angles_t aux = angle_vel;
	pthread_mutex_unlock(&position_mtx);
	return aux;
}

bool angles_is_init(void)
{
	bool is_init;
	pthread_mutex_lock(&init_mtx);
	is_init = init;
	pthread_mutex_unlock(&init_mtx);
	return is_init;
}
