#include <stdint.h>
#include <stdio.h>
#include <machine/fastmath.h>
#include <math.h>

#include "angles.h"
#include "main.h"
#include "lib/timer.h"
#include "util.h"
#include "atan_table.h"

#define SETUP_MEASUREMENTS 512U
#define SETUP_MEASUREMENTS_FIX (fixedpt_fromint(SETUP_MEASUREMENTS))
#define ALPHA (fixedpt_rconst(0.8))


/** Data structures --------------------------------------------------------- */
typedef struct angles_accel_axis_t {
	fixedpt x; /** Accel x position (deg) */
	fixedpt y; /** Accel y position (deg) */
} angles_accel_axis_t;

typedef struct angles_gyro_axis_t {
	fixedpt x;   /** Gyro x axis position (deg) */
	fixedpt y;   /** Gyro y axis position (deg) */
	fixedpt z;   /** Gyro z axis position (deg) */
	fixedpt x_s; /** Gyro x axis speed (deg/s) */
	fixedpt y_s; /** Gyro y axis speed (deg/s) */
	fixedpt z_s; /** Gyro z axis speed (deg/s) */
} angles_gyro_axis_t;

/** Static variables -------------------------------------------------------- */
static bool init; /** Whether the module has been initialized */
static pthread_mutex_t init_mtx = PTHREAD_MUTEX_INITIALIZER;

static angles_gyro_axis_t gyro_err;  /** Gyroscope initial error */
static angles_accel_axis_t accel_err; /** Accelerometer initial error */

static angles_t angles; /** Angles position and speed */
static pthread_mutex_t angles_mtx = PTHREAD_MUTEX_INITIALIZER;
static uint64_t gyro_last_time_us; /** Gyro last measurement time */

/** Function prototypes ----------------------------------------------------- */
static angles_gyro_axis_t calculate_angles_gyro(void);
static angles_accel_axis_t calculate_angles_accel(void);
static inline fixedpt combine(fixedpt pos, fixedpt gyro, fixedpt accel);

/** Function definitions ---------------------------------------------------- */
static angles_gyro_axis_t calculate_angles_gyro(void)
{
	angles_gyro_axis_t gyro;
	mpu6050_gyro_t gyro_data = mpu6050_gyro_read();
	gyro.x_s = fixedpt_xdiv(fixedpt_fromint(gyro_data.x), fixedpt_rconst(32.8))
		- gyro_err.x_s;
	gyro.y_s = fixedpt_div(fixedpt_fromint(gyro_data.y), fixedpt_rconst(32.8))
		- gyro_err.y_s;
	gyro.z_s = fixedpt_div(fixedpt_fromint(gyro_data.z), fixedpt_rconst(32.8))
		- gyro_err.z_s;

	uint64_t current_time = micros();
	fixedpt elapsed_time = (fixedpt) fixedpt_xdiv(
			fixedpt_fromint(current_time - gyro_last_time_us),
			fixedpt_fromint(1000000));
	gyro_last_time_us = current_time;

	gyro.x = fixedpt_mul(gyro.x_s, elapsed_time);
	gyro.y = fixedpt_mul(gyro.y_s, elapsed_time);
	gyro.z = fixedpt_mul(gyro.z_s, elapsed_time);
	return gyro;
}

static angles_accel_axis_t calculate_angles_accel(void)
{
	mpu6050_accel_t accel_data = mpu6050_accel_read();

	fixedpt x = (fixedpt_fromint(accel_data.x) >> 8);
	fixedpt y = (fixedpt_fromint(accel_data.y) >> 8);
	fixedpt z = (fixedpt_fromint(accel_data.z) >> 8);
	fixedpt x2 = fixedpt_mul(x, x);
	fixedpt y2 = fixedpt_mul(y, y);
	fixedpt z2 = fixedpt_mul(z, z);
	fixedpt xs = fixedpt_sqrt(y2 + z2);
	fixedpt ys = fixedpt_sqrt(x2 + z2);
	fixedpt aux_x = fixedpt_div(x, xs);
	fixedpt aux_y = fixedpt_div(y, ys);
	angles_accel_axis_t accel;
	accel.x = - atan_table(aux_x) - accel_err.x;
	accel.y = atan_table(aux_y) - accel_err.y;
	return accel;
}

static inline fixedpt combine(fixedpt pos, fixedpt gyro, fixedpt accel)
{
	return fixedpt_mul(fixedpt_rconst(0.95), (pos + gyro))
		+ fixedpt_mul(fixedpt_rconst(0.05), accel);
}

/** Public functions -------------------------------------------------------- */
void angles_init(void)
{
	pthread_mutex_lock(&print_mtx);
	printf("Calibrating MPU\n");
	pthread_mutex_unlock(&print_mtx);

	mpu6050_init();
	delay(200);

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

	gyro_err.x_s = (fixedpt) fixedpt_xdiv(fixedpt_xdiv(fixedpt_fromint(gyro_x),
				fixedpt_rconst(32.8)), SETUP_MEASUREMENTS_FIX);
	gyro_err.y_s = fixedpt_div(fixedpt_div(fixedpt_fromint(gyro_y),
				fixedpt_rconst(32.8)), SETUP_MEASUREMENTS_FIX);
	gyro_err.z_s = fixedpt_div(fixedpt_div(fixedpt_fromint(gyro_z),
				fixedpt_rconst(32.8)), SETUP_MEASUREMENTS_FIX);

	fixedpt x = (fixedpt_fromint(accel_x) >> 16);
	fixedpt y = (fixedpt_fromint(accel_y) >> 16);
	fixedpt z = (fixedpt_fromint(accel_z) >> 16);
	fixedpt x2 = fixedpt_mul(x, x);
	fixedpt y2 = fixedpt_mul(y, y);
	fixedpt z2 = fixedpt_mul(z, z);
	fixedpt xs = fixedpt_sqrt(y2 + z2);
	fixedpt ys = fixedpt_sqrt(x2 + z2);
	fixedpt aux_x = fixedpt_div(x, xs);
	fixedpt aux_y = fixedpt_div(y, ys);
	accel_err.x = - atan_table(aux_x);
	accel_err.y = atan_table(aux_y);

	pthread_mutex_lock(&print_mtx);
	printf("Gyro err: %.2f, %.2f, %.2f\n", fixedpt_tofloat(gyro_err.y_s),
			fixedpt_tofloat(gyro_err.x_s), fixedpt_tofloat(gyro_err.z_s));
	printf("Accel err: %.6f, %.6f\n", fixedpt_tofloat(accel_err.x),
			fixedpt_tofloat(accel_err.y));
	pthread_mutex_unlock(&print_mtx);

	pthread_mutex_lock(&init_mtx);
	init = true;
	pthread_mutex_unlock(&init_mtx);

	gyro_last_time_us = micros();
}

void calculate_angles(void) {
	angles_gyro_axis_t gyro = calculate_angles_gyro();
	angles_accel_axis_t accel = calculate_angles_accel();

	fixedpt pitch = combine(angles.pitch, gyro.y, accel.x);
	fixedpt roll = combine(angles.roll, gyro.x, accel.y);

	pthread_mutex_lock(&angles_mtx);
	angles.pitch = pitch;
	angles.roll = roll;
	angles.yaw += gyro.z;
	angles.pitch_speed = gyro.y_s;
	angles.roll_speed = gyro.x_s;
	angles.yaw_speed = gyro.z_s;

	if (angles.yaw > fixedpt_fromint(180)) angles.yaw -= fixedpt_fromint(360);
	if (angles.yaw < fixedpt_fromint(-180)) angles.yaw += fixedpt_fromint(360);

	pthread_mutex_unlock(&angles_mtx);
}

angles_t get_angles(void)
{
	pthread_mutex_lock(&angles_mtx);
	angles_t aux = angles;
	pthread_mutex_unlock(&angles_mtx);
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
