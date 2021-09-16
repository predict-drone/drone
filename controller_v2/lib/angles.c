#include <stdint.h>
#include <stdio.h>
#include <machine/fastmath.h>
#include <math.h>

#include "angles.h"
#include "main.h"
#include "lib/timer.h"
#include "util.h"

#define SETUP_MEASUREMENTS 500
#define ALPHA (fixedpt_rconst(0.8))


/** Data structures --------------------------------------------------------- */
typedef struct angles_accel_axis_t {
	double x; /** Accel x position */
	double y; /** Accel x position */
	double z; /** Accel x position */
} angles_accel_axis_t;

typedef struct angles_gyro_axis_t {
	fixedpt x;   /** Gyro x axis position */
	fixedpt y;   /** Gyro y axis position */
	fixedpt z;   /** Gyro z axis position */
	fixedpt x_s; /** Gyro x axis speed */
	fixedpt y_s; /** Gyro y axis speed */
	fixedpt z_s; /** Gyro z axis speed */
} angles_gyro_axis_t;

/** Static variables -------------------------------------------------------- */
static bool init; /** Whether the module has been initialized */
static pthread_mutex_t init_mtx = PTHREAD_MUTEX_INITIALIZER;
//static double pitch_avg = 0;
//static double roll_avg = 0;
//static double yaw_avg = 0;
//static double gyro_x_avg = 0;
//static double gyro_y_avg = 0;
//static double gyro_z_avg = 0;

static angles_gyro_axis_t gyro_err;  /** Gyroscope initial error */
static angles_accel_axis_t accel_err; /** Accelerometer initial error */

static angles_t angles; /** Angles position and speed */
static pthread_mutex_t angles_mtx = PTHREAD_MUTEX_INITIALIZER;
static uint64_t gyro_last_time_us; /** Gyro last measurement time */

/** Function prototypes ----------------------------------------------------- */
static angles_gyro_axis_t calculate_angles_gyro(void);
static angles_accel_axis_t calculate_angles_accel(void);
static inline fixedpt combine(fixedpt pos, fixedpt gyro, double accel);

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
	double x = accel_data.x;
	double y = accel_data.y;
	double z = accel_data.z;
	angles_accel_axis_t accel;
	//Converts the accelerometer data to angle position [degrees].
	accel.x = -(atan(x / (sqrt(pow(y, 2) + pow(z, 2)))) - accel_err.x)*180/M_PI;
	accel.y = (atan(y / (sqrt(pow(x, 2) + pow(z, 2)))) - accel_err.y)*180/M_PI;
	accel.z = (atan(z / (sqrt(pow(y, 2) + pow(x, 2)))) - accel_err.z)*180/M_PI;
	return accel;
}

static inline fixedpt combine(fixedpt pos, fixedpt gyro, double accel)
{
	return fixedpt_mul(fixedpt_rconst(0.95), (pos + gyro))
		+ fixedpt_mul(fixedpt_rconst(0.05), fixedpt_rconst(accel));
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
				fixedpt_rconst(32.8)), fixedpt_rconst(SETUP_MEASUREMENTS));
	gyro_err.y_s = fixedpt_div(fixedpt_div(fixedpt_fromint(gyro_y),
				fixedpt_rconst(32.8)), fixedpt_rconst(SETUP_MEASUREMENTS));
	gyro_err.z_s = fixedpt_div(fixedpt_div(fixedpt_fromint(gyro_z),
				fixedpt_rconst(32.8)), fixedpt_rconst(SETUP_MEASUREMENTS));

	double x = accel_x / (double) SETUP_MEASUREMENTS;
	double y = accel_y / (double) SETUP_MEASUREMENTS;
	double z = accel_z / (double) SETUP_MEASUREMENTS;
	accel_err.x = atan(x / (sqrt(pow(y, 2) + pow(z, 2))));
	accel_err.y = atan(y / (sqrt(pow(x, 2) + pow(z, 2))));
	accel_err.z = atan(z / (sqrt(pow(y, 2) + pow(x, 2))));

	pthread_mutex_lock(&print_mtx);
	printf("Gyro err: %.2f, %.2f, %.2f\n", fixedpt_tofloat(gyro_err.y_s),
			fixedpt_tofloat(gyro_err.x_s), fixedpt_tofloat(gyro_err.z_s));
	printf("Accel err: %.2f, %.2f, %.2f\n", accel_err.z, accel_err.y, accel_err.z);
	pthread_mutex_unlock(&print_mtx);

	pthread_mutex_lock(&init_mtx);
	init = true;
	pthread_mutex_unlock(&init_mtx);

	gyro_last_time_us = micros();
}

void calculate_angles(void) {
	//uint64_t ti = micros();
	angles_gyro_axis_t gyro = calculate_angles_gyro();
	//uint64_t tg = micros();
	angles_accel_axis_t accel = calculate_angles_accel();
	//uint64_t ta = micros();

	//Complementary filtering: Mixing acc and gyro data to minimise both noise
	// and drift. [degrees]
	//
	fixedpt pitch = combine(angles.pitch, gyro.y, accel.x);
	fixedpt roll = combine(angles.roll, gyro.x, accel.y);
	uint64_t tc = micros();

	pthread_mutex_lock(&angles_mtx);
	angles.pitch = pitch;
	angles.roll = roll;
	angles.yaw += gyro.z;
	angles.pitch_speed += gyro.y_s;
	angles.roll_speed += gyro.x_s;
	angles.yaw_speed += gyro.z_s;

	// Yaw limit
	if (angles.yaw > fixedpt_fromint(180)) angles.yaw -= fixedpt_fromint(360);
	if (angles.yaw < fixedpt_fromint(-180)) angles.yaw += fixedpt_fromint(360);

	pthread_mutex_unlock(&angles_mtx);

	//uint64_t te = micros();
	//printf("T: %lld %lld %lld %lld\n", tg-ti, ta-tg, tc-ta, te-tc);
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
