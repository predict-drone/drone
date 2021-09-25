#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "lib/angles.h"
#include "lib/uart.h"
#include "lib/timer.h"
#include "lib/communication.h"

/** Function prototypes ----------------------------------------------------- */
static void rx_callback(uint8_t msg_len, uint8_t* msg);

/** Function definitions ---------------------------------------------------- */
static void rx_callback(uint8_t msg_len, uint8_t* msg) {
	switch (msg[0]) { // opcode
	case COMM_OP_GET_PID_VALUES:
		{
			pid_instance_t* pid_ins;
			if (msg[1] == COMM_PID_PITCH) pid_ins = &pitch_pid;
			else if (msg[1] == COMM_PID_ROLL) pid_ins = &roll_pid;
			else if (msg[1] == COMM_PID_YAW) pid_ins = &yaw_pid;
			else if (msg[1] == COMM_PID_PITCH_VEL) pid_ins = &pitch_vel_pid;
			else if (msg[1] == COMM_PID_ROLL_VEL) pid_ins = &roll_vel_pid;
			else if (msg[1] == COMM_PID_YAW_VEL) pid_ins = &yaw_vel_pid;

			comm_pid_values_t pid_values;
			pid_values.opcode = COMM_OP_RSP_GET_PID_VALUES;
			pid_values.pid_id = msg[1];
			pthread_mutex_lock(&pid_mtx);
			pid_values.kp = pid_ins->kp;
			pid_values.ki = pid_ins->ki;
			pid_values.kd = pid_ins->kd;
			pthread_mutex_unlock(&pid_mtx);
			comm_send_msg(sizeof(pid_values), (uint8_t*)&pid_values);
			break;
		}
	case COMM_OP_SET_PID_VALUES:
		{
			pid_instance_t* pid_ins;
			if (msg[1] == COMM_PID_PITCH) pid_ins = &pitch_pid;
			else if (msg[1] == COMM_PID_ROLL) pid_ins = &roll_pid;
			else if (msg[1] == COMM_PID_YAW) pid_ins = &yaw_pid;
			else if (msg[1] == COMM_PID_PITCH_VEL) pid_ins = &pitch_vel_pid;
			else if (msg[1] == COMM_PID_ROLL_VEL) pid_ins = &roll_vel_pid;
			else if (msg[1] == COMM_PID_YAW_VEL) pid_ins = &yaw_vel_pid;

			comm_pid_values_t* pid_values = (comm_pid_values_t*)msg;
			pthread_mutex_lock(&pid_mtx);
			pid_ins->kp = pid_values->kp;
			pid_ins->ki = pid_values->ki;
			pid_ins->kd = pid_values->kd;
			pthread_mutex_unlock(&pid_mtx);
			uint8_t rsp[] = {COMM_OP_RSP_SET_PID_VALUES, msg[1]};
			comm_send_msg(sizeof(rsp), rsp);
			break;
		}
	default:
		break;
	}
}

/** Public functions -------------------------------------------------------- */
void* thread_3_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 3\n");
	pthread_mutex_unlock(&print_mtx);

	while (!angles_is_init());
	
	comm_init(rx_callback);

	uint64_t next_time = micros();
	while (true) {
		comm_process();
		if (false) {
			angles_t angles = get_angles();
			comm_data_t data;
			data.opcode = COMM_OP_LOG_ANGLES;
			data.x = angles.pitch;
			data.y = angles.roll;
			data.z = angles.yaw;
			pthread_mutex_lock(&motor_mtx);
			data.m0 = motor_0;
			data.m1 = motor_1;
			data.m2 = motor_2;
			data.m3 = motor_3;
			pthread_mutex_unlock(&motor_mtx);

			comm_send_msg(sizeof(data), (uint8_t*)&data);

			next_time += 100000;
		}
	}
}
