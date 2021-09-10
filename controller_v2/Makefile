all:
	rm -rf drone.elf
	patmos-clang -O2 -o drone.elf main.c thread_0.c thread_1.c thread_2.c \
		thread_3.c sensors/mpu6050.c lib/angles.c lib/i2c_master.c lib/leds.c \
		lib/pid.c lib/timer.c lib/uart.c lib/motor.c lib/transmitter.c util.c \
		lib/communication.c -I . -lm
