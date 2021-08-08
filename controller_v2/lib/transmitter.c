#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "transmitter.h"
#include <machine/patmos.h>
#include <machine/spm.h>

// 6 channels of transmitter
__int32_t throttle;
__int32_t roll;
__int32_t pitch;
__int32_t yaw;
__int32_t channel_5;
__int32_t channel_6;


// (roll,pitch,yaw,throttle)
int low[4]    = { 1000, 1000, 1000, 1090 };
int center[4] = { 1495, 1500, 1500, 1530 };
int high[4]   = { 2000, 2000, 2000, 2000 };

const unsigned int CPU_PERIOD = 20;

int receiver_read(unsigned int receiver_id)
{
  unsigned int clock_cycles_counted = *(RECEIVER + receiver_id);
  unsigned int pulse_high_time = (clock_cycles_counted * CPU_PERIOD) / 1000;

  return pulse_high_time;
}

int convert_receiver_channel(unsigned int channel, bool reverse)
{
  int actual, difference;

  actual = receiver_read(channel);

  if (actual < center[channel])
  {
    if (actual < low[channel])
      actual = low[channel];

    difference = ((long)(center[channel] - actual) * (long)500) / (center[channel] - low[channel]);

    if (reverse)
      return 1500 - difference;
    else
      return 1500 + difference;
  }
  else if (actual > center[channel])
  {
    if (actual > high[channel])
      actual = high[channel];

    difference = ((long)(actual - center[channel]) * (long)500) / (high[channel] - center[channel]);

    if (reverse)
      return 1500 + difference;
    else
      return 1500 - difference;
  }
  else
    return 1500;
}



void transmitter_read() {
  // read the receiver pwm duty cycle

  int throttle_tmp 	= convert_receiver_channel(0, true);
  int roll_tmp	 	= convert_receiver_channel(1, true);
  int pitch_tmp 	= convert_receiver_channel(2, false);
  int yaw_tmp 		= convert_receiver_channel(3, true);
  int channel_5_tmp = receiver_read(4);
  int channel_6_tmp = receiver_read(5);

  throttle 	= throttle_tmp;
  roll 		= roll_tmp;
  pitch 	= pitch_tmp;
  yaw 		= yaw_tmp;
  channel_5 = channel_5_tmp;
  channel_6 = channel_6_tmp;
}

transmitter get_transmitter_values()
{
  return (transmitter) { throttle, roll, pitch, yaw, channel_5, channel_6 };
}
