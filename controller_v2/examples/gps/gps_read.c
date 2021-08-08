#include "sensors/gps.h"
#include "lib/read_gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>

#define PROGNAME "simple-usage"
static pthread_mutex_t print_mtx = PTHREAD_MUTEX_INITIALIZER;

  void print_tpv_value(const char* name, const char* format, const int32_t value, const int32_t scale_factor)
{
  printf("%s: ", name);
  if (GPS_INVALID_VALUE != value)
  {
    printf(format, (double)value / scale_factor);
  }
  else
  {
    puts("INVALID");
  }
}

void* read()
{
  struct gps_tpv tpv;
  int result;
  char str[256];

  /* Sets the data to a known state */
  gps_init_tpv(&tpv);

  while (1)
  {
    /* Attempt to decode the user supplied string */
    result = gps_decode(&tpv, str);
    if (result != GPS_OK)
    {
      fprintf(stderr, "Error (%d): %s\n", result, gps_error_string(result));
      break;
    }

    /* Go through each TPV value and show what information was decoded */
    printf("Talker ID: %s\n", tpv.talker_id);
    printf("Time Stamp: %s\n", tpv.time);
    print_tpv_value("Latitude", "%.6f\n", tpv.latitude, GPS_LAT_LON_FACTOR);
    print_tpv_value("Longitude", "%.6f\n", tpv.longitude, GPS_LAT_LON_FACTOR);
    print_tpv_value("Altitude", "%.3f\n", tpv.altitude, GPS_VALUE_FACTOR);
    print_tpv_value("Track", "%.3f\n", tpv.track, GPS_VALUE_FACTOR);
    print_tpv_value("Speed", "%.3f\n", tpv.speed, GPS_VALUE_FACTOR);

    printf("Mode: ");
    switch (tpv.mode)
    {
    case GPS_MODE_UNKNOWN:
      puts("Unknown");
      break;
    case GPS_MODE_NO_FIX:
      puts("No fix");
      break;
    case GPS_MODE_2D_FIX:
      puts("2D");
      break;
    case GPS_MODE_3D_FIX:
      puts("3D");
      break;
    default:
      break;
    }

    printf("\n");
  }

}

static void* readgps(void* args)
{
  struct gps_tpv GPS;
  gps_init_tpv(&GPS);
  bool gps_output = false;
  while (1)
  {

    pthread_mutex_lock(&print_mtx);
    printf("thread2\n");
    gps_output = read_gps(&GPS);
    if (gps_output)
    {
      printf("RESULT long = %ld, lat = %ld, mode = %u\n", GPS.longitude, GPS.latitude, GPS.mode);
    }
    else
    {
      printf("GPS data not valid\n");
    }

    pthread_mutex_unlock(&print_mtx);
  }
}

static void* thread1_main(void* args)
{
  while (1);
}

int main(void)
{
  // Init threads
  pthread_t thread1, thread2, thread3;
  pthread_create(&thread1, NULL, thread1_main, NULL);
  pthread_create(&thread2, NULL, thread1_main, NULL);
  pthread_create(&thread3, NULL, readgps, NULL);

  while (true)
  {
    pthread_mutex_lock(&print_mtx);
    printf("thread1\n");
    pthread_mutex_unlock(&print_mtx);
  }

  return 0;
}
