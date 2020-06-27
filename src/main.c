#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>


#include "bmp_180.h"
#include "main.h"


float make_reference_average_pressure(
      const int fd
    , const BMP_180_Calibration* cal
    , const BMP_180_OSS_Control c
    , const size_t number_samples
    )
{
  float temp     = 0.0f;
  float press    = 0.0f;
  float ret      = 0.0f;

  for(size_t i = 0; i < number_samples; i ++)
  {
    if(0 != read_bmp_180(&temp, &press, fd, cal, c))
    {
      return SEA_LEVEL_PRESSURE_PASCALS;
    }

    ret += press;
  }

  return ret / (float) number_samples;
}


int main(int argc, char** argv)
{
  const char* file_path = "/dev/i2c-1";

  int fd = 0;
  BMP_180_Calibration cal;

  const int return_code = setup_bmp_180(&fd, &cal, file_path);

  if(0 != return_code)
  {
    fprintf(
          stderr
        , "failed to open %s, return code %d\n"
        , file_path
        , return_code
        );

    return 1;
  }

  //printf(
  //      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
  //    , cal.ac1
  //    , cal.ac2
  //    , cal.ac3
  //    , cal.ac4
  //    , cal.ac5
  //    , cal.ac6
  //    , cal.b1
  //    , cal.b2
  //    , cal.mb
  //    , cal.mc
  //    , cal.md
  //    );

  const float ref_pressure =
    make_reference_average_pressure(fd, &cal, BMP_180_OSS_CONTROL_8, 32);

  while(true)
  {
    for(size_t i = 0; i < 4; i ++)
    {
      float temp_c         = 0.0f;
      float press_Pa       = 0.0f;

      const int _ = read_bmp_180(&temp_c, &press_Pa, fd, &cal, (i << 6) & 0x04);

      if(0 != _)
      {
        fprintf(stderr, "failed to read sensor, return code %d\n", _);
        fflush(stderr);
        continue;
      }

      const float altitude_abs   =
        bmp_180_altitude_from_ref(press_Pa, SEA_LEVEL_PRESSURE_PASCALS);

      const float altitude_delta =
        bmp_180_altitude_from_ref(press_Pa, ref_pressure);

      printf(
            "%zu, %.1f C, %.0f Pa, %f m, %f m\n"
          , i
          , temp_c
          , press_Pa
          , altitude_delta
          , altitude_abs
          );
    }

    usleep((int) 1E6 / 4); // 4 Hz
  }

  return close(fd);
  (void) argc;
  (void) argv;
}
