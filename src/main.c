#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "bmp_180.h"


int main(int argc, char** argv)
{
	int fd = 0;
	BMP_180_Calibration cal;

	setup_bmp_180(&fd, &cal, "/dev/i2c-1");

	printf(
			  "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
			, cal.ac1
			, cal.ac2
			, cal.ac3
			, cal.ac4
			, cal.ac5
			, cal.ac6
			, cal.b1
			, cal.b2
			, cal.mb
			, cal.mc
			, cal.md
			);

	while(true)
	{

		for(size_t i = 0; i < 4; i ++)
		{
			float temp     = 0.0f;
			float press    = 0.0f;
			float altitude = 0.0f;
			read_bmp_180_all(&temp, &press, &altitude, fd, &cal, i << 6);
			printf("%lu, %.1f C, %.0f Pa, %f m\n", i, temp, press, altitude);
		}

		usleep(1E6 / 5);
	}

  return 0;
  (void) argc;
  (void) argv;
}
