#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "bmp_180.h"


int main(int argc, char** argv)
{
	int fd = 0;
	BMP_180_Calibration cal;

	setup_bmp_180(&fd, &cal);

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

	//BMP_180_Calibration cal2 =
	//{
	//	.ac1 = 408,
	//	.ac2 = -72,
	//	.ac3 = -14383,
	//	.ac4 = 32741,
	//	.ac5 = 32757,
	//	.ac6 = 23153,
	//	.b1  = 6190,
	//	.b2  = 4,
	//	.mb  = -32768,
	//	.mc  = -8711,
	//	.md  = 2868,
	//};

	while(true)
	{
		float temp  = 0;
		float press = 0;

		for(size_t i = 0; i < 4; i ++)
		{
			read_bmp_180(&temp, &press, fd, &cal, i << 6);

			float altitude = bmp_180_altitude(press);

			printf("%lu, %f = degrees C, %f = pascals, %f = meters\n\n", i, temp, press, altitude);

            usleep(1E6);
		}
	}

  return 0;
  (void) argc;
  (void) argv;
}
