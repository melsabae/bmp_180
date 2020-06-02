#include <stdio.h>
#include "bmp_180.h"


int main(int argc, char** argv)
{
	int fd = 0;
	BMP_180_Calibration cal;

	//setup_bmp_180(&fd, &cal);

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

	BMP_180_Calibration cal2 =
	{
		.ac1 = 408,
		.ac2 = -72,
		.ac3 = -14383,
		.ac4 = 32741,
		.ac5 = 32757,
		.ac6 = 23153,
		.b1  = 6190,
		.b2  = 4,
		.mb  = -32768,
		.mc  = -8711,
		.md  = 2868,
	};

	debug_read_bmp_180(fd, &cal2, BMP_180_OSS_CONTROL_1);
	debug_read_bmp_180(fd, &cal2, BMP_180_OSS_CONTROL_2);
	debug_read_bmp_180(fd, &cal2, BMP_180_OSS_CONTROL_4);
	debug_read_bmp_180(fd, &cal2, BMP_180_OSS_CONTROL_8);

	//compute_true_temp_and_pressure(&true_temp, &true_press, 27898, 23843, 0, NULL);
	//printf("%f, %f\n", true_temp, true_press);

  return 0;
  (void) argc;
  (void) argv;
}
