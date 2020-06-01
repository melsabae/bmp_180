#include <stdio.h>
#include "bmp_180.h"


void compute_true_temp_and_pressure(
			float* true_temp
		, float* true_pressure
		, const int32_t raw_temp
		, const int32_t raw_pressure
		, const int16_t oss
		, const BMP_180_Calibration* cal
		)
{
	const float x11 = (raw_temp - cal->ac6) * cal->ac5 / (1 << 15);
  const float x21 = cal->mc * (1 << 11) / (x11 + cal->md);
  const float b5  = x11 + x21;
	const float b6  = b5 - 4000;
	const float x12 = (cal->b2 * (b6 * b6 / (1 << 12))) / (1 << 11);
	const float x22 = cal->ac2 * b6 / (1 << 11);
	const float x32 = x12 + x22;
	const float b3  = (((cal->ac1 * 4 + x32) * (1 << oss)) + 2) / 4;
	const float x13 = cal->ac3 * b6 / (1 << 13);
	const float x23 = (cal->b1 * (b6 * b6 / (1 << 12))) / (1 << 16);
	const float x33 = ((x13 + x23) + 2) / 4;
	const float b4  = cal->ac4 * (uint32_t)(x33 + 32768) / (1 << 15);
	const float b7  = ((uint32_t) raw_pressure - b3) * (50000 >> oss);
	const float p   = (b7 < 0x80000000)
										 ? (b7 * 2) / b4
										 : (b7 / b4) * 2
										 ;
	const float x14 = (p / (1 << 8) * (p / (1 << 8))) * 3038 / (1 << 16);
	const float x24 = (-7357 * p) / (1 << 16);

	*true_temp = (b5 + 8) / 15 / 10;
	*true_pressure = p + (x14 + x24 + 3791) / (1 << 4);
	(void) cal;
}


int main(int argc, char** argv)
{
	int fd;
	BMP_180_Calibration cal;

	setup_bmp_180(&fd, &cal);

	printf(
			  "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"
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

	//compute_true_temp_and_pressure(&true_temp, &true_press, 27898, 23843, 0, NULL);
	//printf("%f, %f\n", true_temp, true_press);

  return 0;
  (void) argc;
  (void) argv;
}
