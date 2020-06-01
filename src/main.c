#include <fcntl.h>
#include <assert.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include "bmp_180.h"


const int16_t  ac1 = 408;
const int16_t  ac2 = -72;
const int16_t  ac3 = -14383;
const uint16_t ac4 = 32741;
const uint16_t ac5 = 32757;
const uint16_t ac6 = 23153;
const int16_t  b1  = 6190;
const int16_t  b2  = 4;
const int16_t  mb  = -32768;
const int16_t  mc  = -8711;
const int16_t  md  = 2868;

void compute_true_temp_and_pressure(
			float* true_temp
		, float* true_pressure
		, const int32_t raw_temp
		, const int32_t raw_pressure
		, const int16_t oss
		, const uint8_t cal[22]
		)
{
	const float x11 = (raw_temp - ac6) * ac5 / (1 << 15);
  const float x21 = mc * (1 << 11) / (x11 + md);
  const float b5  = x11 + x21;
	const float b6  = b5 - 4000;
	const float x12 = (b2 * (b6 * b6 / (1 << 12))) / (1 << 11);
	const float x22 = ac2 * b6 / (1 << 11);
	const float x32 = x12 + x22;
	const float b3  = (((ac1 * 4 + x32) * (1 << oss)) + 2) / 4;
	const float x13 = ac3 * b6 / (1 << 13);
	const float x23 = (b1 * (b6 * b6 / (1 << 12))) / (1 << 16);
	const float x33 = ((x13 + x23) + 2) / 4;
	const float b4  = ac4 * (uint32_t)(x33 + 32768) / (1 << 15);
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


void test_i2c(BMP_180* bmp)
{
  const char* i2c_dev_file = "/dev/i2c-1";

  const int file = open(i2c_dev_file, O_RDWR);
  assert(file >= 0);

  int ret = ioctl(file, I2C_SLAVE, I2CDETECT_ADDRESS);
  assert(ret >= 0);

  for(size_t i = BMP_180_REGISTER_CALIB_00; i <= BMP_180_REGISTER_CALIB_21; i ++)
  {
    const uint8_t buf[1] = { i };

    ret = write(file, buf, 1);
    assert(ret == 1);

    const size_t offset = i - BMP_180_REGISTER_CALIB_00;
    uint8_t* storage = bmp->calibration_coefficients + offset;
    ret = read(file, (char*) storage, 1);
    assert(ret == 1);
  }

  close(file);
}

int main(int argc, char** argv)
{
  //BMP_180 bmp;
  //test_i2c(&bmp);

  //for (int i = 0; i < 22; i ++)
  //{
  //  printf("%.2X,", bmp.calibration_coefficients[i] & 0xff);
  //}
  //puts("");

	float true_temp;
	float true_press;

	compute_true_temp_and_pressure(&true_temp, &true_press, 27898, 23843, 0, NULL);
	printf("%f, %f\n", true_temp, true_press);

  return 0;
  (void) argc;
  (void) argv;
}
