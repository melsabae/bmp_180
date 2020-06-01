#include <fcntl.h>
#include <assert.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdio.h>
#include "bmp_180.h"

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
  BMP_180 bmp;
  test_i2c(&bmp);

  for (int i = 0; i < 22; i ++)
  {
    printf("%.2X,", bmp.calibration_coefficients[i] & 0xff);
  }
  puts("");

  return 0;
  (void) argc;
  (void) argv;
}
