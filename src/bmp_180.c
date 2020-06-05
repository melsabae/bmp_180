#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>

#include "bmp_180.h"


BMP_180_Start_Conversion convert_oss_to_conversion(const BMP_180_OSS_Control c)
{
  switch(c)
  {
    case BMP_180_OSS_CONTROL_1: return BMP_180_START_CONVERSION_PRESSURE_OSS_0;
    case BMP_180_OSS_CONTROL_2: return BMP_180_START_CONVERSION_PRESSURE_OSS_1;
    case BMP_180_OSS_CONTROL_4: return BMP_180_START_CONVERSION_PRESSURE_OSS_2;
    case BMP_180_OSS_CONTROL_8: return BMP_180_START_CONVERSION_PRESSURE_OSS_3;
  }

  return BMP_180_START_CONVERSION_PRESSURE_OSS_3;
}


useconds_t convert_convesion_to_sleep_interval(const BMP_180_Start_Conversion s)
{
  switch(s)
  {
    case BMP_180_START_CONVERSION_TEMPERATURE   : return 4500;  // 4.5 ms
    case BMP_180_START_CONVERSION_PRESSURE_OSS_0: return 4500;  // 4.5 ms
    case BMP_180_START_CONVERSION_PRESSURE_OSS_1: return 7500;  // 7.5 ms
    case BMP_180_START_CONVERSION_PRESSURE_OSS_2: return 13500; // 13.5 ms
    case BMP_180_START_CONVERSION_PRESSURE_OSS_3: return 25500; // 25.5 ms
  }

  return 76500;
}


int32_t raw_read_temperature(const int fd)
{
  const BMP_180_Start_Conversion s = BMP_180_START_CONVERSION_TEMPERATURE;
  const useconds_t sleep_interval = convert_convesion_to_sleep_interval(s);
  uint8_t write_buf[2] = { (uint8_t) BMP_180_REGISTER_CTRL_MEAS, (uint8_t) s };
  uint8_t read_buf[2] = { 0 };

  write(fd, write_buf, sizeof(write_buf));
  usleep(sleep_interval); // wish the sensor had an interrupt pin

  write_buf[0] = BMP_180_REGISTER_OUT_MSB;
  write(fd, write_buf, 1);
  read(fd, read_buf, sizeof(read_buf));

  return ((read_buf[0] << 8) | read_buf[1]) & ((1 << 16) - 1);
}


int32_t raw_read_pressure(const int fd, const BMP_180_OSS_Control c)
{
  const BMP_180_Start_Conversion s = convert_oss_to_conversion(c);
  const useconds_t sleep_interval = convert_convesion_to_sleep_interval(s);
  uint8_t write_buf[2] = { (uint8_t) BMP_180_REGISTER_CTRL_MEAS, (uint8_t) s };
  uint8_t read_buf[3] = { 0 };

  write(fd, write_buf, sizeof(write_buf));
  usleep(sleep_interval); // wish the sensor had an interrupt pin

  write_buf[0] = BMP_180_REGISTER_OUT_MSB;
  write(fd, write_buf, 1);
  read(fd, read_buf, sizeof(read_buf));

  const int32_t raw_pressure = ((read_buf[0] << 16) | (read_buf[1] << 8) | (read_buf[2]));
  return raw_pressure >> (8 - (c >> 6));
}


BMP_180_Calibration compute_bmp_calibrations(const uint8_t array[BMP_180_CALIBRATION_BYTES])
{
  return (BMP_180_Calibration)
  {
    .ac1 = array[ 0] << 8 | array[ 1],
    .ac2 = array[ 2] << 8 | array[ 3],
    .ac3 = array[ 4] << 8 | array[ 5],
    .ac4 = array[ 6] << 8 | array[ 7],
    .ac5 = array[ 8] << 8 | array[ 9],
    .ac6 = array[10] << 8 | array[11],
    .b1  = array[12] << 8 | array[13],
    .b2  = array[14] << 8 | array[15],
    .mb  = array[16] << 8 | array[17],
    .mc  = array[18] << 8 | array[19],
    .md  = array[20] << 8 | array[21],
  };
}


int setup_bmp_180_fd(const char* device_path)
{
  const int file = open(device_path, O_RDWR);
  ioctl(file, I2C_SLAVE, I2CDETECT_ADDRESS);
  return file;
}


void convert_raw_to_true(float* true_temperature, float* true_pressure, const int32_t ut, const int32_t up, const BMP_180_OSS_Control c, const BMP_180_Calibration* cal)
{
  const uint32_t oss = ((uint32_t) c) >> 6;
  const  int32_t x11 = (ut - ((int32_t)cal->ac6)) * ((int32_t) cal->ac5) / (1 << 15);
  const  int32_t x21 = ((int32_t) cal->mc) * (1 << 11) / (x11 + cal->md);
  const  int32_t b5  = x11 + x21;
  const  int32_t b6  = b5 - 4000;
  const  int32_t x12 = cal->b2 * b6 * b6 / (1 << 23);
  const  int32_t x22 = cal->ac2 * b6 / (1 << 11);
  const  int32_t x32 = x12 + x22;
  const  int32_t b3  = (((cal->ac1 * 4 + x32) << oss) + 2) / 4;
  const  int32_t x13 = cal->ac3 * b6 / (1 << 13);
  const  int32_t x23 = cal->b1 * b6 * b6 / (1 << 28);
  const  int32_t x33 = (x13 + x23 + 2) / (1 << 2);
  const uint32_t b4  = cal->ac4 * (uint32_t)(x33 + 32768) / (1 << 15);
  const uint32_t b7  = ((uint32_t) up - b3) * (50000 >> oss);
  const  int32_t p   = b7 < 0x80000000
                     ? b7 * 2 / b4
                     : b7 / b4 * 2
                     ;
  const  int32_t x14 = p * p / (1 << 16);
  const  int32_t x15 = x14 * 3038 / (1 << 16);
  const  int32_t x24 = (-7357) * p / (1 << 16);

  *true_temperature  = ((float) ((b5 + 8) / (1 << 4))) / 10.0f;
  *true_pressure     = p + (x15 + x24 + 3791) / (1 << 4);
}


BMP_180_Calibration get_bmp_calibration(const int fd)
{
  const uint8_t write_buf[1] = { BMP_180_REGISTER_CALIB_00 };
  uint8_t buffer[BMP_180_CALIBRATION_BYTES];

  write(fd, write_buf, 1);
  read(fd, buffer, BMP_180_CALIBRATION_BYTES);
  return compute_bmp_calibrations(buffer);
}


void setup_bmp_180(int* fd, BMP_180_Calibration* cal, const char* file_path)
{
  *fd  = setup_bmp_180_fd(file_path);
  *cal = get_bmp_calibration(*fd);
}


void read_bmp_180(float* true_temperature, float* true_pressure, const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c)
{
  const int32_t ut = raw_read_temperature(fd);
  const int32_t up = raw_read_pressure(fd, c);
  convert_raw_to_true(true_temperature, true_pressure, ut, up, c, cal);
}


float bmp_180_altitude(const float true_pressure)
{
  return 44330.0 * (1.0f - pow(true_pressure / 101325.0f, 1.0f/5.255f));
}


void read_bmp_180_all(float* true_temperature, float* true_pressure, float* altitude, const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c)
{
  read_bmp_180(true_temperature, true_pressure, fd, cal, c);
  *altitude = bmp_180_altitude(*true_pressure);
}

