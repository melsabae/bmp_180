#include <linux/i2c-dev.h>
#include <math.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>


#include "bmp_180.h"


int setup_bmp_180_fd(
		  int* fd
		, const char* device_path
		)
{
  const int _fd = open(device_path, O_RDWR);

	if(_fd <= STDERR_FILENO) { return 1; }

  if(0 != flock(_fd, LOCK_EX))
	{
		if(0 != close(_fd)) { return 2; }
		return 3;
	}

  if(0 != ioctl(_fd, I2C_SLAVE, I2CDETECT_ADDRESS))
	{
		if(0 != close(_fd)) { return 4; }
		return 5;
	}

  if(0 != flock(_fd, LOCK_UN))
	{
		if(0 != close(_fd)) { return 6; }
		return 7;
	}

	*fd = _fd;
	return 0;
}


int raw_bmp_180_write(
			const int fd
		, const uint8_t* data
		, const size_t len
		)
{
  if(0 != flock(fd, LOCK_EX)) { return 1; }
  if(((ssize_t) len) != write(fd, data, len)) { return 2; }
	if(0 != flock(fd, LOCK_UN)) { return 3; }
	return 0;
}


int raw_bmp_180_read(
			uint8_t* data
		, const int fd
		, const size_t len
		)
{
  if(0 != flock(fd, LOCK_EX)) { return 1; }
  if(((ssize_t) len) != read(fd, data, len)) { return 2; }
	if(0 != flock(fd, LOCK_UN)) { return 3; }
	return 0;
}


BMP_180_Calibration compute_bmp_calibrations(
		const uint8_t array[BMP_180_CALIBRATION_BYTES]
		)
{
  return (BMP_180_Calibration)
  {
      .ac1 = array[ 0] << 8 | array[ 1]
    , .ac2 = array[ 2] << 8 | array[ 3]
    , .ac3 = array[ 4] << 8 | array[ 5]
    , .ac4 = array[ 6] << 8 | array[ 7]
    , .ac5 = array[ 8] << 8 | array[ 9]
    , .ac6 = array[10] << 8 | array[11]
    , .b1  = array[12] << 8 | array[13]
    , .b2  = array[14] << 8 | array[15]
    , .mb  = array[16] << 8 | array[17]
    , .mc  = array[18] << 8 | array[19]
    , .md  = array[20] << 8 | array[21]
  };
}


int get_bmp_calibration(
		  BMP_180_Calibration* cal
		, const int fd
		)
{
  const uint8_t write_buf[1] = { BMP_180_REGISTER_CALIB_00 };
  uint8_t read_buf[BMP_180_CALIBRATION_BYTES];

  if(0 != raw_bmp_180_write(fd, write_buf, 1)) { return 1; }
  if(0 != raw_bmp_180_read(read_buf, fd, BMP_180_CALIBRATION_BYTES)) { return 2; }

  *cal = compute_bmp_calibrations(read_buf);
	return 0;
}


int setup_bmp_180(
			int* fd
		, BMP_180_Calibration* cal
		, const char* file_path
		)
{
  if(0 != setup_bmp_180_fd(fd, file_path)) { return 1; }
  return get_bmp_calibration(cal, *fd);
}


int read_uncompensated_temperature(
		  int32_t* ut
		, const int fd
		)
{
  const BMP_180_Start_Conversion s = BMP_180_START_CONVERSION_TEMPERATURE;
  const useconds_t sleep_interval = convert_convesion_to_sleep_interval(s);
  uint8_t write_buf[2] = { (uint8_t) BMP_180_REGISTER_CTRL_MEAS, (uint8_t) s };
  uint8_t read_buf[2] = { 0 };

  if(0 != raw_bmp_180_write(fd, write_buf, sizeof(write_buf))) { return 1; }
  if(0 != usleep(sleep_interval)) { return 2; }

  write_buf[0] = BMP_180_REGISTER_OUT_MSB;

  if(0 != raw_bmp_180_write(fd, write_buf, 1)) { return 3; }
  if(0 != raw_bmp_180_read(read_buf, fd, sizeof(read_buf))) { return 4; }

  *ut = ((read_buf[0] << 8) | read_buf[1]) & ((1 << 16) - 1);
	return 0;
}


int read_uncompensated_pressure(
		  int32_t* up
		, const int fd
		, const BMP_180_OSS_Control c
		)
{
  const BMP_180_Start_Conversion s = convert_oss_to_conversion(c);
  const useconds_t sleep_interval = convert_convesion_to_sleep_interval(s);
  uint8_t write_buf[2] = { (uint8_t) BMP_180_REGISTER_CTRL_MEAS, (uint8_t) s };
  uint8_t read_buf[3] = { 0 };

  if(0 != raw_bmp_180_write(fd, write_buf, sizeof(write_buf))) { return 1; }
  if(0 != usleep(sleep_interval)) { return 2; }

  write_buf[0] = BMP_180_REGISTER_OUT_MSB;

  if(0 != raw_bmp_180_write(fd, write_buf, 1)) { return 3; }
  if(0 != raw_bmp_180_read(read_buf, fd, sizeof(read_buf))) { return 4; }

  const int32_t raw_pressure = ((read_buf[0] << 16) | (read_buf[1] << 8) | (read_buf[2]));
  *up = raw_pressure >> (8 - (((size_t) c) >> 6));
	return 0;
}


int read_bmp_180(
			float* true_temperature
		, float* true_pressure
		, const int fd
		, const BMP_180_Calibration* cal
		, const BMP_180_OSS_Control c
		)
{
	int32_t ut = 0;
	int32_t up = 0;

  if(0 != read_uncompensated_temperature(&ut, fd)) { return 1; }
  if(0 != read_uncompensated_pressure(&up, fd, c)) { return 2; }

  if(0 != convert_uncompensated_to_true(true_temperature, true_pressure, ut, up, c, cal))
	{
		return 3;
	}

	return 0;
}


int read_bmp_180_all(
			float* true_temperature_celcius
		, float* true_pressure_pascals
		, float* altitude_meters
    , const float ref_pressure_pascals
		, const int fd
		, const BMP_180_Calibration* cal
		, const BMP_180_OSS_Control c
		)
{
  if(0 != read_bmp_180(true_temperature_celcius, true_pressure_pascals, fd, cal, c))
	{
		return 1;
	}

  *altitude_meters = bmp_180_altitude_from_ref(*true_pressure_pascals, ref_pressure_pascals);
	return 0;
}


BMP_180_Start_Conversion convert_oss_to_conversion(
		const BMP_180_OSS_Control c
	)
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


useconds_t convert_convesion_to_sleep_interval(
		const BMP_180_Start_Conversion s
		)
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


float bmp_180_altitude_from_ref(
			const float true_pressure_pascals
    , const float ref_pressure_pascals
		)
{
  return 44330.0 * (1.0f - pow(true_pressure_pascals / ref_pressure_pascals, 1.0f/5.255f));
}


int convert_uncompensated_to_true(
			float* true_temperature
		, float* true_pressure
		, const int32_t ut
		, const int32_t up
		, const BMP_180_OSS_Control c
		, const BMP_180_Calibration* cal
		)
{
  const uint32_t oss = ((uint32_t) c) >> 6;
  const  int32_t x11 = (ut - ((int32_t)cal->ac6)) * ((int32_t) cal->ac5) / (1 << 15);

	const int32_t guard = x11 + cal->md;

	if(0 == guard) { return 1; }

  const  int32_t x21 = ((int32_t) cal->mc) * (1 << 11) / guard;
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

	if(0 == b4) { return 2; }

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

	return 0;
}


int convert_uncompensated_temperature_to_true(
			float* true_temperature
	  , const int32_t ut
		, const BMP_180_OSS_Control c
		, const BMP_180_Calibration* cal
		)
{
	const int32_t dummy_up = 0;

	float dummy_pressure = 0.0f;
	float t = 0.0f;

	if(0 != convert_uncompensated_to_true(&t, &dummy_pressure, ut, dummy_up, c, cal)) { return 1; }

	*true_temperature = t;
	return 0;
}

