#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include "bmp_180.h"


#include <inttypes.h>


typedef enum
{
    BMP_180_SCO_FINISHED_CONVERSION = 0b00000000
  , BMP_180_SCO_START_CONVERSION    = 0b00100000
} BMP_180SCO_Control;


typedef enum
{
    BMP_180_MEASUREMENT_CONTROL_TEMPERATURE    = 0x0E
  , BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_0 = 0x14 | BMP_180_OSS_CONTROL_1
  , BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_1 = 0x14 | BMP_180_OSS_CONTROL_2
  , BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_2 = 0x14 | BMP_180_OSS_CONTROL_4
  , BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_3 = 0x14 | BMP_180_OSS_CONTROL_8
} BMP_180_Measurement_Control;


typedef enum
{
    BMP_180_START_CONVERSION_TEMPERATURE    = BMP_180_MEASUREMENT_CONTROL_TEMPERATURE    | BMP_180_SCO_START_CONVERSION
  , BMP_180_START_CONVERSION_PRESSURE_OSS_0 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_0
  , BMP_180_START_CONVERSION_PRESSURE_OSS_1 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_1
  , BMP_180_START_CONVERSION_PRESSURE_OSS_2 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_2
  , BMP_180_START_CONVERSION_PRESSURE_OSS_3 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_3
} BMP_180_Start_Conversion;



typedef enum
{
    BMP_180_REGISTER_OUT_XLSB   = 0xF8
  , BMP_180_REGISTER_OUT_LSB    = 0XF7
  , BMP_180_REGISTER_OUT_MSB    = 0xF6
  , BMP_180_REGISTER_CTRL_MEAS  = 0xF4
  , BMP_180_REGISTER_SOFT_RESET = 0xE0
  , BMP_180_REGISTER_ID         = 0xD0
  , BMP_180_REGISTER_CALIB_00   = 0xAA
  , BMP_180_REGISTER_CALIB_01   = 0xAB
  , BMP_180_REGISTER_CALIB_02   = 0xAC
  , BMP_180_REGISTER_CALIB_03   = 0xAD
  , BMP_180_REGISTER_CALIB_04   = 0xAE
  , BMP_180_REGISTER_CALIB_05   = 0xAF
  , BMP_180_REGISTER_CALIB_06   = 0xB0
  , BMP_180_REGISTER_CALIB_07   = 0xB1
  , BMP_180_REGISTER_CALIB_08   = 0xB2
  , BMP_180_REGISTER_CALIB_09   = 0xB3
  , BMP_180_REGISTER_CALIB_10   = 0xB4
  , BMP_180_REGISTER_CALIB_11   = 0xB5
  , BMP_180_REGISTER_CALIB_12   = 0xB6
  , BMP_180_REGISTER_CALIB_13   = 0xB7
  , BMP_180_REGISTER_CALIB_14   = 0xB8
  , BMP_180_REGISTER_CALIB_15   = 0xB9
  , BMP_180_REGISTER_CALIB_16   = 0xBA
  , BMP_180_REGISTER_CALIB_17   = 0xBB
  , BMP_180_REGISTER_CALIB_18   = 0xBC
  , BMP_180_REGISTER_CALIB_19   = 0xBD
  , BMP_180_REGISTER_CALIB_20   = 0xBE
  , BMP_180_REGISTER_CALIB_21   = 0xBF
} BMP_180_Registers;


const int DATASHEET_ADDRESS          = 0xEE;
const int I2CDETECT_ADDRESS          = DATASHEET_ADDRESS >> 1;
const int BMP_180_CALIBRATION_NUMBER = 22;


static BMP_180_Start_Conversion convert_oss_to_conversion(const BMP_180_OSS_Control c)
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


static useconds_t convert_convesion_to_sleep_interval(const BMP_180_Start_Conversion s)
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


#include <stdio.h>
static int32_t raw_read_temperature(const int fd)
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


static int32_t raw_read_pressure(const int fd, const BMP_180_OSS_Control c)
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

	printf("%s:oss=%d,", __FUNCTION__, c >> 6);
  for(size_t i = 0; i < sizeof(read_buf); i ++)
  {
    printf("%.2X,", read_buf[i] & 0xFF);
  }
  printf("\n");

  const int32_t raw_pressure = (read_buf[0] << 16) | (read_buf[1] << 8) | read_buf[2];
  const int32_t oss_corrected_pressure = raw_pressure >> (8 - (c >> 6));
  return oss_corrected_pressure & ((1 << 19) - 1);
}


static BMP_180_Calibration compute_bmp_calibrations(const uint8_t array[BMP_180_CALIBRATION_NUMBER])
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


static int setup_bmp_180_fd(const char* device_path)
{
	const int file = open(device_path, O_RDWR);
	ioctl(file, I2C_SLAVE, I2CDETECT_ADDRESS);
	return file;
}


static void convert_raw_to_true(float* true_temperature, float* true_pressure, const int32_t ut, const int32_t up, const BMP_180_OSS_Control c, const BMP_180_Calibration* cal)
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
	uint8_t buffer[BMP_180_CALIBRATION_NUMBER];

	write(fd, write_buf, 1);
	read(fd, buffer, BMP_180_CALIBRATION_NUMBER);
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

