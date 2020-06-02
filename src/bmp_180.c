#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
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
  , BMP_180_START_CONVERSION_PRESSURE_OSS_0 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_0 | BMP_180_SCO_START_CONVERSION
  , BMP_180_START_CONVERSION_PRESSURE_OSS_1 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_1 | BMP_180_SCO_START_CONVERSION
  , BMP_180_START_CONVERSION_PRESSURE_OSS_2 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_2 | BMP_180_SCO_START_CONVERSION
  , BMP_180_START_CONVERSION_PRESSURE_OSS_3 = BMP_180_MEASUREMENT_CONTROL_PRESSURE_OSS_3 | BMP_180_SCO_START_CONVERSION
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


static BMP_180_OSS_Control convert_conversion_to_oss(const BMP_180_Start_Conversion s)
{
	switch(s)
	{
		case BMP_180_START_CONVERSION_PRESSURE_OSS_0: return BMP_180_OSS_CONTROL_1;
		case BMP_180_START_CONVERSION_PRESSURE_OSS_1: return BMP_180_OSS_CONTROL_2;
		case BMP_180_START_CONVERSION_PRESSURE_OSS_2: return BMP_180_OSS_CONTROL_4;
		case BMP_180_START_CONVERSION_PRESSURE_OSS_3: return BMP_180_OSS_CONTROL_8;

		case BMP_180_START_CONVERSION_TEMPERATURE:    break;
	}

	return BMP_180_OSS_CONTROL_8;
}


static useconds_t convert_oss_to_sleep_inverval(const BMP_180_OSS_Control c)
{
	switch(c)
	{
		case BMP_180_OSS_CONTROL_1: return 4500;  // 4.5 ms
		case BMP_180_OSS_CONTROL_2: return 7500;  // 7.5 ms
		case BMP_180_OSS_CONTROL_4: return 13500; // 13.5 ms
		case BMP_180_OSS_CONTROL_8: return 25500; // 25.5 ms
	}

	return 76500; // try not sucking at meeting basic interface requirements, take the max
}


static void raw_read_bmp_180(uint8_t bytes[3], const int fd, const BMP_180_Start_Conversion s)
{
	const BMP_180_OSS_Control oss = convert_conversion_to_oss(s);
	const useconds_t sleep_interval = convert_oss_to_sleep_inverval(oss);

	const size_t read_size = BMP_180_START_CONVERSION_TEMPERATURE == s
		? 2
		: 3
		;

	const uint8_t start_address = BMP_180_START_CONVERSION_TEMPERATURE == s
		? BMP_180_REGISTER_OUT_LSB
		: BMP_180_REGISTER_OUT_XLSB
		;

	uint8_t write_buf[1] = { (uint8_t) s };
	write(fd, write_buf, 1);

	// sensor does not have an interrupt pin
	usleep(sleep_interval);

	write_buf[0] = start_address;
	write(fd, write_buf, 1);

	uint8_t* p = BMP_180_START_CONVERSION_TEMPERATURE == s
		? bytes + 1
		: bytes
		;

	bytes[0] = 0;
	read(fd, p, read_size);
}


static BMP_180_Calibration compute_bmp_calibrations(const uint8_t array[BMP_180_CALIBRATION_NUMBER])
{
	const size_t size           = (size_t) BMP_180_CALIBRATION_NUMBER;
	const size_t _size          = size /2;
	const uint8_t* p            = array;
	uint16_t  raw_values[_size];

	for(size_t i = 0; i < _size; i ++)
	{
		raw_values[i] = ((*p) << 8) | *(p + 1);
		p  += 2;
	}

	size_t i = 0;

	return (BMP_180_Calibration) {
			.ac1 = raw_values[i ++],
			.ac2 = raw_values[i ++],
			.ac3 = raw_values[i ++],
			.ac4 = raw_values[i ++],
			.ac5 = raw_values[i ++],
			.ac6 = raw_values[i ++],
			.b1  = raw_values[i ++],
			.b2  = raw_values[i ++],
			.mb  = raw_values[i ++],
			.mc  = raw_values[i ++],
			.md  = raw_values[i ++]
	};
}


static int setup_bmp_180_fd(const char* device_path)
{
	const int file = open(device_path, O_RDWR);
	ioctl(file, I2C_SLAVE, I2CDETECT_ADDRESS);
	return file;
}


BMP_180_Calibration get_bmp_calibration(int fd)
{
	const size_t size = BMP_180_CALIBRATION_NUMBER;
	uint8_t buffer[size];

	const uint8_t write_buf[1] = { BMP_180_REGISTER_CALIB_00 };
	write(fd, write_buf, 1);
	read(fd, buffer, BMP_180_CALIBRATION_NUMBER);
	return compute_bmp_calibrations(buffer);
}


void setup_bmp_180(int* fd, BMP_180_Calibration* cal)
{
	*fd  = setup_bmp_180_fd("/dev/i2c-1");
	*cal = get_bmp_calibration(*fd);
}


void read_bmp_180(float* true_temperature, float* true_pressure, const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c)
{
	uint8_t b[3] = { 0 };
	raw_read_bmp_180(b, fd, BMP_180_START_CONVERSION_TEMPERATURE);

	// temperature is 16 bit, skip xlsb
	const int32_t ut = ((b[2] << 8) | b[1]) & ((1 << 16) - 1);

	const BMP_180_Start_Conversion conversion = convert_oss_to_conversion(c);

	raw_read_bmp_180(b, fd, conversion);

	// pressure is up to 19 bit, use them
	const int32_t up = ((b[2] << 11) | (b[1] << 3) | (b[0] & 0x0b00000111)) & ((1 << 19) - 1);

	// the control enumeration is the oss value shifted into the correct position
	const int oss = ((int) c) >> 6;

	const  int32_t x11 = (ut - cal->ac6) * cal->ac5 / (1 << 15);
	const  int32_t x21 = cal->mc * (1 << 11) / (x11 + cal->md);
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

#include <stdio.h>
void debug_read_bmp_180(const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c)
{
	const uint32_t oss = ((uint32_t) c) >> 6;
	const  int32_t ut  = 27898;
	const  int32_t up  = 23843;
	const  int32_t x11 = (ut - cal->ac6) * cal->ac5 / (1 << 15);
	const  int32_t x21 = cal->mc * (1 << 11) / (x11 + cal->md);
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

 	float true_temperature = ((float) ((b5 + 8) / (1 << 4))) / 10.0f;
 	float true_pressure    = p + (x15 + x24 + 3791) / (1 << 4);

	printf(
		"%s,%d,%d,%d,%d,%d,%d,%f,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d,%d,%d,%d,%f\n"
  	, __FUNCTION__
		, ut
		, oss
		, up
		, x11
		, x21
 		, b5
		, true_temperature
 		, b6
 		, x12
 		, x22
 		, x32
 		, b3
 		, x13
 		, x23
 		, x33
 		, b4
 		, b7
 		, p
 		, x14
		, x15
 		, x24
		, true_pressure
  );

	(void) fd;
}

