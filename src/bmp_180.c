#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "bmp_180.h"


#include <inttypes.h>


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


typedef enum
{
    BMP_180_OSS_CONTROL_1 = 0b00000000
  , BMP_180_OSS_CONTROL_2 = 0b01000000
  , BMP_180_OSS_CONTROL_4 = 0b10000000
  , BMP_180_OSS_CONTROL_8 = 0b11000000
} BMP_180_OSS_Control;


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


const int DATASHEET_ADDRESS          = 0xEE;
const int I2CDETECT_ADDRESS          = DATASHEET_ADDRESS >> 1;
const int BMP_180_CALIBRATION_NUMBER = 22;


BMP_180_Calibration compute_bmp_calibrations(const uint8_t array[BMP_180_CALIBRATION_NUMBER])
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

#include <stdio.h>
BMP_180_Calibration get_bmp_calibration(int fd)
{
	const size_t size = BMP_180_CALIBRATION_NUMBER;
	uint8_t read_buffer[size];

	for(size_t i = 0; i < size; i ++)
	{
		const uint8_t write_buf[1] = { BMP_180_REGISTER_CALIB_00 + i };
		write(fd, write_buf, 1);

		read(fd, read_buffer + i, 1);

		printf("%.2X,", read_buffer[i] % 0xff);
	}

	return compute_bmp_calibrations(read_buffer);
}

int setup_bmp_180_fd(const char* device_path)
{
	const int file = open(device_path, O_RDWR);
	ioctl(file, I2C_SLAVE, I2CDETECT_ADDRESS);
	return file;
}

void setup_bmp_180(int* fd, BMP_180_Calibration* cal)
{
	*fd  = setup_bmp_180_fd("/dev/i2c-1");
	*cal = get_bmp_calibration(*fd);
}

