#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "bmp_180.h"

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

BMP_180_Calibration get_bmp_calibration(int fd)
{
	const size_t size = BMP_180_CALIBRATION_NUMBER;
	uint8_t read_buffer[size];

	for(size_t i = 0; i < size; i ++)
	{
		const uint8_t write_buf[1] = { BMP_180_REGISTER_CALIB_00 + i };
		write(fd, write_buf, 1);

		read(fd, read_buffer + i, 1);
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

