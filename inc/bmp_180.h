#ifndef __BMP_180_H__
#define __BMP_180_H__


#define DATASHEET_ADDRESS         (0xEE)
#define I2CDETECT_ADDRESS         (DATASHEET_ADDRESS >> 1)
#define BMP_180_CALIBRATION_BYTES (22)


#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>


typedef struct
{
	int16_t  ac1;
	int16_t  ac2;
	int16_t  ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t  b1 ;
	int16_t  b2 ;
	int16_t  mb ;
	int16_t  mc ;
	int16_t  md ;
} BMP_180_Calibration;


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


BMP_180_Start_Conversion convert_oss_to_conversion(const BMP_180_OSS_Control c);

useconds_t convert_convesion_to_sleep_interval(const BMP_180_Start_Conversion s);

int32_t raw_read_temperature(const int fd);

int32_t raw_read_pressure(const int fd, const BMP_180_OSS_Control c);

BMP_180_Calibration compute_bmp_calibrations(const uint8_t array[BMP_180_CALIBRATION_BYTES]);

int setup_bmp_180_fd(const char* device_path);

void convert_raw_to_true(float* true_temperature, float* true_pressure, const int32_t ut, const int32_t up, const BMP_180_OSS_Control c, const BMP_180_Calibration* cal);

BMP_180_Calibration get_bmp_calibration(const int fd);

void setup_bmp_180(int* fd, BMP_180_Calibration* cal, const char* file_path);

void read_bmp_180(float* true_temperature, float* true_pressure, const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c);

float bmp_180_altitude(const float true_pressure);

void read_bmp_180_all(float* true_temperature, float* true_pressure, float* altitude, const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c);


#ifdef __cplusplus
  }
#endif

#endif // __BMP_180_H__

