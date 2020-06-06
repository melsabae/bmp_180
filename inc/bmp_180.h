#ifndef __BMP_180_H__
#define __BMP_180_H__
#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>


#define DATASHEET_ADDRESS          (0xEE)
#define I2CDETECT_ADDRESS          (DATASHEET_ADDRESS >> 1)
#define BMP_180_CALIBRATION_BYTES  (22)
#define SEA_LEVEL_PRESSURE_PASCALS (101325.0f)


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


int setup_bmp_180_fd(
      int* fd
    , const char* device_path
    ) __attribute__((warn_unused_result)) ;


int raw_bmp_180_write(
      const int fd
    , const uint8_t* data
    , const size_t len
    ) __attribute__((warn_unused_result)) ;


int raw_bmp_180_read(
      uint8_t* data
    , const int fd
    , const size_t len
    ) __attribute__((warn_unused_result)) ;


BMP_180_Calibration compute_bmp_calibrations(
    const uint8_t data[BMP_180_CALIBRATION_BYTES]
    ) __attribute__((warn_unused_result)) ;


int get_bmp_calibration(
      BMP_180_Calibration* cal
    , const int fd
    ) __attribute__((warn_unused_result)) ;


int setup_bmp_180(
      int* fd
    , BMP_180_Calibration* cal
    , const char* device_path
    ) __attribute__((warn_unused_result)) ;


int read_uncompensated_temperature(
      int32_t* ut
    , const int fd
    ) __attribute__((warn_unused_result)) ;


int read_uncompensated_pressure(
      int32_t* up
    , const int fd
    , const BMP_180_OSS_Control c
    ) __attribute__((warn_unused_result)) ;


int read_bmp_180(
      float* true_temperature_celcius
    , float* true_pressure_pascals
    , const int fd
    , const BMP_180_Calibration* cal
    , const BMP_180_OSS_Control c
    ) __attribute__((warn_unused_result)) ;


int read_bmp_180_all(
      float* true_temperature_celcius
    , float* true_pressure_pascals
    , float* altitude_meters
    , const float ref_pressure_pascals
    , const int fd
    , const BMP_180_Calibration* cal
    , const BMP_180_OSS_Control c
    ) __attribute__((warn_unused_result)) ;


BMP_180_Start_Conversion convert_oss_to_conversion(
    const BMP_180_OSS_Control c
    ) __attribute__((warn_unused_result)) ;


useconds_t convert_convesion_to_sleep_interval(
    const BMP_180_Start_Conversion s
    ) __attribute__((warn_unused_result)) ;


float bmp_180_altitude_from_ref(
      const float true_pressure_celcius
    , const float ref_pressure_pascals
    ) __attribute__((warn_unused_result)) ;


int convert_uncompensated_to_true(
      float* true_temperature
    , float* true_pressure_celcius
    , const int32_t ut
    , const int32_t up
    , const BMP_180_OSS_Control c
    , const BMP_180_Calibration* cal
    ) __attribute__((warn_unused_result)) ;


int convert_uncompensated_temperature_to_true(
      float* true_temperature_celcius
    , const int32_t ut
    , const BMP_180_OSS_Control c
    , const BMP_180_Calibration* cal
    ) __attribute__((warn_unused_result)) ;


#ifdef __cplusplus
  }
#endif
#endif // __BMP_180_H__

