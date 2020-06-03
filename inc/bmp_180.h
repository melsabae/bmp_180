#ifndef __BMP_180_H__
#define __BMP_180_H__

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


BMP_180_Calibration get_bmp_calibration(int fd);
void setup_bmp_180(int* fd, BMP_180_Calibration* cal);
void read_bmp_180(float* true_temperature, float* true_pressure, const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c);
void debug_read_bmp_180(const int fd, const BMP_180_Calibration* cal, const BMP_180_OSS_Control c);


#ifdef __cplusplus
  }
#endif

#endif // __BMP_180_H__

