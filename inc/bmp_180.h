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


BMP_180_Calibration get_bmp_calibration(int fd);
int setup_bmp_180_fd(const char* device_path);
void setup_bmp_180(int* fd, BMP_180_Calibration* cal);


#ifdef __cplusplus
  }
#endif

#endif // __BMP_180_H__

