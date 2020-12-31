/*
 * bmp180.c
 *
 *  Created on: 27 feb. 2019
 *      Author: gheorghe.ghirjev
 */

#include <string.h>
#include <math.h>
#include "bmp180.h"
#include "stm32f4xx_hal.h"


// global struct to store all BMP180 main data.
bmp_t bmp;

extern I2C_HandleTypeDef hi2c3;


/*!
* @brief:    - Performe initial sequence of BMP sensor
* @param[in] - pointer to struct of type bmp_calib_param_t
* @return    - None.
*/
void bmp_init (bmp_t * bmp)
{
	uint8_t i2cBuffer[2];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xAA, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xAB, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.AC1 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xAC, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xAD, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.AC2 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xAE, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xAF, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.AC3 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB0, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB1, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.AC4 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB2, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB3, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.AC5 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB4, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB5, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.AC6 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB6, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB7, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.B1 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB8, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xB9, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.B2 = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xBA, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xBB, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.MB = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xBC, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xBD, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.MC = i2cBuffer[0]<<8|i2cBuffer[1];
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xBE, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xBF, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	bmp->calib.MD = i2cBuffer[0]<<8|i2cBuffer[1];
}

/*!
* @brief:    - Compute Temperature.
* @param[in] - pointer to struct of type bmp_calib_param_t
* @return    - Floating point temperature value.
*/
float get_temp(bmp_t * bmp){
	uint8_t i2cBuffer[2];
	i2cBuffer[0] = BMP_SET_TEMP_CONV;
	HAL_I2C_Mem_Write( &hi2c3, 0x00, BMP_CTRL_REG, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_Delay (BMP_TEMP_CONV_TIME);
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xF6, 1, &i2cBuffer[0], 1, BMP_I2C_TIMEOUT );
	HAL_I2C_Mem_Read ( &hi2c3, 0x00, 0xF7, 1, &i2cBuffer[1], 1, BMP_I2C_TIMEOUT );
	//Compute non compensated temperature.
	int32_t uncomp_tmp = (i2cBuffer[0] << BYTE_SHIFT) | i2cBuffer[1];
	//Compensate the temperature.
	int32_t X1 = 0;
	int32_t X2 = 0;
	float temp = 0;
	X1 = ((uncomp_tmp - bmp->calib.AC6) * bmp->calib.AC5) >> 15;
	X2 = (bmp->calib.MC << 11) / (X1 + bmp->calib.MD);
	bmp->data.B5 = X1 + X2;
	temp = ((bmp->data.B5 + 8) >> 4) * 0.1f;
	return temp;
}


