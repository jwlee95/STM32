/*
 * @File: adxl345_i2c.h
 * @Driver Name: [ADXL345 3-axis accelometer - I2C I/F ]]
 * @SW Layer:   BMEAL
 * Created on: Sep 22, 2023
 * @Author:     JeongWhan Lee
 *
 */

#ifndef ADXL345_I2C_H_
#define ADXL345_I2C_H_

// Change the library according to your uC
//#include "stm32l1xx_hal.h"
#include "main.h"

//Adxl345 Device Address 
#define adxl_address 0x53<<1	// given in date spec.


void adxl_write (I2C_HandleTypeDef * hi2c, uint8_t reg, uint8_t value);
void adxl_read_values (I2C_HandleTypeDef * hi2c, uint8_t reg);
void adxl_read_address (I2C_HandleTypeDef *hi2c, uint8_t reg);
void adxl_init (I2C_HandleTypeDef * hi2c);
int16_t adxl_readx(I2C_HandleTypeDef * hi2c);
int16_t adxl_ready(I2C_HandleTypeDef * hi2c);
int16_t adxl_readz(I2C_HandleTypeDef * hi2c);


#endif /* ADXL345_I2C_H_ */
