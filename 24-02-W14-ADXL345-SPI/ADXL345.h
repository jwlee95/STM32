/*
 * @File: ADXL345.h
 * @Driver Name: [ADXL345 3-axis accelometer - SPI I/F ]]
 * @SW Layer:   BMEAL
 * Created on: Sep 22, 2023
 * @Author:     JeongWhan Lee
 *
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "main.h"

//Adxl345 Device Address
// #define adxl_address 0x53<<1  // 0101 0011  (0x53)

#define DEVID		0x00

//Set SPI Handeler here
extern SPI_HandleTypeDef hspi1;
extern uint8_t data_rec[6];

void adxl_write (uint8_t address, uint8_t value);
void adxl_read (uint8_t address);
void adxl_init (void);

HAL_StatusTypeDef ReadRegister(uint8_t addr, uint8_t *byte);



#endif /* INC_ADXL345_H_ */
