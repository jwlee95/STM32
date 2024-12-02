/*
 * @File: ADXL345.c
 * @Driver Name: [ADXL345 3-axis accelometer - SPI I/F ]]
 * @SW Layer:   BMEAL
 * Created on: Sep 22, 2023
 * @Author:     JeongWhan Lee
 *
 */

#include "ADXL345.h"

uint8_t data_rec[6];
uint8_t chipid=0;
char x_char[3], y_char[3], z_char[3];


void adxl_init (void){

	adxl_write (0x31, 0x00);  	// sets D6 = 0 ; the device to 4-wire SPI mode.
								// data_format range= +- 2g
	/*
	 * Register 0x31—DATA_FORMAT (Read/Write)
	 * D7         D6   D5          D4  D3        D2       D1:D0
	 * SELF_TEST  SPI  INT_INVERT  0   FULL_RES  Justify  Range
	 *
	 * 	D1 D0	g Range
	 * 	0  0 	±2g
	 * 	0  1 	±4g
	 * 	1  0 	±8g
	 * 	1  1 	±16g
	 */

	adxl_write (0x2d, 0x00);  // reset all bits
	adxl_write (0x2d, 0x08);  // power_cntl measure and wake up 8hz

	/*
	 * Register 0x2D—POWER_CTL (Read/Write)
	 * D7 D6 D5   D4         D3      D2    D1:D0
	 * 0  0  Link Auto_Sleep Measure Sleep Wakeup
	 *
	 */


/*
 * Register 0x2C—BW_RATE (Read/Write)
 * D7 D6 D5 D4         D3:D2:D1:D0
 * 0  0  0  LOW_POWER  Rate
 *
 *   BW value(HEX) 	|  Output Data Rate (Hz)
	---------------------------------
		6  (0110)	|   6.25 // Default
		7  (0111)	|  12.5
		8  (1000)	|  25
		9  (1001)	|  50
		A  (1010)	|  100
		B  (1011)	|  200
		C  (1100)	|  400
		D  (1101)	|  800
		E  (1110)	|  1600
		F  (1111)	|  3200
 */
	adxl_write (0x2c, 0x08);  // reset all bits


}


void adxl_write (uint8_t address, uint8_t value){
uint8_t data[2];
	data[0] = address | 0x40 ;  // multibyte read
	data[1] = value;
	/*
	 * NOTE that in data[0], address is OR with 0x40.
	 * This is for multibyte writing.
	 * It informs ADXL that we want to transfer more
	 *   than one byte in a single transmission.
	 * According to ADXL datasheet,
	 * this byte should be high if you want to do that.
	 *
	 *
	 * Clearing the SPI bit (Bit D6) in the DATA_FORMAT register (Address 0x31)
	 *   selects 4-wire mode, whereas setting the SPI bit selects 3-wire mode
	 * The maximum SPI clock speed is 5 MHz with 100 pF maximum loading,
	 *   and the timing scheme follows clock polarity (CPOL) = 1 and clock phase (CPHA) = 1.
	 * To read or write multiple bytes in a single transmission,
	 *   the multiple-byte bit, located after the R/W bit in the first byte transfer must be set
	 *  After the register addressing and the first byte of data, each subsequent set of clock pulses
	 *   (eight clock pulses) causes the ADXL345 to point to the next register for a read or write.
	 *
	 * */

	HAL_GPIO_WritePin (mySS_GPIO_Port, mySS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&hspi1, data, 2, 100);  // write data to register
	HAL_GPIO_WritePin (mySS_GPIO_Port, mySS_Pin, GPIO_PIN_SET);
}


void adxl_read (uint8_t address)
{
	address |= 0x80;  // read operation
	address |= 0x40;  // multibyte read

	HAL_GPIO_WritePin (mySS_GPIO_Port, mySS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit (&hspi1, &address, 1, 100);  // send address
	//HAL_Delay(1);
	HAL_SPI_Receive (&hspi1, data_rec, 6, 100);   // receive 6 bytes data
	HAL_GPIO_WritePin (mySS_GPIO_Port, mySS_Pin, GPIO_PIN_SET);
}


HAL_StatusTypeDef ReadRegister(uint8_t addr, uint8_t *byte)
{
	HAL_StatusTypeDef hal_status;
	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = addr | 0x80;  	// read operation
	tx_data[1] = 0;				// dummy byte for response

	HAL_GPIO_WritePin (mySS_GPIO_Port, mySS_Pin, GPIO_PIN_RESET);
	hal_status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
	HAL_GPIO_WritePin (mySS_GPIO_Port, mySS_Pin, GPIO_PIN_SET);

	if (hal_status == HAL_OK) {
		*byte = rx_data[1]; // response is in the second byte
	}
	return hal_status;
}


