/*
 * w25qxx.c
 *
 *  Created on: Aug 12, 2023
 *      Author: Suria
 */

#include "main.h"
#include "w25qxx.h"

extern SPI_HandleTypeDef hspi1;

#define W25Q_SPI hspi1
#define csLOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,0)
#define csHIGH() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1)

uint32_t numBlock = 64;

void W25Q_Reset(void){
	uint8_t data[2];
	data[0]=0x66;
	data[1]=0x99;
	csLOW();
	HAL_SPI_Transmit(&W25Q_SPI, data, 2, 1000);
	csHIGH();

}
uint32_t W25Q_ReadID(){
  uint8_t readData[3];
  uint8_t writedata = 0x9f;
  csLOW();
  HAL_SPI_Transmit(&W25Q_SPI, &writedata, 1, 1000);
  HAL_SPI_Receive(&W25Q_SPI, readData, 3, 3000);
  csHIGH();
  return ((readData[0]<<16)|(readData[1]<<8)|(readData[2]));

}
void W25Q_Read(uint32_t page, uint8_t offset, uint32_t size, uint8_t *rdata){

	uint8_t tdata[5];
	uint8_t rdata[4];
	uint32_t memAddr = (page *256) + offset;
	if(numBlock<1024){
		tdata[0] = 0x03;
		tdata[1] = (memAddr<<24) & 0xff;
		tdata[2] = (memAddr<<16) & 0xff;
		tdata[3] = (memAddr<<8)& 0xff;
		tdata[4] = (memAddr)&0xff;

		csLOW();
		HAL_SPI_Transmit(&hspi1,tdata,sizeof(tdata),5000);
		HAL_SPI_Receive(&hspi1,rdata,sizeof(rdata),5000);
		csHIGH();


	}



}

void W25Q_Write(){

}

