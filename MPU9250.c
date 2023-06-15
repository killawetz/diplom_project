/**
  ******************************************************************************
  * @file    MPU9250.h
  * @author  Roman Vassilyev
  * @version V1.0
  * @date    20/03/2023
  * @brief   This file contains all the functions for ineracting with MPU9250
  *          over the I2C.
  ******************************************************************************
*/


#include "MDR32F9Qx_config.h"
#include "MPU9250.h"

I2C_InitTypeDef I2C_InitStruct_MPU9250;


void MPU9250_I2C_Init(I2C_InitTypeDef I2C_InitStructInput) {
	I2C_InitStruct_MPU9250 = I2C_InitStructInput;
}


uint8_t MPU9250_I2C_addressCorrection(uint8_t address) {
	return ((uint8_t)(address << 1));
}


uint8_t MPU9250_readOneRegister(uint8_t regAddr) {
	
		uint8_t slaveAddr = MPU9250_I2C_addressCorrection(MPU9250_ADDRESS);
	
		while (I2C_GetFlagStatus(I2C_FLAG_BUS_FREE) != SET){}
		
		I2C_Send7bitAddress(slaveAddr,I2C_Direction_Transmitter); // START, send slave address, initiate transfer: MASTER -> SLAVE

		while (I2C_GetFlagStatus(I2C_FLAG_SLAVE_ACK) != SET || I2C_GetFlagStatus(I2C_FLAG_TRANS) == SET) {}
		
		I2C_SendByte(regAddr); // send address of the register for reading
			
		while (I2C_GetFlagStatus(I2C_FLAG_SLAVE_ACK) != SET || I2C_GetFlagStatus(I2C_FLAG_TRANS) == SET) {}
				
		I2C_Send7bitAddress(slaveAddr,I2C_Direction_Receiver); // reSTART and initiate transfer MASTER <- SLAVE
			
		while (I2C_GetFlagStatus(I2C_FLAG_SLAVE_ACK) != SET || I2C_GetFlagStatus(I2C_FLAG_TRANS) == SET) {}

		I2C_StartReceiveData(I2C_Send_to_Slave_NACK);

		while (I2C_GetFlagStatus(I2C_FLAG_TRANS) == SET) {}

		uint8_t res = I2C_GetReceivedData();
		
		I2C_SendSTOP();
			
		return res;
	
}


void MPU9250_readAllAccelData(uint8_t array[]) {
		array[0] = MPU9250_readOneRegister(ACCEL_X_H_ADDRESS);
		array[1] = MPU9250_readOneRegister(ACCEL_X_L_ADDRESS);
		array[2] = MPU9250_readOneRegister(ACCEL_Y_H_ADDRESS);
		array[3] = MPU9250_readOneRegister(ACCEL_Y_L_ADDRESS);
		array[4] = MPU9250_readOneRegister(ACCEL_Z_H_ADDRESS);
		array[5] = MPU9250_readOneRegister(ACCEL_Z_L_ADDRESS);
}

void MPU9250_readAllGyroData(uint8_t array[]) {
		array[0] = MPU9250_readOneRegister(GYRO_X_H_ADDRESS);
		array[1] = MPU9250_readOneRegister(GYRO_X_L_ADDRESS);
		array[2] = MPU9250_readOneRegister(GYRO_Y_H_ADDRESS);
		array[3] = MPU9250_readOneRegister(GYRO_Y_L_ADDRESS);
		array[4] = MPU9250_readOneRegister(GYRO_Z_H_ADDRESS);
		array[5] = MPU9250_readOneRegister(GYRO_Z_L_ADDRESS);
}

int16_t convertTo16Bit(uint8_t high, uint8_t low) {
		return (int16_t) ( high << 8 | low);
}
