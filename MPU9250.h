/**
  ******************************************************************************
  * @file    MPU9250.h
  * @author  Roman Vassilyev
  * @version V1.0
  * @date    20/03/2023
  * @brief   This file contains all the functions prototypes for ineracting with MPU9250
  *          over the I2C.
  ******************************************************************************
*/

// include guards
#ifndef MPU9250_H
#define MPU9250_H

// подключение необходимых файлов
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_i2c.h"
#include "MDR32F9Qx_port.h"


#define MPU9250_ADDRESS		0x68

// accelerometer registers addresses
#define ACCEL_X_H_ADDRESS 0x3B
#define ACCEL_X_L_ADDRESS 0x3C
#define ACCEL_Y_H_ADDRESS 0x3D	
#define ACCEL_Y_L_ADDRESS 0x3E
#define ACCEL_Z_H_ADDRESS 0x3F	
#define ACCEL_Z_L_ADDRESS 0x40


//gyroscope registers addresses
#define GYRO_X_H_ADDRESS 0x43
#define GYRO_X_L_ADDRESS 0x44
#define GYRO_Y_H_ADDRESS 0x45	
#define GYRO_Y_L_ADDRESS 0x46
#define GYRO_Z_H_ADDRESS 0x47	
#define GYRO_Z_L_ADDRESS 0x48




void MPU9250_I2C_Init(I2C_InitTypeDef I2C_InitStruct);
uint8_t MPU9250_I2C_addressCorrection(uint8_t address);
uint8_t MPU9250_readOneRegister(uint8_t address);
void MPU9250_readAllAccelData(uint8_t array[]);
void MPU9250_readAllGyroData(uint8_t array[]);
int16_t convertTo16Bit(uint8_t high, uint8_t low); 




#endif