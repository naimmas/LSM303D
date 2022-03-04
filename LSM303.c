/*
 * LSM303.c
 *
 *  Created on: Jan 16, 2022
 *      Author: ajanx
 */
#include "LSM303.h"
#include "stm32h743xx.h"

LSM303D_ErrorTypeDef LSM303D_CheckDevice(LSM303D *dev)
{
	uint8_t id=0x0;
	if(HAL_I2C_IsDeviceReady(dev->i2cHandle, LSM303D_SA0H_R, 5, 100) == HAL_OK)
		LSM303D_ReadRegister(dev, LSM303D_WHO_AM_I, &id);
	return (id==LSM303D_Device_ID) ? (LSM303_SENSOR_CONNECTION_OK) : (LSM303_SENSOR_CONNECTION_ERROR);
}
#ifdef DEBUG_EN
LSM303D_ErrorTypeDef LSM303D_Init(LSM303D *dev,  I2C_HandleTypeDef *i2cHandle, uint8_t* errHandling)
#else
LSM303D_ErrorTypeDef LSM303D_Init(LSM303D *dev, I2C_HandleTypeDef *i2cHandle)
#endif
{
	dev->i2cHandle = i2cHandle;
#ifdef DEBUG_EN
	*errHandling = 0x0;
#endif
	uint8_t dataWr;
	if(LSM303D_CheckDevice(dev) == LSM303_SENSOR_CONNECTION_ERROR) {
#ifdef DEBUG_EN
		*errHandling = 0xF;
#endif
		return LSM303_SENSOR_CONNECTION_ERROR;
	}

	dataWr = LSM303D_ACC_DATARATE_800_Hz | LSM303D_ACC_ENABLE_3AXES;
	if(LSM303D_WriteRegister(dev, LSM303D_CTRL1, &dataWr)!=HAL_OK) {
#ifdef DEBUG_EN
		(*errHandling |= 0x1<<1);
#endif
	}
	
	dataWr = LSM303D_ACC_ANTI_ALIAS_FILTER_BW_773_Hz | LSM303D_ACC_FULL_SCALE_2G;
	LSM303D_Acc_Sens = LSM303D_ACC_2G_SENS;
	if(LSM303D_WriteRegister(dev, LSM303D_CTRL2, &dataWr)!=HAL_OK) {
#ifdef DEBUG_EN
		(*errHandling |= 0x1<<1);
#endif
	}

	dataWr = LSM303D_MAG_DATARATE_6_25_Hz | LSM303D_MAG_RES_HIGH;
	if(LSM303D_WriteRegister(dev, LSM303D_CTRL5, &dataWr)!=HAL_OK) {
#ifdef DEBUG_EN
		(*errHandling |= 0x1<<3);
#endif
	}
	
	dataWr = LSM303D_MAG_FULL_SCALE_2GAUSS;
	LSM303D_Mag_Sens = LSM303D_MAG_2G_SENS;

	if(LSM303D_WriteRegister(dev, LSM303D_CTRL6, &dataWr)!=HAL_OK) {
#ifdef DEBUG_EN
		(*errHandling |= 0x1<<4);
#endif
	}
	
	dataWr = LSM303D_HIGH_PASS_FILTER_MODE_NORMAL_MODE;
	if(LSM303D_WriteRegister(dev, LSM303D_CTRL7, &dataWr)!=HAL_OK) {
#ifdef DEBUG_EN
		(*errHandling |= 0x1<<4);
#endif
	}
#ifdef DEBUG_EN
		return (*errHandling==0x00) ? REGISTER_WRITE_OK : REGISTER_WRITE_ERROR;
#endif

#ifndef DEBUG_EN
	return LSM303_SENSOR_CONNECTION_OK;
#endif
}

LSM303D_ErrorTypeDef LSM303D_ReadAcc(LSM303D *dev)
{
	uint8_t inComeData[6]; HAL_StatusTypeDef state; int16_t tempData[3];
	state  = LSM303D_ReadRegister(dev, LSM303D_OUT_X_L_A, &inComeData[0]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_X_H_A, &inComeData[1]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Y_L_A, &inComeData[2]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Y_H_A, &inComeData[3]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Z_L_A, &inComeData[4]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Z_H_A, &inComeData[5]);
	if (state==HAL_OK)
	{
		tempData[0] = (int16_t) (inComeData[1]<<8 | inComeData[0]);
		tempData[1] = (int16_t) (inComeData[3]<<8 | inComeData[2]);
		tempData[2] = (int16_t) (inComeData[5]<<8 | inComeData[4]);

		dev->acc[0] = (double)(tempData[0]) * LSM303D_Acc_Sens * GRAVITY / 1000;
		dev->acc[1] = (double)(tempData[1]) * LSM303D_Acc_Sens * GRAVITY / 1000;
		dev->acc[2] = (double)(tempData[2]) * LSM303D_Acc_Sens * GRAVITY / 1000;
		return LSM303_REGISTER_WRITE_OK;
	}
	return LSM303_REGISTER_WRITE_ERROR;
}

LSM303D_ErrorTypeDef LSM303D_ReadMag(LSM303D *dev)
{
	uint8_t inComeData[6]; HAL_StatusTypeDef state; int16_t tempData[3];

	state  = LSM303D_ReadRegister(dev, LSM303D_OUT_X_L_M, &inComeData[0]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_X_H_M, &inComeData[1]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Y_L_M, &inComeData[2]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Y_H_M, &inComeData[3]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Z_L_M, &inComeData[4]);
	state |= LSM303D_ReadRegister(dev, LSM303D_OUT_Z_H_M, &inComeData[5]);
	if (state==HAL_OK)
	{
		tempData[0] = (int16_t) (inComeData[1]<<8 | inComeData[0]);
		tempData[1] = (int16_t) (inComeData[3]<<8 | inComeData[2]);
		tempData[2] = (int16_t) (inComeData[5]<<8 | inComeData[4]);

		dev->mag[0] = (float)tempData[0] * LSM303D_Mag_Sens / 1000;
		dev->mag[1] = (float)tempData[1] * LSM303D_Mag_Sens / 1000;
		dev->mag[2] = (float)tempData[2] * LSM303D_Mag_Sens / 1000;

		return LSM303_REGISTER_WRITE_OK;
	}
	return LSM303_REGISTER_WRITE_ERROR;
}

HAL_StatusTypeDef LSM303D_ReadRegister(LSM303D *dev, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM303D_SA0H_R, reg, 1, data,
			I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LSM303D_WriteRegister(LSM303D *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, LSM303D_SA0H_W, reg,
			I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

