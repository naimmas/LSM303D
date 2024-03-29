/*
 * LSM303.h
 *
 *  Created on: Jan 16, 2022
 *      Author: ajanx
 */

#ifndef L303D_LSM303D_H_
#define L303D_LSM303D_H_
#ifndef stdint
#include <stdint.h>
#endif
//#define C_SIMULATION
#ifndef C_SIMULATION
#include "stm32h7xx_hal.h"
#include "stm32h743xx.h"
#include "arm_math.h"
#endif
#define GRAVITY 9.80665F

#include <stdlib.h>

/* Register address map */
#define LSM303D_TEMP_OUT_L 	    ((uint8_t)(0x05))	/*OUTPUT*/
#define LSM303D_TEMP_OUT_H 	    ((uint8_t)(0x06))	/*OUTPUT*/
#define LSM303D_STATUS_M 	    ((uint8_t)(0x07))	/*OUTPUT*/
#define LSM303D_OUT_X_L_M	    ((uint8_t)(0x08))	/*OUTPUT*/
#define LSM303D_OUT_X_H_M	    ((uint8_t)(0x09))	/*OUTPUT*/
#define LSM303D_OUT_Y_L_M	    ((uint8_t)(0x0A))	/*OUTPUT*/
#define LSM303D_OUT_Y_H_M	    ((uint8_t)(0x0B))	/*OUTPUT*/
#define LSM303D_OUT_Z_L_M	    ((uint8_t)(0x0C))	/*OUTPUT*/
#define LSM303D_OUT_Z_H_M	    ((uint8_t)(0x0D))	/*OUTPUT*/

#define LSM303D_WHO_AM_I		    ((uint8_t)(0x0F))	/*DEFAULT 01001001*/

#define LSM303D_INT_CTRL_M 	    ((uint8_t)(0x12))	/*DEFAULT 11101000*/
#define LSM303D_INT_SRC_M 	    ((uint8_t)(0x13))	/*OUTPUT*/
#define LSM303D_INT_THS_L_M	    ((uint8_t)(0x14))	/*DEFAULT 00000000*/
#define LSM303D_INT_THS_H_M 	    ((uint8_t)(0x15))	/*DEFAULT 00000000*/

#define LSM303D_OFFSET_X_L_M     ((uint8_t)(0x16))	/*DEFAULT 00000000*/
#define LSM303D_OFFSET_X_H_M     ((uint8_t)(0x17))	/*DEFAULT 00000000*/
#define LSM303D_OFFSET_Y_L_M     ((uint8_t)(0x18))	/*DEFAULT 00000000*/
#define LSM303D_OFFSET_Y_H_M     ((uint8_t)(0x19))	/*DEFAULT 00000000*/
#define LSM303D_OFFSET_Z_L_M     ((uint8_t)(0x1A))	/*DEFAULT 00000000*/
#define LSM303D_OFFSET_Z_H_M     ((uint8_t)(0x1B))	/*DEFAULT 00000000*/

#define LSM303D_REFERENCE_X      ((uint8_t)(0x1C))	/*DEFAULT 00000000*/
#define LSM303D_REFERENCE_Y      ((uint8_t)(0x1D))	/*DEFAULT 00000000*/
#define LSM303D_REFERENCE_Z      ((uint8_t)(0x1E))	/*DEFAULT 00000000*/

#define LSM303D_CTRL0            ((uint8_t)(0x1F))	/*DEFAULT 00000000*/
#define LSM303D_CTRL1            ((uint8_t)(0x20))	/*DEFAULT 00000111*/
#define LSM303D_CTRL2            ((uint8_t)(0x21))	/*DEFAULT 00000000*/
#define LSM303D_CTRL3            ((uint8_t)(0x22))    /*DEFAULT 00000000*/
#define LSM303D_CTRL4            ((uint8_t)(0x23))    /*DEFAULT 00000000*/
#define LSM303D_CTRL5            ((uint8_t)(0x24))    /*DEFAULT 00011000*/
#define LSM303D_CTRL6            ((uint8_t)(0x25))    /*DEFAULT 00100000*/
#define LSM303D_CTRL7            ((uint8_t)(0x26))    /*DEFAULT 00000001*/

#define LSM303D_STATUS_A         ((uint8_t)(0x27))    /*Output*/

#define LSM303D_OUT_X_L_A        ((uint8_t)(0x28))    /*Output*/
#define LSM303D_OUT_X_H_A        ((uint8_t)(0x29))    /*Output*/
#define LSM303D_OUT_Y_L_A        ((uint8_t)(0x2A))    /*Output*/
#define LSM303D_OUT_Y_H_A        ((uint8_t)(0x2B))    /*Output*/
#define LSM303D_OUT_Z_L_A        ((uint8_t)(0x2C))    /*Output*/
#define LSM303D_OUT_Z_H_A        ((uint8_t)(0x2D))    /*Output*/

#define LSM303D_IG_CFG1          ((uint8_t)(0x30))    /*DEFUALT 00000000*/
#define LSM303D_IG_SRC1          ((uint8_t)(0x31))    /*Output*/
#define LSM303D_IG_THS1          ((uint8_t)(0x32))    /*DEFUALT 00000000*/
#define LSM303D_IG_DUR1          ((uint8_t)(0x33))    /*DEFUALT 00000000*/
#define LSM303D_IG_CFG2          ((uint8_t)(0x34))    /*DEFUALT 00000000*/
#define LSM303D_IG_SRC2          ((uint8_t)(0x35))    /*Output*/
#define LSM303D_IG_THS2          ((uint8_t)(0x36))    /*DEFUALT 00000000*/
#define LSM303D_IG_DUR2          ((uint8_t)(0x37))    /*DEFUALT 00000000*/

#define LSM303D_TIME_LIMIT       ((uint8_t)(0x3B))    /*DEFUALT 00000000*/
#define LSM303D_TIME_LATENCY     ((uint8_t)(0x3C))    /*DEFUALT 00000000*/
#define LSM303D_TIME_WINDOW      ((uint8_t)(0x3D))    /*DEFUALT 00000000*/

#define LSM303D_ACT_THS          ((uint8_t)(0x3E))    /*DEFUALT 00000000*/
#define LSM303D_ACT_DUR          ((uint8_t)(0x3F))    /*DEFUALT 00000000*/

#define LSM303D_SA0H_R			((uint8_t)(0x3B))
#define LSM303D_SA0H_W			((uint8_t)(0x3A))

#define LSM303D_Device_ID		((uint8_t)(0x49))	/*WHO_AM_I register content*/

//#define DEBUG_EN

typedef struct {
	/* I2C handle */
	#ifndef C_SIMULATION
	I2C_HandleTypeDef *i2cHandle;
	#endif
	/* Acceleration data (X, Y, Z)*/
	float raw_acc[3];
	/* Magnetic data (X, Y, Z)*/
	float raw_mag[3];
	/* Temperature data in deg */
	int16_t temp_C;
	#ifdef DEBUG_EN
	uint8_t err;
	#endif
} LSM303D;

typedef enum CTRL1{
	//Acceleration data-rate selection.
	AODR3	= 0x80, AODR2 = 0x40, AODR1	= 0x20, AODR0 = 0x10,
	//Block data update for acceleration and magnetic data.
	BDU		= 0x08,
	//Acceleration Z-axis enable
	AZEN	= 0x04,
	//Acceleration Y-axis enable
	AYEN	= 0x02,
	//A cceleration X-axis enable	
	AXEN	= 0x01
}LSM303D_CTRL1TypeDef;

typedef enum CTRL2{
	// Accelerometer anti-alias filter bandwidth.
	ABW1 = 0x80, ABW0 = 0x40,				
	//Acceleration full-scale selection.
	AFS2 = 0x20, AFS1 = 0x10, AFS0 = 0x08,	
	//Acceleration full-scale selection
	AST	= 0x02,	
	//SPI serial interface mode selection.
	SIM	= 0x01	
} LSM303D_CTRL2TypeDef;

typedef enum CTRL5{
	//Temperature sensor enable
	TEMP_EN	= 0x80,	
	//Magnetic resolution selection
	M_RES1	= 0x40, M_RES0 = 0x20,
	//Magnetic data rate selection.
	M_ODR2	= 0x10, M_ODR1 = 0x08, M_ODR0 = 0x04,
	//Latch interrupt request on INT2_SRC register, with INT2_SRC register cleared by reading INT2_SRC itself
	LIR2	= 0x02,	
	//Latch interrupt request on INT1_SRC register, with INT1_SRC register cleared by reading INT1_SRC itself. 
	LIR1	= 0x01	
} LSM303D_CTRL5TypeDef;

typedef enum CTRL6{
	MFS1 = 0x40, MFS0 = 0x20 //Magnetic full-scale selection.
} LSM303D_CTRL6TypeDef;

typedef enum CTRL7{
	//High-pass filter mode selection for acceleration data. 
	AHPM1 	= 0x80, AHPM0 =0x40,	
	//Filtered acceleration data selection. 
	AFDS 	= 0x20,	
	//Temperature sensor only mode
	T_ONLY 	= 0x10,	
	//Magnetic data low-power mode.
	MLP 	= 0x04,	
	//Magnetic sensor mode selection.
	MD1 	= 0x02, MD0 = 0x01
} LSM303D_CTRL7TypeDef;

//************ REGISTER CTRL1 ************//
/* Acceleration data-rate selection REGISTER CTRL1*/
#define LSM303D_ACC_POWER_DOWN_MODE				((uint8_t)(0x00))
#define LSM303D_ACC_DATARATE_3_125_Hz			((uint8_t)(AODR0))
#define LSM303D_ACC_DATARATE_6_25_Hz			((uint8_t)(AODR1))
#define LSM303D_ACC_DATARATE_12_5_Hz			((uint8_t)(AODR1 | AODR0))
#define LSM303D_ACC_DATARATE_25_Hz				((uint8_t)(AODR2))
#define LSM303D_ACC_DATARATE_50_Hz				((uint8_t)(AODR2 | AODR0))
#define LSM303D_ACC_DATARATE_100_Hz				((uint8_t)(AODR2 | AODR1))
#define LSM303D_ACC_DATARATE_200_Hz				((uint8_t)(AODR2 | AODR1 | AODR0))
#define LSM303D_ACC_DATARATE_400_Hz				((uint8_t)(AODR3))
#define LSM303D_ACC_DATARATE_800_Hz				((uint8_t)(AODR2 | AODR0))
#define LSM303D_ACC_DATARATE_1600_Hz			((uint8_t)(AODR2 | AODR1))
/*Enable acc mesurement on x y z axes*/
#define LSM303D_ACC_ENABLE_3AXES				((uint8_t)(AZEN | AYEN | AXEN))

//************ REGISTER CTRL2 ************//
/*Acceleration anti-alias filter bandwidth*/
#define  LSM303D_ACC_ANTI_ALIAS_FILTER_BW_773_Hz	((uint8_t)(0x00))
#define  LSM303D_ACC_ANTI_ALIAS_FILTER_BW_194_Hz	((uint8_t)(ABW0))
#define  LSM303D_ACC_ANTI_ALIAS_FILTER_BW_362_Hz	((uint8_t)(ABW1))
#define  LSM303D_ACC_ANTI_ALIAS_FILTER_BW_50_Hz		((uint8_t)(ABW1 | ABW0))
/*Acceleration full-scale selection REGISTER CTRL2*/
#define LSM303D_ACC_FULL_SCALE_2G				((uint8_t)(0X00))
#define LSM303D_ACC_FULL_SCALE_4G				((uint8_t)(AFS0))
#define LSM303D_ACC_FULL_SCALE_6G				((uint8_t)(AFS1))
#define LSM303D_ACC_FULL_SCALE_8G				((uint8_t)(AFS1 | AFS0))
#define LSM303D_ACC_FULL_SCALE_16G				((uint8_t)(AFS2))

//************ REGISTER CTRL5 ************//
/*Magnetic data rate configuration REGISTER CTRL5*/
#define LSM303D_MAG_DATARATE_3_125_Hz			((uint8_t)(0X00))
#define LSM303D_MAG_DATARATE_6_25_Hz			((uint8_t)(M_ODR0))
#define LSM303D_MAG_DATARATE_12_5_Hz			((uint8_t)(M_ODR1))
#define LSM303D_MAG_DATARATE_25_Hz				((uint8_t)(M_ODR1 | M_ODR0))
#define LSM303D_MAG_DATARATE_50_Hz				((uint8_t)(M_ODR2))
#define LSM303D_MAG_DATARATE_DONT_USE			((uint8_t)(M_ODR2 | M_ODR1))
/*Magnetic resolution selection*/
#define LSM303D_MAG_RES_LOW						((uint8_t)(0x00))
#define LSM303D_MAG_RES_HIGH					((uint8_t)(M_RES1 | M_RES0))
/*Temperature sensor enable*/
#define LSM303D_ENABLE_TEMP_SENS					((uint8_t)(TEMP_EN))

//************ REGISTER CTRL6 ************//
/*Magnetic full-scale selection REGISTER CTRL6*/
#define LSM303D_MAG_FULL_SCALE_2GAUSS			((uint8_t)(0X00))
#define LSM303D_MAG_FULL_SCALE_4GAUSS			((uint8_t)(MFS0))
#define LSM303D_MAG_FULL_SCALE_8GAUSS			((uint8_t)(MFS1))
#define LSM303D_MAG_FULL_SCALE_12GAUSS			((uint8_t)(MFS1 | MFS0))

//************ REGISTER CTRL7 ************//
/*High-pass filter mode selection REGISTER CTRL7*/
#define LSM303D_HIGH_PASS_FILTER_MODE_NORMAL_MODE_REF	((uint8_t)(0X00))
#define LSM303D_HIGH_PASS_FILTER_MODE_REF_SIG_FILTERING	((uint8_t)(AHPM0))
#define LSM303D_HIGH_PASS_FILTER_MODE_NORMAL_MODE		((uint8_t)(AHPM1))
#define LSM303D_HIGH_PASS_FILTER_MODE_AUTO_INT_RESET	((uint8_t)(AHPM1 | AHPM0))

/*Magnetic sensor mode selection REGISTER CTRL7*/
#define LSM303D_MAG_SENSOR_MODE_CONT_CONV_MODE			((uint8_t)(0X00))
#define LSM303D_MAG_SENSOR_MODE_SINGLE_CONV_MODE		((uint8_t)(MD0))
#define LSM303D_MAG_SENSOR_MODE_PWR_DOWN_MODE			((uint8_t)(MD1))
#define LSM303D_MAG_SENSOR_LOW_PWR_MODE					((uint8_t)(MLP))

#define LSM303D_ACC_2G_SENS	 0.061036F
#define LSM303D_ACC_4G_SENS	 0.122072F
#define LSM303D_ACC_6G_SENS	 0.183108F
#define LSM303D_ACC_8G_SENS	 0.244144F
#define LSM303D_ACC_16G_SENS 0.732000F

#define LSM303D_MAG_2G_SENS  0.080F
#define LSM303D_MAG_4G_SENS  0.160F
#define LSM303D_MAG_8G_SENS  0.320F
#define LSM303D_MAG_12G_SENS 0.479F

typedef enum {
	LSM303_REGISTER_READ_ERROR=0x00,
	LSM303_REGISTER_READ_OK=0x01,
	LSM303_SENSOR_CONNECTION_ERROR=0x02,
	LSM303_SENSOR_CONNECTION_OK=0x03
} LSM303D_ErrorTypeDef;

float LSM303D_Acc_Sens;
float LSM303D_Mag_Sens;

#ifndef C_SIMULATION
LSM303D_ErrorTypeDef LSM303D_CheckDevice(LSM303D *dev);
#ifdef DEBUG_EN
LSM303D_ErrorTypeDef LSM303D_Init(LSM303D *dev,  I2C_HandleTypeDef *i2cHandle, uint8_t* errHandling);
#else
LSM303D_ErrorTypeDef LSM303D_Init(LSM303D *dev,  I2C_HandleTypeDef *i2cHandle);
#endif
HAL_StatusTypeDef LSM303D_ReadRegister(LSM303D *dev, uint8_t reg, uint8_t *data);

HAL_StatusTypeDef LSM303D_WriteRegister(LSM303D *dev, uint8_t reg, uint8_t *data);

LSM303D_ErrorTypeDef LSM303D_ReadAcc(LSM303D *dev);
LSM303D_ErrorTypeDef LSM303D_ReadMag(LSM303D *dev);

/*********CALIBRATION************/
/*Structure to handle calibrated sensor values and calibration coefficient*/
#define BIAS_ROW	3
#define BIAS_COL	1
#define OFFSET_ROW	3
#define OFFSET_COL	3

const static float MagBias[BIAS_ROW][BIAS_COL] =
					{ 0 };
const static float MagOffset[OFFSET_ROW][OFFSET_COL] =
					{ 0 };
const static float AccOffset[OFFSET_ROW][OFFSET_COL] =
					{	{1.028952 ,-0.001601 ,0.001006	},
						{-0.001601 , 0.986407 , 0.000016},
						{0.001006, 0.000016 , 0.995411	}};
const static float AccBias[BIAS_ROW][BIAS_COL] =
					{	{-0.007601}, {-0.006302}, {0.062237}};

typedef enum{
	CAL_SUCCESS 	= 0,
	CAL_PARAMS_ERR 	= 1,
	CAL_MATH_ERR 	= 2,
	CAL_FAILED 		= 3
}LSM303D_CalStateTypedef;

typedef struct{
	float SensorRead[3][1];
	arm_matrix_instance_f32 BM;	//Bias
	arm_matrix_instance_f32 OM;	//Offset
	arm_matrix_instance_f32 RM;	//Raw
	arm_matrix_instance_f32 CM;	//Output
}LSM303D_Calibrated;

/*Simple algorithm to calibrate accelerometer and magnetometer*/
/*For expert calibration use Matlab or Python code*/
LSM303D_CalStateTypedef LSM303D_SimpleCalibrateAcc(LSM303D* dev, LSM303D_Calibrated* cal);
LSM303D_CalStateTypedef LSM303D_SimpleCalibrateMag(LSM303D* dev, LSM303D_Calibrated* cal);
/*Read accelerometer and magnetometer then apply calibration*/
LSM303D_CalStateTypedef LSM303D_GetCalibratedAcc(LSM303D* dev, LSM303D_Calibrated* cal);
LSM303D_CalStateTypedef LSM303D_GetCalibratedMag(LSM303D* dev, LSM303D_Calibrated* cal);

#endif
#endif /* L303D_LSM303D_H_ */
