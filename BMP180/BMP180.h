/*
 * BMP180.h
 *
 *  Created on: 07.10.2024
 *      Author: Furkan Gundogdu
 */


#ifndef BAROMETRE_BMP180_H_
#define BAROMETRE_BMP180_H_



#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define BMP180_I2C_ADDR											(0x77U)
#define BMP180_DEVICE_ADDR_WRITE								(0xEE)
#define BMP180_DEVICE_ADDR_READ									(0xEF)


#define BMP180_OK												(0x0)
#define BMP180_NOK												(0x1)

#define BMP180_CALIBRATION_VALUE_LENGTH							(22)

#define BMP180_CALIBRATION_VALUES_START_ADDR				    (0xAA)


#define BMP180_CTRL_REG_ADDR									(0xF4)


/* Commands */
#define	BMP180_COMMAND_TEMPERATURE 					(0x2E)
#define	BMP180_COMMAND_PRESSURE_0 					(0x34)
#define	BMP180_COMMAND_PRESSURE_1 					(0x74)
#define	BMP180_COMMAND_PRESSURE_2 					(0xB4)
#define	BMP180_COMMAND_PRESSURE_3 					(0xF4)


/* Minimum waiting delay, in milliseconds */
#define BMP180_TEMPERATURE_DELAY					(5)
#define BMP180_PRESSURE_0_DELAY						(5)
#define BMP180_PRESSURE_1_DELAY						(8)
#define BMP180_PRESSURE_2_DELAY						(13)
#define BMP180_PRESSURE_3_DELAY						(26)



/* Multiple is faster than divide */
#define BMP180_1_16     					((float) 0.0625)
#define BMP180_1_256   						((float) 0.00390625)
#define BMP180_1_2048   					((float) 0.00048828125)
#define BMP180_1_4096   					((float) 0.000244140625)
#define BMP180_1_8192   					((float) 0.0001220703125)
#define BMP180_1_32768  					((float) 0.000030517578125)
#define BMP180_1_65536  					((float) 0.0000152587890625)
#define BMP180_1_101325 					((float) 0.00000986923266726)


/* 				Calibration Coefficients TypeDef Structure											    */


typedef struct
{
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
}CalibrationValues_t;

typedef enum
{
	BMP180_Oversampling_UltraLowPower = 0x00,

	BMP180_Oversampling_Standard = 0x01,

	BMP180_Oversampling_HighResolution = 0x02,

	BMP180_Oversampling_UltraHighResolution = 0x03
} BMP180_OverSampling_t;



typedef struct
{
I2C_HandleTypeDef *I2CHandle;

int16_t uc_tempValue;

float temperatureVal;

uint32_t pressure;

CalibrationValues_t *calibrationValues;

BMP180_OverSampling_t overSampling;

uint16_t Delay;

float altitude;
}BMP180_dev_t;


typedef struct
{
	uint8_t onhz;

	uint8_t yirmibeshz;

	uint8_t kirkhz;

	uint8_t ellihz;

	uint8_t seksenhz;
}Sayaclar_t;


/* 		Initialise and read calibration values function 		*/
uint8_t BMP180_Initialise(BMP180_dev_t *dev,I2C_HandleTypeDef *I2CHandle);
void BMP180_Get_Calibration_Value(BMP180_dev_t *dev);

/* 		Data Acquisition Functions 								*/
HAL_StatusTypeDef Read_Register(BMP180_dev_t *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef Write_Register(BMP180_dev_t *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef Read_Registers(BMP180_dev_t *dev, uint8_t reg, uint8_t *data,uint8_t lenght);

/* 		Calculate and Read Functions							*/
void read_uncompensated_pressure_value(BMP180_dev_t *dev,BMP180_OverSampling_t overSampling);
void read_uncompensated_Temperature(BMP180_dev_t *dev);
void calculate_real_temperature(BMP180_dev_t *dev);
void calculate_real_pressure(BMP180_dev_t *dev, BMP180_OverSampling_t overSampling);

/* 	    Calculate and Read All Values 							*/
void Read_BMP180_Data(BMP180_dev_t *dev,BMP180_OverSampling_t overSampling);

#endif /* INC_BMP180_H_ */









#endif /* BAROMETRE_BMP180_BMP180_H_ */
