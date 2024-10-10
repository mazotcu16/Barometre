/*
 * BMP180.c
 *
 *  Created on: 07.10.2024
 *      Author: Furkan Gundogdu
 */

#include "BMP180.h"
#include <math.h>

/* EEPROM values */
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD ;
uint16_t AC4, AC5, AC6, UT;
/* OK */
int32_t X1, X2, X3, B3, B5, B6, T, p;
uint32_t B4, B7, UP;



extern I2C_HandleTypeDef hi2c2;


void BMP180_Get_Calibration_Value(BMP180_dev_t *dev)
{
	uint8_t calibration_Values[BMP180_CALIBRATION_VALUE_LENGTH] = {0};
	Read_Registers(dev, BMP180_CALIBRATION_VALUES_START_ADDR, calibration_Values, BMP180_CALIBRATION_VALUE_LENGTH);

	 AC1 = 	 (int16_t)((calibration_Values[0]<<8)   | calibration_Values[1]);
	 AC2 =	 (int16_t)((calibration_Values[2]<< 8)  | calibration_Values[3]);
	 AC3 = 	 (int16_t)((calibration_Values[4]<<8)   | calibration_Values[5]);
	 AC4 =	 (uint16_t)((calibration_Values[6]<<8)  | calibration_Values[7]);
	 AC5 =   (uint16_t)((calibration_Values[8]<<8)  | calibration_Values[9]);
	 AC6 =   (uint16_t)((calibration_Values[10]<<8) | calibration_Values[11]);
	 B1  =   (int16_t)((calibration_Values[12]<<8)  | calibration_Values[13]);
	 B2  =   (int16_t)((calibration_Values[14]<<8)  | calibration_Values[15]);
	 MB  =   (int16_t)((calibration_Values[16]<<8)  | calibration_Values[17]);
	 MC  =   (int16_t)((calibration_Values[18]<<8)  | calibration_Values[19]);
	 MD  =   (int16_t)((calibration_Values[20]<<8)  | calibration_Values[21]);

	 dev->calibrationValues->AC1 = AC1;
	 dev->calibrationValues->AC2 = AC2;
	 dev->calibrationValues->AC3 = AC3;
	 dev->calibrationValues->AC4 = AC4;
	 dev->calibrationValues->AC5 = AC5;
	 dev->calibrationValues->AC6 = AC6;
	 dev->calibrationValues->B1  = B1;
	 dev->calibrationValues->B2  = B2;
	 dev->calibrationValues->MB  = MB;
	 dev->calibrationValues->MC  = MC;
	 dev->calibrationValues->MD  = MD;
}



uint8_t BMP180_Initialise(BMP180_dev_t *dev,I2C_HandleTypeDef *I2CHandle)
{
	dev->I2CHandle = I2CHandle;
	if( (HAL_I2C_IsDeviceReady(&hi2c2, BMP180_DEVICE_ADDR_WRITE, 1, 100)) != HAL_OK)
	{
	return  BMP180_NOK;
	}

	BMP180_Get_Calibration_Value(dev);

	return BMP180_OK;
}


HAL_StatusTypeDef Read_Register(BMP180_dev_t *dev, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status;

	return HAL_I2C_Mem_Read(dev->I2CHandle, BMP180_DEVICE_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);

	return status;
}

HAL_StatusTypeDef Read_Registers(BMP180_dev_t *dev, uint8_t reg, uint8_t *data,uint8_t lenght)
{
	HAL_StatusTypeDef status;

	return HAL_I2C_Mem_Read(dev->I2CHandle, BMP180_DEVICE_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, 100);

	return status;
}


HAL_StatusTypeDef Write_Register(BMP180_dev_t *dev, uint8_t reg, uint8_t *data)
{
	 return HAL_I2C_Mem_Write(dev->I2CHandle, BMP180_DEVICE_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}



void read_uncompensated_Temperature(BMP180_dev_t *dev)
{
	uint8_t datatowrite[1];
	uint8_t data[2]  = {0};
	datatowrite[0]   = BMP180_COMMAND_TEMPERATURE;


	//Send to Command Code (0x2E) to Control Register (0xF4)
    Write_Register(dev, BMP180_CTRL_REG_ADDR, datatowrite);
	HAL_Delay(5);

	//Read Data from Sensor
    HAL_I2C_Mem_Read(&hi2c2, BMP180_DEVICE_ADDR_READ, 0xF6, I2C_MEMADD_SIZE_8BIT, data, 2, 100);

	UT  = ((data[0] << 8) | (data[1]));
	dev->uc_tempValue = UT;
}

void calculate_real_temperature(BMP180_dev_t *dev)
{
	/* Calculate Temperature *C	*/
	if(UT != 0)
	{
	X1 = (UT - AC6) * AC5  * BMP180_1_32768;
	X2 = MC * 2048 / (X1 + MD);
	B5 = X1 + X2;
	}
	dev->temperatureVal = (B5 + 8) / ((float)160);

}


void read_uncompensated_pressure_value(BMP180_dev_t *dev,BMP180_OverSampling_t overSampling)
{
	uint8_t command = 0;
	uint8_t data[3];

	switch (overSampling)
	{
		case BMP180_Oversampling_UltraLowPower:
			command = BMP180_COMMAND_PRESSURE_0;
			dev->Delay = BMP180_PRESSURE_0_DELAY;
			break;
		case BMP180_Oversampling_Standard:
			command = BMP180_COMMAND_PRESSURE_1;
			dev->Delay = BMP180_PRESSURE_1_DELAY;
			break;
		case BMP180_Oversampling_HighResolution:
			command = BMP180_COMMAND_PRESSURE_2;
			dev->Delay = BMP180_PRESSURE_2_DELAY;
			break;
		case BMP180_Oversampling_UltraHighResolution:
			command = BMP180_COMMAND_PRESSURE_3;
			dev->Delay = BMP180_PRESSURE_3_DELAY;
			break;
		default:
			command = BMP180_COMMAND_PRESSURE_0;
			dev->Delay = BMP180_PRESSURE_0_DELAY;
			break;
	}
	dev->overSampling = overSampling;
	Write_Register(dev, BMP180_CTRL_REG_ADDR, &command);

    Read_Registers(dev, 0xF6, data, 3);
	HAL_Delay(dev->Delay);

	UP = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2]) >> (8 - (dev->overSampling));
}


/* Calculate the actual pressure in Pa */
void calculate_real_pressure(BMP180_dev_t *dev, BMP180_OverSampling_t overSampling)
{
    // Integer arithmetic as per the datasheet
    B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << overSampling) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (50000 >> overSampling);

    if(B4!= 0)
    {
    if (B7 < 0x80000000)
    {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);
    }
    dev->pressure = p;  // Pressure in Pa

}




void Read_BMP180_Data(BMP180_dev_t *dev,BMP180_OverSampling_t overSampling)
{
	read_uncompensated_Temperature(dev);
    read_uncompensated_pressure_value(dev,overSampling);
    calculate_real_temperature(dev);
    calculate_real_pressure(dev,overSampling);
}











