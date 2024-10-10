/*
 * Barometre.c
 *
 *  Created on: 07.10.2024
 *      Author: Furkan Gundogdu
 */

#include "Barometre.h"

//Barometre barometre_degerler;
static BMP180_dev_t bmp180_dev;


void Barometre_Init(void)
{
    /* BMP180 sensörünü başlat */
    BMP180_Initialise(&bmp180_dev,I2C2_HANDLE);
}

void Barometre_Read(Barometre *barometre_deger)
{
    /* BMP180 sensöründen verileri oku */
    Read_BMP180_Data(&bmp180_dev, 0);								 //Ultra Low Power Mode

    barometre_deger->temperature = (bmp180_dev.temperatureVal);  	 // Sıcaklık değeri
    barometre_deger->pressure = bmp180_dev.pressure;           		 // Basınç değeri
}




















