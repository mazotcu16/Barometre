/*
 * Barometre.h
 *
 *  Created on: Sep 11, 2024
 *      Author: Furkan Gundogdu
 */

#ifndef BAROMETRE_BMP180_BMP180_H_
#define BAROMETRE_BMP180_BMP180_H_

#include "BMP180/BMP180.h"


extern I2C_HandleTypeDef hi2c2;

#define I2C2_HANDLE   &hi2c2


//typedef struct
//{
//	BMP180_dev_t barometre;
//}barometre_t;


//uint8_t barometre_baslat(barometre_t *barometre);
//void  barometre_veri_oku(barometre_t *barometre);

/******************************************************************** */

/* Barometre okuma değerlerini tutan yapı 2 */
typedef struct
{
    float temperature;  // Sıcaklık (°C)
    float pressure;  // Basınç (Pa)
} Barometre;

/* Barometre sensörünü başlatma fonksiyonu */
void Barometre_Init(void);

/* Barometreden okunan değerleri alma fonksiyonu */
void Barometre_Read(Barometre *barometre_deger);








#endif /* BAROMETRE_BMP180_BMP180_H_ */
