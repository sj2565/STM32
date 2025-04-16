/*
 * dht22.h
 *
 *  Created on: Mar 23, 2025
 *      Author: seojoon
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include "main.h"

void DHT22_Init(void);
uint8_t DHT22_Read(float *temperature, float *humidity);

#endif /* INC_DHT22_H_ */

