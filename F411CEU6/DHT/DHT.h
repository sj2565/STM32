/*
 * DHT.h
 *
 *  Created on: Mar 24, 2025
 *      Author: seojoon
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_

#ifndef DHT_H_
#define DHT_H_

typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypedef;


void DHT_GetData (DHT_DataTypedef *DHT_Data);

#endif /* INC_DHT_H_ */

#endif /* INC_DHT_H_ */
