#ifndef DHT22_H
#define DHT22_H

#include "stm32l1xx_hal.h"

typedef enum {
    DHT22_OK = 0,
    DHT22_ERROR_TIMEOUT,
    DHT22_ERROR_CHECKSUM
} DHT22_Status;

typedef struct {
    float Temperature;
    float Humidity;
} DHT22_DataTypedef;

DHT22_Status DHT22_ReadData(DHT22_DataTypedef *data, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT22_DelayUs(uint16_t us);

#endif
