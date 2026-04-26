#ifndef __DHT11_H
#define __DHT11_H

#include "main.h"

// DHT11 GPIO Define
#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_0

typedef struct {
    uint8_t temp;
    uint8_t humi;
} DHT11_Data_t;

uint8_t DHT11_Init(void);
uint8_t DHT11_Read_Data(DHT11_Data_t *DHT11_Data);

#endif
