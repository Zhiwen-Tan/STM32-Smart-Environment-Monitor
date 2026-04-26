#include "DHT11.h"

// Microsecond delay function for 72MHz STM32F103
static void Delay_us(uint32_t us)
{
    uint32_t delay = us * 8; // Approx 72MHz / 9 instructions per loop
    while(delay--);
}

// Set PA0 as Output
static void DHT11_Mode_Out(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

// Set PA0 as Input
static void DHT11_Mode_In(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

// Send Start Signal
static void DHT11_Start(void)
{
    DHT11_Mode_Out();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(20); // Pull down at least 18ms
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    Delay_us(30);  // Pull up 20-40us
}

// Check Response
static uint8_t DHT11_Check(void)
{
    uint8_t retry = 0;
    DHT11_Mode_In();
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if(retry >= 100) return 1;
    else retry = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if(retry >= 100) return 1;
    return 0;
}

// Read one bit with Timeout
static uint8_t DHT11_Read_Bit(void)
{
    uint8_t retry = 0;
    // Wait for bit start (low level)
    while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if(retry >= 100) return 0xFF; // Timeout error

    retry = 0;
    // Wait for high level (start of pulse)
    while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if(retry >= 100) return 0xFF; // Timeout error

    Delay_us(40); // 26-28us is '0', 70us is '1'
    if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) return 1;
    else return 0;
}

// Read one byte with Timeout
static int16_t DHT11_Read_Byte(void)
{
    uint8_t i;
    uint8_t dat = 0;
    for (i = 0; i < 8; i++)
    {
        uint8_t bit = DHT11_Read_Bit();
        if(bit == 0xFF) return -1; // Propagate timeout error
        dat <<= 1;
        dat |= bit;
    }
    return (int16_t)dat;
}

// Initialize DHT11
uint8_t DHT11_Init(void)
{
    DHT11_Start();
    return DHT11_Check();
}

// Read Data with Error Handling
uint8_t DHT11_Read_Data(DHT11_Data_t *DHT11_Data)
{
    uint8_t buf[5];
    uint8_t i;
    DHT11_Start();
    if(DHT11_Check() == 0)
    {
        for(i = 0; i < 5; i++)
        {
            int16_t byte = DHT11_Read_Byte();
            if(byte == -1) return 1; // Timeout during reading
            buf[i] = (uint8_t)byte;
        }
        if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            DHT11_Data->humi = buf[0];
            DHT11_Data->temp = buf[2];
            return 0; // Success
        }
    }
    return 1; // Failure
}
