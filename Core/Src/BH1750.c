#include "BH1750.h"
#include "i2c.h"

/**
  * @brief BH1750 Initialization
  */
void BH1750_Init(void)
{
    uint8_t cmd = 0x01; // Power on
    if (HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDRESS, &cmd, 1, 10) != HAL_OK)
    {
        I2C1_Bus_Clear();
        MX_I2C1_Init();
    }
    HAL_Delay(10);
    cmd = 0x10; // H-Resolution Mode
    if (HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDRESS, &cmd, 1, 10) != HAL_OK)
    {
        I2C1_Bus_Clear();
        MX_I2C1_Init();
    }
}

/**
  * @brief Read light intensity from BH1750
  * @retval Light intensity in Lux
  */
float BH1750_ReadLux(void)
{
    uint8_t buf[2];
    uint16_t raw;
    float lux;

    if (HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDRESS, buf, 2, 10) == HAL_OK)
    {
        raw = (buf[0] << 8) | buf[1];
        lux = (float)raw / 1.2f;
        return lux;
    }
    else
    {
        I2C1_Bus_Clear();
        MX_I2C1_Init();
        return -1.0f;
    }
}
