#include "OLED.h"
#include "i2c.h"
#include "oledfont.h"

#define OLED_ADDRESS 0x78

static uint8_t oled_error_flag = 0;

static void OLED_WriteCommand(uint8_t cmd)
{
    if (oled_error_flag) return; // Skip if already in error state for this cycle

    uint8_t buf[2];
    buf[0] = 0x00;
    buf[1] = cmd;
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buf, 2, 10) != HAL_OK)
    {
        oled_error_flag = 1;
        I2C1_Bus_Clear();
        MX_I2C1_Init();
    }
}

static uint8_t OLED_ReverseByte(uint8_t byte)
{
    byte = ((byte & 0x55) << 1) | ((byte & 0xAA) >> 1);
    byte = ((byte & 0x33) << 2) | ((byte & 0xCC) >> 2);
    byte = ((byte & 0x0F) << 4) | ((byte & 0xF0) >> 4);
    return byte;
}

void OLED_SetPos(uint8_t x, uint8_t y)
{
    OLED_WriteCommand(0xb0 + y);
    OLED_WriteCommand(((x & 0xf0) >> 4) | 0x10);
    OLED_WriteCommand((x & 0x0f) | 0x01);
}

void OLED_DisplayOn(void)
{
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);
}

void OLED_DisplayOff(void)
{
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x10);
    OLED_WriteCommand(0xAE);
}

void OLED_Clear(void)
{
    if (oled_error_flag) return;

    uint8_t i, n;
    uint8_t buf[129];
    buf[0] = 0x40;
    for (n = 1; n < 129; n++) buf[n] = 0;
    for (i = 0; i < 8; i++)
    {
        OLED_WriteCommand(0xb0 + i);
        OLED_WriteCommand(0x00);
        OLED_WriteCommand(0x10);
        
        if (oled_error_flag) return;

        if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buf, 129, 50) != HAL_OK)
        {
            oled_error_flag = 1;
            I2C1_Bus_Clear();
            MX_I2C1_Init();
            return;
        }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size)
{
    if (oled_error_flag) return;

    uint8_t c = chr - ' ';
    uint8_t i;
    uint8_t buf[9]; // 1 control byte + 8 data bytes
    
    if (x > 128 - (size / 2)) { x = 0; y += 2; }
    
    if (size == 16)
    {
        // Page 0 (Upper part)
        OLED_SetPos(x, y);
        if (oled_error_flag) return;

        buf[0] = 0x40; // Data mode
        for (i = 0; i < 8; i++) buf[i + 1] = OLED_ReverseByte(asc2_1608[c][i * 2]);
        if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buf, 9, 10) != HAL_OK)
        {
            oled_error_flag = 1;
            I2C1_Bus_Clear();
            MX_I2C1_Init();
            return;
        }

        // Page 1 (Lower part)
        OLED_SetPos(x, y + 1);
        if (oled_error_flag) return;

        buf[0] = 0x40; // Data mode
        for (i = 0; i < 8; i++) buf[i + 1] = OLED_ReverseByte(asc2_1608[c][i * 2 + 1]);
        if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buf, 9, 10) != HAL_OK)
        {
            oled_error_flag = 1;
            I2C1_Bus_Clear();
            MX_I2C1_Init();
            return;
        }
    }
}

// Add a function to reset the error flag and potentially re-init OLED
void OLED_StartRefresh(void)
{
    if (oled_error_flag)
    {
        // If there was an error, try a full re-init
        OLED_Init(); 
        oled_error_flag = 0;
    }
}


void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t size)
{
    while (*chr != '\0')
    {
        if (x > 128 - (size / 2)) { x = 0; y += 2; }
        OLED_ShowChar(x, y, *chr, size);
        x += (size / 2);
        chr++;
    }
}

void OLED_Init(void)
{
    HAL_Delay(100);
    OLED_WriteCommand(0xAE); // Turn off OLED panel
    OLED_WriteCommand(0x00); // Set low column address
    OLED_WriteCommand(0x10); // Set high column address
    OLED_WriteCommand(0x40); // Set start line address
    OLED_WriteCommand(0x81); // Set contrast control register
    OLED_WriteCommand(0xCF); // Set SEG Output Current Brightness
    OLED_WriteCommand(0xA1); // Set SEG/Column Mapping
    OLED_WriteCommand(0xC8); // Set COM/Row Scan Direction
    OLED_WriteCommand(0xA6); // Set normal display
    OLED_WriteCommand(0xA8); // Set multiplex ratio
    OLED_WriteCommand(0x3F); // 1/64 duty
    OLED_WriteCommand(0xD3); // Set display offset
    OLED_WriteCommand(0x00); // Not offset
    OLED_WriteCommand(0xD5); // Set display clock divide ratio/oscillator frequency
    OLED_WriteCommand(0x80); // Set divide ratio
    OLED_WriteCommand(0xD9); // Set pre-charge period
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDA); // Set com pins hardware configuration
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0xDB); // Set vcomh
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x20); // Set Memory Addressing Mode
    OLED_WriteCommand(0x02); // Page Addressing Mode
    OLED_WriteCommand(0x8D); // Set Charge Pump enable/disable
    OLED_WriteCommand(0x14); // Set(0x10) disable
    OLED_WriteCommand(0xA4); // Disable Entire Display On
    OLED_WriteCommand(0xA6); // Disable Inverse Display On
    OLED_WriteCommand(0xAF); // Turn on OLED panel
    OLED_Clear();
}

static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size);
                continue;
            }
            else enshow = 1;
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size);
    }
}

/**
  * @brief Show float number on OLED
  * @param x,y: coordinates
  * @param num: float number
  * @param int_len: length of integer part
  * @param dec_len: length of decimal part
  * @param size: font size
  */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size)
{
    uint32_t pow10 = 1;
    for (uint8_t i = 0; i < dec_len; i++) pow10 *= 10;
    
    uint32_t temp_int = (uint32_t)num;
    uint32_t temp_dec = (uint32_t)((num - temp_int) * pow10 + 0.5f); // +0.5 for rounding
    
    OLED_ShowNum(x, y, temp_int, int_len, size);
    OLED_ShowChar(x + (size / 2) * int_len, y, '.', size);
    OLED_ShowNum(x + (size / 2) * (int_len + 1), y, temp_dec, dec_len, size);
}
