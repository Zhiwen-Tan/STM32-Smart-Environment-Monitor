/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "DHT11.h"
#include "BH1750.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    ACT_NONE = 0,
    ACT_PUMP,
    ACT_BEEP
} Actuator_t;

typedef enum {
    SENS_NONE = 0,
    SENS_TEMP,
    SENS_AIR,
    SENS_LIGHT,
    SENS_SOIL
} Sensor_t;

typedef struct {
    Actuator_t actuator;
    Sensor_t sensor;
    float low_val;
    float high_val;
    uint8_t action; // 1 for run, 0 for stop
    uint8_t in_use; // 1 if rule is active
} ControlRule_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOIL_DRY_RAW 4094.0f
#define SOIL_WET_RAW 1740.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Pump Control Macros with EMI Protection
#define PUMP_ON() do { \
    system_busy = 1; \
    HAL_Delay(200); \
    HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET); \
    pump_is_on = 1; \
    last_pump_tick = HAL_GetTick(); \
    UART_Log("[Action] Pump Turned ON (PA4 High)\r\n"); \
    HAL_Delay(200); \
    system_busy = 0; \
} while(0)

#define PUMP_OFF() do { \
    system_busy = 1; \
    HAL_Delay(200); \
    HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET); \
    pump_is_on = 0; \
    last_pump_tick = HAL_GetTick(); \
    UART_Log("[Action] Pump Turned OFF (PA4 Low)\r\n"); \
    HAL_Delay(200); \
    system_busy = 0; \
} while(0)

// Buzzer Control Macros
#define BUZZER_ON() do { \
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET); \
    buzzer_on = 1; \
} while(0)

#define BUZZER_OFF() do { \
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET); \
    buzzer_on = 0; \
} while(0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Values[3]; // PA1, PA2, PA3
float soil_humidity = 0;
float last_soil_raw = 0; // For software lag filter
DHT11_Data_t dht_data;
float bh1750_lux = 0;
uint32_t last_display_tick = 0; // For OLED refresh rate control
uint8_t heartbeat_state = 0;    // For Heartbeat indicator
uint32_t last_dht_tick = 0;     // For DHT11 sampling rate control
uint32_t last_soil_tick = 0;    // For Soil sampling rate control
uint32_t last_pump_tick = 0;    // For Relay test timing
uint8_t pump_is_on = 0;         // Pump state flag
uint8_t system_busy = 0;        // System busy flag for EMI protection
uint8_t uart_rx_char = 0;       // For manual command receiving
uint8_t buzzer_on = 0;          // Buzzer state flag
uint8_t manual_buzzer = 0;      // Manual override flag for buzzer: 0=Auto, 1=ForceON, 2=ForceOFF
uint8_t manual_pump = 0;        // Manual override flag for pump: 0=Auto, 1=ForceON, 2=ForceOFF
uint8_t cmd_ready = 0;          // Flag for command processing
#define MAX_RULES 10
#define FLASH_SAVE_ADDR  0x0800FC00
#define RULES_MAGIC      0x5A5A1234
ControlRule_t rules[MAX_RULES];

#define UART_RX_BUF_SIZE 128
char uart_rx_buffer[UART_RX_BUF_SIZE];
uint8_t uart_rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_Read_All(void);
uint16_t Get_Strong_Filtered_Soil(void);
void UART_Log(const char *fmt, ...);
void Process_Command(char *cmd);
void List_Rules(void);
void Delete_Rule(char *cmd_body);
void Delete_All_Actuator_Rules(Actuator_t act);
void Apply_Rules(void);
void Save_Rules_To_Flash(void);
void Load_Rules_From_Flash(void);
void Print_Help(void);
int strcasecmp_custom(const char *s1, const char *s2);
void Save_Rules_To_Flash(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseConfig;
    eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseConfig.PageAddress = FLASH_SAVE_ADDR;
    eraseConfig.NbPages = 1;
    uint32_t pageError = 0;
    if (HAL_FLASHEx_Erase(&eraseConfig, &pageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return;
    }

    // Save Magic Number
    uint32_t magic = RULES_MAGIC;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SAVE_ADDR, magic);

    // Save Rules as words
    uint32_t *pData = (uint32_t *)rules;
    uint32_t size_in_words = (sizeof(rules) + 3) / 4;

    for (uint32_t i = 0; i < size_in_words; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SAVE_ADDR + 4 + (i * 4), pData[i]);
    }

    HAL_FLASH_Lock();
}

void Load_Rules_From_Flash(void)
{
    uint32_t magic = *(__IO uint32_t *)FLASH_SAVE_ADDR;
    if (magic == RULES_MAGIC)
    {
        uint32_t *pData = (uint32_t *)rules;
        uint32_t size_in_words = (sizeof(rules) + 3) / 4;

        for (uint32_t i = 0; i < size_in_words; i++)
        {
            pData[i] = *(__IO uint32_t *)(FLASH_SAVE_ADDR + 4 + (i * 4));
        }
    }
}

void Print_Help(void)
{
    UART_Log("\r\n--- STM32 Smart System Help ---\r\n");
    UART_Log("1. Set Rule: [ACT] [SENS] [LOW] low [HIGH] high [ACTION]\r\n");
    UART_Log("   ACT: PUMP, BEEP\r\n");
    UART_Log("   SENS: T, Air, Light, Soil\r\n");
    UART_Log("   ACTION: 1(Run), 0(Stop)\r\n");
    UART_Log("2. Force Control: [ACT] [1/0]\r\n");
    UART_Log("   Example: PUMP 1 (ON), BEEP 0 (OFF)\r\n");
    UART_Log("3. Reset Auto: [ACT] auto\r\n");
    UART_Log("   Example: PUMP auto (Reset to Rule Mode)\r\n");
    UART_Log("4. List Rules: ls cmd\r\n");
    UART_Log("5. Delete Rule: Del [Full Rule Content]\r\n");
    UART_Log("6. Clear Actuator: Del [ACT] ALL\r\n");
    UART_Log("7. This Help: help\r\n");
    UART_Log("--------------------------------\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Log(const char *fmt, ...)
{
  char buf[128];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
  }
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * @brief Read all 3 ADC channels (PA1, PA2, PA3)
  * Robust polling for F1: Configure and sample each channel one by one
  */
void ADC_Read_All(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    // --- Channel 1 (PA1) ---
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    for(volatile int d=0; d<200; d++); // Settlement delay after mux switch
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        ADC_Values[0] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // --- Channel 2 (PA2 - Soil) ---
    sConfig.Channel = ADC_CHANNEL_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    for(volatile int d=0; d<200; d++); // Settlement delay
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        ADC_Values[1] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // --- Channel 3 (PA3) ---
    sConfig.Channel = ADC_CHANNEL_3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    for(volatile int d=0; d<200; d++); // Settlement delay
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        ADC_Values[2] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
}

/**
  * @brief Median Average Filter for Soil
  * Optimized: Reduced samples to 6 for faster response
  * 1. Collect 6 samples
  * 2. Sort samples
  * 3. Average middle 4
  * @retval Filtered ADC value
  */
uint16_t Get_Strong_Filtered_Soil(void)
{
    uint16_t samples[6];
    uint32_t sum = 0;
    
    // 1. Sampling (6 times)
    for (int i = 0; i < 6; i++)
    {
        ADC_Read_All();
        samples[i] = ADC_Values[1]; // PA2 (Soil Sensor)
        HAL_Delay(1); // Reduced delay for faster internal loop
    }
    
    // 2. Sorting (Bubble Sort)
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5 - i; j++)
        {
            if (samples[j] > samples[j + 1])
            {
                uint16_t temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }
    
    // 3. Average middle 4 samples (from index 1 to 4)
    for (int i = 1; i < 5; i++)
    {
        sum += samples[i];
    }
    
    return (uint16_t)(sum / 4);
}

int strcasecmp_custom(const char *s1, const char *s2)
{
    while (*s1 && (tolower((unsigned char)*s1) == tolower((unsigned char)*s2)))
    {
        s1++;
        s2++;
    }
    return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

void List_Rules(void)
{
    UART_Log("\r\n--- Current Threshold Rules ---\r\n");
    int count = 0;
    for (int i = 0; i < MAX_RULES; i++)
    {
        if (rules[i].in_use)
        {
            count++;
            const char *act_str = (rules[i].actuator == ACT_PUMP) ? "PUMP" : "BEEP";
            const char *sens_str = "";
            switch (rules[i].sensor)
            {
                case SENS_TEMP: sens_str = "T"; break;
                case SENS_AIR: sens_str = "Air"; break;
                case SENS_LIGHT: sens_str = "Light"; break;
                case SENS_SOIL: sens_str = "Soil"; break;
                default: sens_str = "Unknown"; break;
            }
            UART_Log("%s %s %.0f low %.0f high %d\r\n", 
                     act_str, sens_str, rules[i].low_val, rules[i].high_val, rules[i].action);
        }
    }
    if (count == 0) UART_Log("No rules set.\r\n");
    UART_Log("-------------------------------\r\n");
}

void Delete_All_Actuator_Rules(Actuator_t act)
{
    int deleted = 0;
    for (int i = 0; i < MAX_RULES; i++)
    {
        if (rules[i].in_use && rules[i].actuator == act)
        {
            rules[i].in_use = 0;
            deleted++;
        }
    }
    UART_Log("Deleted %d rules for %s.\r\n", deleted, (act == ACT_PUMP) ? "PUMP" : "BEEP");
    if (deleted > 0)
    {
        Save_Rules_To_Flash();
        List_Rules();
    }
}

void Process_Command(char *cmd)
{
    // Echo the original string
    UART_Log("Echo: %s\r\n", cmd);

    // Remove trailing \r or \n
    char *p = cmd + strlen(cmd) - 1;
    while (p >= cmd && (*p == '\r' || *p == '\n' || *p == ' '))
    {
        *p = '\0';
        p--;
    }

    if (strlen(cmd) == 0) return;

    char cmd_copy[128];
    strncpy(cmd_copy, cmd, sizeof(cmd_copy));
    char *tokens[10];
    int token_count = 0;
    
    char *token = strtok(cmd, " ");
    while (token != NULL && token_count < 10)
    {
        tokens[token_count++] = token;
        token = strtok(NULL, " ");
    }

    if (token_count == 0) return;

    if (strcasecmp_custom(tokens[0], "ls") == 0 && token_count >= 2 && strcasecmp_custom(tokens[1], "cmd") == 0)
    {
        List_Rules();
        return;
    }

    if (strcasecmp_custom(tokens[0], "help") == 0)
    {
        Print_Help();
        return;
    }

    // Manual Force Control
    if (token_count == 2)
    {
        if (strcasecmp_custom(tokens[0], "BEEP") == 0)
        {
            if (strcmp(tokens[1], "1") == 0) { 
                manual_buzzer = 1; 
                UART_Log("BEEP Forced ON (Ignoring Sensors)\r\n"); 
            }
            else if (strcmp(tokens[1], "0") == 0) { 
                manual_buzzer = 2; 
                UART_Log("BEEP Forced OFF (Ignoring Sensors)\r\n"); 
            }
            else if (strcasecmp_custom(tokens[1], "auto") == 0) { 
                manual_buzzer = 0; 
                UART_Log("BEEP Back to Auto Mode\r\n"); 
            }
            return;
        }
        else if (strcasecmp_custom(tokens[0], "PUMP") == 0)
        {
            if (strcmp(tokens[1], "1") == 0) { 
                manual_pump = 1; 
                UART_Log("PUMP Forced ON (Ignoring Sensors)\r\n"); 
            }
            else if (strcmp(tokens[1], "0") == 0) { 
                manual_pump = 2; 
                UART_Log("PUMP Forced OFF (Ignoring Sensors)\r\n"); 
            }
            else if (strcasecmp_custom(tokens[1], "auto") == 0) { 
                manual_pump = 0; 
                UART_Log("PUMP Back to Auto Mode\r\n"); 
            }
            return;
        }
    }

    // Check for Del command
    if (strcasecmp_custom(tokens[0], "Del") == 0)
    {
        if (token_count == 3 && strcasecmp_custom(tokens[2], "ALL") == 0)
        {
            if (strcasecmp_custom(tokens[1], "PUMP") == 0) Delete_All_Actuator_Rules(ACT_PUMP);
            else if (strcasecmp_custom(tokens[1], "BEEP") == 0) Delete_All_Actuator_Rules(ACT_BEEP);
            else UART_Log("Invalid Actuator\r\n");
        }
        else if (token_count == 8) // Del + 7 params
        {
            // Full match delete
            Actuator_t act = ACT_NONE;
            if (strcasecmp_custom(tokens[1], "PUMP") == 0) act = ACT_PUMP;
            else if (strcasecmp_custom(tokens[1], "BEEP") == 0) act = ACT_BEEP;

            Sensor_t sens = SENS_NONE;
            if (strcasecmp_custom(tokens[2], "T") == 0) sens = SENS_TEMP;
            else if (strcasecmp_custom(tokens[2], "Air") == 0) sens = SENS_AIR;
            else if (strcasecmp_custom(tokens[2], "Light") == 0) sens = SENS_LIGHT;
            else if (strcasecmp_custom(tokens[2], "Soil") == 0) sens = SENS_SOIL;

            float low = (float)atof(tokens[3]);
            float high = (float)atof(tokens[5]);
            uint8_t action = (uint8_t)atoi(tokens[7]);

            int found = 0;
            for (int i = 0; i < MAX_RULES; i++)
            {
                if (rules[i].in_use && rules[i].actuator == act && rules[i].sensor == sens &&
                    rules[i].low_val == low && rules[i].high_val == high && rules[i].action == action)
                {
                    rules[i].in_use = 0;
                    found = 1;
                    break;
                }
            }
            if (found)
            {
                UART_Log("Rule deleted successfully.\r\n");
                Save_Rules_To_Flash();
                List_Rules();
            }
            else UART_Log("Rule not found.\r\n");
        }
        else
        {
            UART_Log("Invalid Del Format\r\n");
        }
        return;
    }

    // Check for Rule command (7 parts)
    if (token_count == 7)
    {
        Actuator_t act = ACT_NONE;
        if (strcasecmp_custom(tokens[0], "PUMP") == 0) act = ACT_PUMP;
        else if (strcasecmp_custom(tokens[0], "BEEP") == 0) act = ACT_BEEP;

        if (act == ACT_NONE) { UART_Log("Invalid Actuator\r\n"); return; }

        Sensor_t sens = SENS_NONE;
        if (strcasecmp_custom(tokens[1], "T") == 0) sens = SENS_TEMP;
        else if (strcasecmp_custom(tokens[1], "Air") == 0) sens = SENS_AIR;
        else if (strcasecmp_custom(tokens[1], "Light") == 0) sens = SENS_LIGHT;
        else if (strcasecmp_custom(tokens[1], "Soil") == 0) sens = SENS_SOIL;

        if (sens == SENS_NONE) { UART_Log("Invalid Sensor\r\n"); return; }

        float low = (float)atof(tokens[2]);
        float high = (float)atof(tokens[4]);
        uint8_t action = (uint8_t)atoi(tokens[6]);

        // Validation
        if (high <= low) { UART_Log("Invalid Range: high must be > low\r\n"); return; }
        
        // Value bounds
        if (sens == SENS_AIR || sens == SENS_SOIL) {
            if (low < 0 || high > 100) { UART_Log("Invalid Range for Air/Soil (0-100)\r\n"); return; }
        } else if (sens == SENS_TEMP) {
            if (low < 0 || high > 100) { UART_Log("Invalid Range for Temp (0-100)\r\n"); return; }
        } else if (sens == SENS_LIGHT) {
            if (low < 0 || high > 20000) { UART_Log("Invalid Range for Light (0-20000)\r\n"); return; }
        }

        // Add or Update rules
        int slot = -1;
        // First check if a rule for this actuator + sensor already exists
        for (int i = 0; i < MAX_RULES; i++)
        {
            if (rules[i].in_use && rules[i].actuator == act && rules[i].sensor == sens)
            {
                slot = i;
                break;
            }
        }

        // If not found, find an empty slot
        if (slot == -1)
        {
            for (int i = 0; i < MAX_RULES; i++)
            {
                if (!rules[i].in_use)
                {
                    slot = i;
                    break;
                }
            }
        }

        if (slot != -1)
        {
            rules[slot].actuator = act;
            rules[slot].sensor = sens;
            rules[slot].low_val = low;
            rules[slot].high_val = high;
            rules[slot].action = action;
            rules[slot].in_use = 1;
            UART_Log("Command applied (Added/Updated).\r\n");
            Save_Rules_To_Flash();
            List_Rules();
        }
        else
        {
            UART_Log("Rule library full!\r\n");
        }
    }
    else
    {
        UART_Log("Invalid Format\r\n");
    }
}

void Apply_Rules(void)
{
    uint8_t pump_should_run = 0;
    uint8_t beep_should_run = 0;

    for (int i = 0; i < MAX_RULES; i++)
    {
        if (rules[i].in_use)
        {
            float val = 0;
            switch (rules[i].sensor)
            {
                case SENS_TEMP: val = (float)dht_data.temp; break;
                case SENS_AIR: val = (float)dht_data.humi; break;
                case SENS_LIGHT: val = bh1750_lux; break;
                case SENS_SOIL: val = soil_humidity; break;
                default: continue;
            }

            // Hysteresis logic (1% deadband)
            // If already running, stay running until out of [low-1%, high+1%]
            float deadband = (rules[i].sensor == SENS_LIGHT) ? 200.0f : 1.0f; // 200 lux or 1%
            uint8_t current_actuator_on = (rules[i].actuator == ACT_PUMP) ? pump_is_on : buzzer_on;
            
            uint8_t match = 0;
            if (current_actuator_on)
            {
                if (val >= (rules[i].low_val - deadband) && val <= (rules[i].high_val + deadband))
                {
                    match = 1;
                }
            }
            else
            {
                if (val >= rules[i].low_val && val <= rules[i].high_val)
                {
                    match = 1;
                }
            }
            
            if (match)
            {
                if (rules[i].actuator == ACT_PUMP)
                {
                    if (rules[i].action == 1) pump_should_run = 1;
                }
                else if (rules[i].actuator == ACT_BEEP)
                {
                    if (rules[i].action == 1) beep_should_run = 1;
                }
            }
        }
    }

    // Execute actions
    if (manual_pump == 1) pump_should_run = 1;
    else if (manual_pump == 2) pump_should_run = 0;

    if (manual_buzzer == 1) beep_should_run = 1;
    else if (manual_buzzer == 2) beep_should_run = 0;

    if (pump_should_run)
    {
        if (!pump_is_on) PUMP_ON();
    }
    else
    {
        if (pump_is_on) PUMP_OFF();
    }

    if (beep_should_run)
    {
        if (!buzzer_on) BUZZER_ON();
    }
    else
    {
        if (buzzer_on) BUZZER_OFF();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (uart_rx_char == '\n' || uart_rx_char == '\r')
        {
            if (uart_rx_index > 0)
            {
                uart_rx_buffer[uart_rx_index] = '\0';
                cmd_ready = 1;
            }
        }
        else if (uart_rx_index < UART_RX_BUF_SIZE - 1)
        {
            uart_rx_buffer[uart_rx_index++] = (char)uart_rx_char;
        }
        HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Earliest possible reset for Pump (PA4) to prevent power surge
  __HAL_RCC_GPIOA_CLK_ENABLE(); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
  BH1750_Init();
  
  // Load saved rules from Flash
  Load_Rules_From_Flash();
  
  // Start UART Interrupt Reception
  HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
  
  // ADC Calibration for F1 series
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1); 
  
  // 1. Power-on Protection: Force Pump OFF (PA4 Low)
  HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
  OLED_ShowString(0, 0, "System Init...", 16);
  OLED_ShowString(0, 2, "Pump Protected", 16);
  HAL_Delay(500); // Short delay for user to see the message
  
  OLED_Clear();
  OLED_ShowString(0, 0, "System Start", 16);
  
  UART_Log("\r\n=================================");
  UART_Log("\r\nSmart Irrigation System Started");
  UART_Log("\r\nBuild Date: %s %s", __DATE__, __TIME__);
  UART_Log("\r\nUSART1: 115200 Baudrate Initialized");
  UART_Log("\r\nManual Control: '0'->All OFF, '1'->Pump ON, '2'->Beep ON, '3'->Auto Mode");
  UART_Log("\r\nThresholds: Temp>40C, Light>3000, Air_Humi>70%%");
  UART_Log("\r\n=================================\r\n");
  
  HAL_Delay(500);
  OLED_Clear();
  
  last_pump_tick = HAL_GetTick();
  last_soil_tick = HAL_GetTick();
  last_dht_tick = HAL_GetTick() - 500; // Offset DHT11 sampling to avoid collision with ADC

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 0. Command Handling (Check Serial Port Flag)
    if (cmd_ready)
    {
        Process_Command(uart_rx_buffer);
        uart_rx_index = 0;
        cmd_ready = 0;
    }

    // 1. Get Median Filtered ADC Value from PA2 (Sampling every 1s) - Only when system is not busy
    if (system_busy == 0 && (HAL_GetTick() - last_soil_tick >= 1000))
    {
        last_soil_tick = HAL_GetTick();
        uint16_t current_soil_raw = Get_Strong_Filtered_Soil();
        
        // 2. Software Lag Filter: Final = Last * 0.7 + Current * 0.3 (Faster Response)
        if (last_soil_raw == 0) last_soil_raw = (float)current_soil_raw;
        float soil_raw_f = (last_soil_raw * 0.7f) + ((float)current_soil_raw * 0.3f);
        last_soil_raw = soil_raw_f;
        
        uint16_t soil_raw = (uint16_t)soil_raw_f;
        
        // 4. Software Noise Cancellation
        if (soil_raw < 100) soil_raw = 0;
        
        // 5. Convert to Percentage using Adjusted Calibration Range
        soil_humidity = (SOIL_DRY_RAW - (float)soil_raw) * 100.0f / (SOIL_DRY_RAW - SOIL_WET_RAW);
        
        // 6. Locking 100% and 0% Logic
        if (soil_humidity < 0.0f) soil_humidity = 0.0f;
        if (soil_humidity > 100.0f) soil_humidity = 100.0f;
        
        // UART_Log("[Log] Soil Update: Raw=%u, Soil_Humi=%.1f%%\r\n", soil_raw, soil_humidity);
    }
    
    // 3. Read other sensors (DHT11 every 2s) - Use system_busy to prevent ADC interference
    if (system_busy == 0 && (HAL_GetTick() - last_dht_tick >= 2000))
    {
        system_busy = 1; // Lock system to prevent ADC crosstalk during digital/I2C comm
        HAL_Delay(5);    // Short stabilization delay
        
        last_dht_tick = HAL_GetTick();
        int8_t status = DHT11_Read_Data(&dht_data);
        
        float current_lux = BH1750_ReadLux();
        if (current_lux < 0 && bh1750_lux >= 0) // Just lost connection
        {
             BH1750_Init(); // Try to re-init sensor
        }
        else if (current_lux >= 0)
        {
             bh1750_lux = current_lux;
        }
        
        system_busy = 0; // Unlock
        
        // UART_Log("[Log] Sensor Update: Temp=%d C, Air_Humi=%d %%, Light=%.1f Lux, DHT11_Status=%d\r\n", 
        //        dht_data.temp, dht_data.humi, bh1750_lux, status);
               
        // 9. Rule-based Control Logic
        if (manual_buzzer == 0) // Keep this flag if you want a global "mute" but rules usually handle it
        {
            Apply_Rules();
        }
    }
    
    // 7. Update OLED Display (Limited to once per second: 1000ms) - Only when system is not busy
    if (system_busy == 0 && (HAL_GetTick() - last_display_tick >= 1000))
    {
        last_display_tick = HAL_GetTick();
        
        system_busy = 1; // Protect OLED I2C transmission
        OLED_StartRefresh(); // Reset error flag and re-init if needed
        
        // Row 1: Temperature & Air Humidity
        OLED_ShowString(0, 0, "T:", 16);
        OLED_ShowNum(16, 0, dht_data.temp, 2, 16);
        OLED_ShowString(32, 0, "C Air:", 16);
        OLED_ShowNum(80, 0, dht_data.humi, 2, 16);
        OLED_ShowString(96, 0, "%", 16);
        
        // Row 2: Light Intensity (Lux)
        OLED_ShowString(0, 2, "Light:", 16);
        OLED_ShowNum(48, 2, (uint32_t)bh1750_lux, 5, 16);
        OLED_ShowString(96, 2, "Lux", 16);
        
        // Row 3: Soil Humidity
        OLED_ShowString(0, 4, "Soil Humi:", 16);
        OLED_ShowNum(80, 4, (uint32_t)soil_humidity, 3, 16);
        OLED_ShowString(104, 4, "%", 16);
        
        // Row 4: Pump & Buzzer Status
        if (HAL_GetTick() < 5000)
        {
            OLED_ShowString(0, 6, "Wait 5s...      ", 16);
        }
        else
        {
            if (pump_is_on) OLED_ShowString(0, 6, "P:ON    ", 16);
            else            OLED_ShowString(0, 6, "P:OFF   ", 16);
            
            if (buzzer_on)  OLED_ShowString(64, 6, "B:ON    ", 16);
            else            OLED_ShowString(64, 6, "B:OFF   ", 16);
        }
        
        // Heartbeat Indicator: A blinking dot at the bottom right
        heartbeat_state = !heartbeat_state;
        if (heartbeat_state) OLED_ShowString(120, 6, ".", 16);
        else OLED_ShowString(120, 6, " ", 16);
        
        system_busy = 0; // Unlock
    }
    
    // 8. Wait for Startup delay
    if (HAL_GetTick() < 5000)
    {
        // Keep pump off during startup delay
        HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
        last_pump_tick = HAL_GetTick();
    }
    
    HAL_Delay(10); // Small delay to prevent tight loop
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
