#include "main.h"
#include "cmsis_os.h"
#include "dht22.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_10
#define LED_GPIO_PORT GPIOA
#define LED_GPIO_PIN  GPIO_PIN_8


extern void initialise_monitor_handles(void);  // protótipo da função

extern osMutexId dataMutexHandle;

ADC_HandleTypeDef hadc;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId sensorTaskHandle;
osThreadId alarmeTaskHandle;
osThreadId loggerTaskHandle;
osThreadId cliTaskHandle;

DHT22_DataTypedef DHT22_Data;
float ph = 7.0f;
uint8_t led_status = 0;


osMutexId dataMutexHandle;
osMutexId uartMutexHandle;


/* Function prototypes */
int fputc(int ch, FILE *f);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);
void StartDefaultTask(void const * argument);
void SensorTask(void const * argument);
void AlarmeTask(void const * argument);
void LoggerTask(void const * argument);
void CLITask(void const * argument);
void floatToString(float value, char *buffer, int decimals);



void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure DHT22 pin (PB10) */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

    /* Configure LED pin (PA8) */
    GPIO_InitStruct.Pin = LED_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);


    /* Configure USART2 pins (PA2-TX, PA3-RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = TIM_CLOCKPRESCALER_DIV2;  // Mantido para STM32L1
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.ScanConvMode = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion = 1;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_48CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    /* Turn on LED to indicate error */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    while(1)
    {
        /* Blink LED in case of error */
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }
}

int main(void)
{
  HAL_Init();
  //DWT_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
 // HAL_UART_Transmit(&huart2, (uint8_t*)"UART FUNCIONANDO!\r\n", 21, HAL_MAX_DELAY);





  /* Create mutex for shared data protection */
  osMutexDef(dataMutex);
  dataMutexHandle = osMutexCreate(osMutex(dataMutex));
  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));


  /* Create tasks */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(sensorTask, SensorTask, osPriorityNormal, 0, 512);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);
  if (sensorTaskHandle == NULL) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"FALHA SensorTask\r\n", 19, HAL_MAX_DELAY);
  } else {
      //HAL_UART_Transmit(&huart2, (uint8_t*)"SensorTask criada\r\n", 20, HAL_MAX_DELAY);
  }

  osThreadDef(alarmeTask, AlarmeTask, osPriorityHigh, 0, 512); // Prioridade alta
  alarmeTaskHandle = osThreadCreate(osThread(alarmeTask), NULL);



  osThreadDef(loggerTask, LoggerTask, osPriorityNormal, 0, 512);
  loggerTaskHandle = osThreadCreate(osThread(loggerTask), NULL);
  if (loggerTaskHandle == NULL) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"FALHA LoggerTask\r\n", 19, HAL_MAX_DELAY);
  } else {
     // HAL_UART_Transmit(&huart2, (uint8_t*)"LoggerTask criada\r\n", 20, HAL_MAX_DELAY);
  }

  osThreadDef(cliTask, CLITask, osPriorityAboveNormal, 0, 256);
  cliTaskHandle = osThreadCreate(osThread(cliTask), NULL);

  /* Start scheduler */
  osKernelStart();


  while (1)
  {
	  osDelay(1000);
  }
}

extern osMutexId uartMutexHandle; // lembre de declarar no topo do arquivo, se ainda não tiver

void SensorTask(void const * argument)
{
    if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
        //HAL_UART_Transmit(&huart2, (uint8_t*)"SensorTask INICIOU\r\n", 21, HAL_MAX_DELAY);
        osMutexRelease(uartMutexHandle);
    }

    for (;;)
    {
        // Leitura do pH via ADC
        if (HAL_ADC_Start(&hadc) == HAL_OK)
        {
            if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK)
            {
                uint32_t ph_raw = HAL_ADC_GetValue(&hadc);
                HAL_ADC_Stop(&hadc);

                // DEBUG: Mostrar valor bruto do ADC
                if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
                    char debug_msg[64];
                    char rawStr[16];
                    floatToString((float)ph_raw, rawStr, 0);
                    strcpy(debug_msg, "[DEBUG] ph_raw: ");
                    strcat(debug_msg, rawStr);
                    strcat(debug_msg, "\r\n");
                    //HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
                    osMutexRelease(uartMutexHandle);
                }

                float voltage = ph_raw * (3.3f / 4095.0f);
                float calculated_ph = 7.0f + ((2.5f - voltage) / 0.18f);

                if (dataMutexHandle != NULL && osMutexWait(dataMutexHandle, osWaitForever) == osOK)
                {
                    ph = calculated_ph;
                    osMutexRelease(dataMutexHandle);
                }
            }
            else
            {
                HAL_ADC_Stop(&hadc);
            }
        }

        // Leitura real do DHT22
        DHT22_Status dhtStatus = DHT22_ReadData(&DHT22_Data, DHT22_PORT, DHT22_PIN);

        if (dhtStatus == DHT22_OK)
        {
            if (dataMutexHandle != NULL && osMutexWait(dataMutexHandle, osWaitForever) == osOK)
            {
                // DHT22_Data já foi preenchido pela função DHT22_ReadData
                osMutexRelease(dataMutexHandle);
            }
        }
        else
        {
            const char *erro;
            if (dhtStatus == DHT22_ERROR_TIMEOUT)
                erro = "[DHT22] Timeout\r\n";
            else if (dhtStatus == DHT22_ERROR_CHECKSUM)
                erro = "[DHT22] Checksum inválido\r\n";
            else
                erro = "[DHT22] Erro desconhecido\r\n";

            if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
                HAL_UART_Transmit(&huart2, (uint8_t*)erro, strlen(erro), HAL_MAX_DELAY);
                osMutexRelease(uartMutexHandle);
            }
        }

        osDelay(1000);  // Delay entre leituras
    }
}



void AlarmeTask(void const * argument)
{
    for (;;)
    {
        float localPh = 0.0f;

        if (osMutexWait(dataMutexHandle, osWaitForever) == osOK)
        {
            localPh = ph;
            osMutexRelease(dataMutexHandle);
        }

        // Se pH fora da faixa segura [4.0, 5.5] → LED acende
        if (localPh < 4.0f || localPh > 5.5f)
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
            led_status = 1;
        }
        else
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
            led_status = 0;
        }

        osDelay(500);
    }
}





void floatToString(float value, char *buffer, int decimals) {
    int intPart = (int)value;
    float fracPart = fabs(value - intPart); // Parte fracionária absoluta

    // Trata números negativos
    if (value < 0) {
        *buffer++ = '-';
        intPart = abs(intPart);
    }

    // Converte parte inteira
    int i = 0;
    if (intPart == 0) {
        buffer[i++] = '0';
    } else {
        while (intPart > 0) {
            buffer[i++] = (intPart % 10) + '0';
            intPart /= 10;
        }
        // Inverte os dígitos
        for (int j = 0; j < i / 2; j++) {
            char temp = buffer[j];
            buffer[j] = buffer[i - j - 1];
            buffer[i - j - 1] = temp;
        }
    }

    // Parte decimal
    if (decimals > 0) {
        buffer[i++] = '.';
        for (int d = 0; d < decimals; d++) {
            fracPart *= 10;
            int digit = (int)fracPart;
            buffer[i++] = digit + '0';
            fracPart -= digit;
        }
    }

    buffer[i] = '\0'; // Terminador nulo
}

// Buffers movidos para fora da stack da Task (evita stack overflow)
static char buffer[128];
static char tempStr[16];
static char humStr[16];
static char phStr[16];

void LoggerTask(void const * argument)
{
    if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
        //HAL_UART_Transmit(&huart2, (uint8_t*)"LoggerTask INICIOU\r\n", 22, HAL_MAX_DELAY);
        osMutexRelease(uartMutexHandle);
    }

    for (;;)
    {
        float localTemp, localHum, localPh;
        uint8_t localLed;

        if (osMutexWait(dataMutexHandle, osWaitForever) == osOK)
        {
            localTemp = DHT22_Data.Temperature;
            localHum = DHT22_Data.Humidity;
            localPh = ph;
            localLed = led_status;
            osMutexRelease(dataMutexHandle);
        }

        floatToString(localTemp, tempStr, 1);
        floatToString(localHum, humStr, 1);
        floatToString(localPh, phStr, 2);

        strcpy(buffer, "[LOGGER] Temp: ");
        strcat(buffer, tempStr);
        strcat(buffer, " C | Hum: ");
        strcat(buffer, humStr);
        strcat(buffer, " % | pH: ");
        strcat(buffer, phStr);
        strcat(buffer, " | LED: ");
        strcat(buffer, localLed ? "ON" : "OFF");
        strcat(buffer, "\r\n");

        if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK)
        {
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            osMutexRelease(uartMutexHandle);
        }

        osDelay(2222); // Delay não múltiplo de 100ms para evitar conflito com CLI
    }
}



void CLITask(void const * argument)
{
    if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
        //HAL_UART_Transmit(&huart2, (uint8_t*)"CLITask INICIOU\r\n", 19, HAL_MAX_DELAY);
        osMutexRelease(uartMutexHandle);
    }

    uint8_t rxData;

    for (;;)
    {
        if (HAL_UART_Receive(&huart2, &rxData, 1, 100) == HAL_OK)
        {
            if (rxData == 'H' || rxData == 'h')
            {
                char *help =
                  "Comandos:\r\n"
                  "'H' - Ajuda\r\n"
                  "'R' - Resetar pH para 7.0\r\n"
                  "'P' - Mostrar valor atual do pH\r\n"
                  "'T' - Mostrar temperatura do sensor\r\n"
                  ;
                if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)help, strlen(help), HAL_MAX_DELAY);
                    osMutexRelease(uartMutexHandle);
                }
            }
            else if (rxData == 'R' || rxData == 'r')
            {
                if (osMutexWait(dataMutexHandle, osWaitForever) == osOK) {
                    ph = 7.0f;
                    osMutexRelease(dataMutexHandle);
                }

                char *msg = "pH resetado para 7.0\r\n";
                if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    osMutexRelease(uartMutexHandle);
                }
            }
            else if (rxData == 'P' || rxData == 'p')
            {
                float localPh;
                if (osMutexWait(dataMutexHandle, osWaitForever) == osOK) {
                    localPh = ph;
                    osMutexRelease(dataMutexHandle);
                }

                char phStr[16];
                floatToString(localPh, phStr, 2);
                char msg[64] = "Valor atual do pH: ";
                strcat(msg, phStr);
                strcat(msg, "\r\n");

                if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    osMutexRelease(uartMutexHandle);
                }
            }
            else if (rxData == 'T' || rxData == 't')
            {
                float temp;
                if (osMutexWait(dataMutexHandle, osWaitForever) == osOK) {
                    temp = DHT22_Data.Temperature;
                    osMutexRelease(dataMutexHandle);
                }

                char tempStr[16];
                floatToString(temp, tempStr, 1);
                char msg[64] = "Temperatura: ";
                strcat(msg, tempStr);
                strcat(msg, " °C\r\n");

                if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    osMutexRelease(uartMutexHandle);
                }
            }

        }

        osDelay(100);
    }
}


void StartDefaultTask(void const * argument)
{
  for(;;)
  {

  }
}

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

