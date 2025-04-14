#include "dht22.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

extern UART_HandleTypeDef huart2;
extern osMutexId uartMutexHandle;

#define DHT22_TIMEOUT 10000

/* Prototipação das funções auxiliares */
static void DHT22_SetPinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static void DHT22_SetPinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static uint8_t DHT22_CheckResponse(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* Converte um byte em duas letras hexadecimais */
void byteToHex(uint8_t byte, char* out) {
    const char hex[] = "0123456789ABCDEF";
    out[0] = hex[(byte >> 4) & 0x0F];
    out[1] = hex[byte & 0x0F];
}

void DHT22_DelayUs(uint32_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 4;
    volatile uint32_t count;
    for (count = 0; count < ticks; count++);
}

DHT22_Status DHT22_ReadData(DHT22_DataTypedef *data, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bits[5] = {0};
    uint32_t timeout;
    uint8_t i, j;

    // Start signal
    DHT22_SetPinOutput(GPIOx, GPIO_Pin);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    DHT22_DelayUs(1100);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    DHT22_DelayUs(40);
    DHT22_SetPinInput(GPIOx, GPIO_Pin);

    // Verifica resposta
    if (!DHT22_CheckResponse(GPIOx, GPIO_Pin)) {
        return DHT22_ERROR_TIMEOUT;
    }

    // Leitura de 40 bits (5 bytes)
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            timeout = 0;
            while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
                if (++timeout > DHT22_TIMEOUT)
                    return DHT22_ERROR_TIMEOUT;
            }

            DHT22_DelayUs(35);

            if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
                bits[i] |= (1 << (7 - j));
            }

            timeout = 0;
            while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
                if (++timeout > DHT22_TIMEOUT)
                    return DHT22_ERROR_TIMEOUT;
            }
        }
    }

    // Constrói e envia RAW manualmente
    char raw_msg[32] = "RAW: ";
    for (int k = 0; k < 5; k++) {
        char hex[2];
        byteToHex(bits[k], hex);
        raw_msg[5 + k * 3] = hex[0];
        raw_msg[6 + k * 3] = hex[1];
        raw_msg[7 + k * 3] = ' ';
    }
    raw_msg[5 + 5 * 3 - 1] = '\r';
    raw_msg[5 + 5 * 3] = '\n';
    raw_msg[5 + 5 * 3 + 1] = '\0';

    if (uartMutexHandle != NULL && osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)raw_msg, strlen(raw_msg), HAL_MAX_DELAY);
        osMutexRelease(uartMutexHandle);
    }

    // Verifica checksum
    uint8_t checksum = (bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF;
    if (bits[4] != checksum) {
        return DHT22_ERROR_CHECKSUM;
    }

    // Conversão dos dados
    int16_t raw_humidity = (bits[0] << 8) | bits[1];
    int16_t raw_temp = (bits[2] << 8) | bits[3];

    if (raw_temp & 0x8000) {
        raw_temp &= 0x7FFF;
        data->Temperature = -((float)raw_temp) / 10.0f;
    } else {
        data->Temperature = ((float)raw_temp) / 10.0f;
    }

    data->Humidity = ((float)raw_humidity) / 10.0f;

    return DHT22_OK;
}

/* Funções auxiliares */

static void DHT22_SetPinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static void DHT22_SetPinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static uint8_t DHT22_CheckResponse(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint32_t timeout = 0;

    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
        if (++timeout > DHT22_TIMEOUT) return 0;
        DHT22_DelayUs(1);
    }

    timeout = 0;
    while (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
        if (++timeout > DHT22_TIMEOUT) return 0;
        DHT22_DelayUs(1);
    }

    return 1;
}
