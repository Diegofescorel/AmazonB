#include "dht22.h"
#include "main.h"
#include <math.h>

#define DHT22_TIMEOUT 10000  // Aumentado para ambiente real

/* Private functions */
static void DHT22_SetPinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static void DHT22_SetPinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
static uint8_t DHT22_CheckResponse(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void DHT22_DelayUs(uint16_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 4;
    volatile uint32_t count;
    for(count = 0; count < ticks; count++);
}

static void DHT22_SetPinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static void DHT22_SetPinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

DHT22_Status DHT22_ReadData(DHT22_DataTypedef *data, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint8_t bits[5] = {0};
    uint32_t timeout;
    uint8_t i, j;

    /* Inicializa a comunicação */
    DHT22_SetPinOutput(GPIOx, GPIO_Pin);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    DHT22_DelayUs(1100);  // 1.1ms LOW
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    DHT22_DelayUs(40);    // 40us HIGH
    DHT22_SetPinInput(GPIOx, GPIO_Pin);

    /* Verifica resposta do sensor */
    if(!DHT22_CheckResponse(GPIOx, GPIO_Pin)) {
        return DHT22_ERROR_TIMEOUT;
    }

    /* Lê os 40 bits de dados */
    for(i = 0; i < 5; i++) {
        for(j = 0; j < 8; j++) {
            timeout = 0;
            while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
                if(++timeout > DHT22_TIMEOUT) return DHT22_ERROR_TIMEOUT;
            }

            DHT22_DelayUs(35);  // Espera 35us

            if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
                bits[i] |= (1 << (7 - j));
            }

            timeout = 0;
            while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
                if(++timeout > DHT22_TIMEOUT) return DHT22_ERROR_TIMEOUT;
            }
        }
    }

    /* Verifica checksum */
    if(bits[4] != ((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF)) {
        return DHT22_ERROR_CHECKSUM;
    }

    /* Converte e armazena os dados */
    int16_t temp = (bits[2] << 8) | bits[3];
    int16_t hum = (bits[0] << 8) | bits[1];

    data->Temperature = (float)temp / 10.0f;
    data->Humidity = (float)hum / 10.0f;

    return DHT22_OK;
}

static uint8_t DHT22_CheckResponse(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t timeout = 0;

    // Espera o pino ficar LOW (80us)
    while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
        if(++timeout > DHT22_TIMEOUT) return 0;
        DHT22_DelayUs(1);
    }

    timeout = 0;
    // Espera o pino ficar HIGH (80us)
    while(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
        if(++timeout > DHT22_TIMEOUT) return 0;
        DHT22_DelayUs(1);
    }

    return 1;
}
