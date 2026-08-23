#include "debug_uart.h"
#include <string.h>

static UART_HandleTypeDef debug_huart;

void DebugUart_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA2 = USART2_TX, PA3 = USART2_RX (AF7) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    debug_huart.Instance = USART2;
    debug_huart.Init.BaudRate = 115200;
    debug_huart.Init.WordLength = UART_WORDLENGTH_8B;
    debug_huart.Init.StopBits = UART_STOPBITS_1;
    debug_huart.Init.Parity = UART_PARITY_NONE;
    debug_huart.Init.Mode = UART_MODE_TX_RX;
    debug_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    debug_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&debug_huart) != HAL_OK)
    {
        Error_Handler();
    }
}

void DebugUart_Write(const uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0U)
        return;

    (void)HAL_UART_Transmit(&debug_huart, (uint8_t *)data, len, 100U);
}

void DebugUart_Print(const char *s)
{
    if (s == 0)
        return;

    DebugUart_Write((const uint8_t *)s, (uint16_t)strlen(s));
}
