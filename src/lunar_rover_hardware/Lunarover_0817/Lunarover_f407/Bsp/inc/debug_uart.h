#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__

#include "main.h"
#include <stdint.h>

void DebugUart_Init(void);
void DebugUart_Write(const uint8_t *data, uint16_t len);
void DebugUart_Print(const char *s);

#endif
