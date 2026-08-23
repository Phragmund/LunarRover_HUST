#include "usb_debug.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define USB_DEBUG_BUF_SIZE 256U

/* 双缓冲: 上一帧还在 CDC 发送时, 新帧写到另一块, 避免覆盖正在发送的数据 */
static char usb_debug_buf[2][USB_DEBUG_BUF_SIZE];
static uint8_t usb_debug_idx = 0U;

void UsbDebug_Print(const char *s)
{
    uint16_t len;

    if (s == 0)
        return;

    len = (uint16_t)strlen(s);
    if (len == 0U)
        return;
    if (len >= USB_DEBUG_BUF_SIZE)
        len = (uint16_t)(USB_DEBUG_BUF_SIZE - 1U);

    memcpy(usb_debug_buf[usb_debug_idx], s, len);
    usb_debug_buf[usb_debug_idx][len] = '\0';

    /* CDC 忙则丢弃本次(非阻塞, 不拖慢控制环) */
    (void)CDC_Transmit_FS((uint8_t *)usb_debug_buf[usb_debug_idx], len);
    usb_debug_idx ^= 1U;
}

void UsbDebug_Printf(const char *fmt, ...)
{
    va_list args;
    int n;

    va_start(args, fmt);
    n = vsnprintf(usb_debug_buf[usb_debug_idx], USB_DEBUG_BUF_SIZE, fmt, args);
    va_end(args);

    if (n <= 0)
        return;
    if ((uint32_t)n >= USB_DEBUG_BUF_SIZE)
        n = (int)(USB_DEBUG_BUF_SIZE - 1U);

    (void)CDC_Transmit_FS((uint8_t *)usb_debug_buf[usb_debug_idx], (uint16_t)n);
    usb_debug_idx ^= 1U;
}
