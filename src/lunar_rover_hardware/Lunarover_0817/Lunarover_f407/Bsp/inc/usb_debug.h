#ifndef __USB_DEBUG_H__
#define __USB_DEBUG_H__

#include <stdint.h>

/* USB CDC 虚拟串口调试输出.
   依赖 CubeMX 生成的 USB_DEVICE(CDC), 生成后本模块才可编译. */
void UsbDebug_Print(const char *s);
void UsbDebug_Printf(const char *fmt, ...);

#endif
