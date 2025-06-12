
#include "user_config.h"
#include "types.h"
#include "user_platform.h"
#include <stdio.h>
#include <stdarg.h>


#if (C51_COMPILE==0)

#ifdef DEBUG_PRINT_LEVEL
void drive_uart_print_Send(char *buffer, tek_u16 count);
void Platform_DebugPrint(const char *pMessage, ...)
{
    char buffer[250];
    va_list argptr;
    va_start(argptr, pMessage);
    int count = vsprintf(buffer, pMessage, argptr);
    va_end(argptr);
    if(count > 0)
    {
        buffer[count] = '\r';
        buffer[count + 1] = '\n';
        count += 2;
        drive_uart_print_Send((char *)buffer, count);
    }
}
#endif

#endif


