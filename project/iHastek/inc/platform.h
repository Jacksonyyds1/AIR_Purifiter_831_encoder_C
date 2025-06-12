#ifndef __PLATFORM_H
#define __PLATFORM_H
#ifdef __cplusplus
extern "C" {
#endif
#include "user_config.h"


#if USE_UART_PROTOCOL
void core_uart_protocol_rec_byte(unsigned char value);
#endif
#if USE_KEY
void core_KEY_main(void);
#endif

#ifdef __cplusplus
}
#endif
#endif
