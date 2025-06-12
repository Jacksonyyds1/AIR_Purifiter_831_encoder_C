/**
  **************************************************************************
  * @file     at32f403a_407_board.h
  * @brief    header file for at-start board. set of firmware functions to
  *           manage leds and push-button. initialize delay function.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#ifndef __AT32F403A_407_BOARD_H
#define __AT32F403A_407_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "at32f403a_407.h"
#include "stdbool.h"
/** @addtogroup AT32F403A_407_board
  * @{
  */

/** @addtogroup BOARD
  * @{
  */

/** @defgroup BOARD_pins_definition
  * @{
  */

/**
  * this header include define support list:
  * 1. at-start-f403a v1.x board
  * 2. at-start-f407 v1.x board
  * if define AT_START_F403A_V1, the header file support at-start-f403a v1.x board
  * if define AT_START_F407_V1, the header file support at-start-f407 v1.x board
  */

#if !defined (AT_START_F403A_V1)&& !defined (AT_START_F407_V1)
#error "please select first the board at-start device used in your application (in at32f403a_407_board.h file)"
#endif


/**************** define print uart ******************/
#define PRINT_UART                       USART1
#define PRINT_UART_CRM_CLK               CRM_USART1_PERIPH_CLOCK
#define PRINT_UART_TX_PIN                GPIO_PINS_9
#define PRINT_UART_TX_GPIO               GPIOA
#define PRINT_UART_TX_GPIO_CRM_CLK       CRM_GPIOA_PERIPH_CLOCK

/**
  * @}
  */

/** @defgroup BOARD_exported_functions
  * @{
  */
typedef enum
{
    USART_IDLE,
    USART_READING,
    USART_READ_COMPLETED,
    USART_WRITING,
    USART_WRITING_COMPLETED,
} usartState;

/* exported constants --------------------------------------------------------*/

#define DIPC_HEADER_LENGTH 5U
#define DIPC_DELIMITER 0x12U
#define USART2_TX_BUFFER_SIZE       255U
#define USART2_RX_BUFFER_SIZE       (2*128)//(4*1024U)

/******************** functions ********************/
void at32_board_init(void);

/* button operation function */
void at32_GPIO_init(void);


/* delay function */
void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_sec(uint16_t sec);

/* printf uart init function */
void uart_print_init(uint32_t baudrate);

void wdt_Init(void);
void rtc_config(void);
void Timer1_Init(void);
void Timer2_Init(void);
void Timer5_Init(void);
void Timer8_Init(void);
void spim_init(void);
void usart1_configuration(void);
void usart2_configuration(void);
void usart5_configuration(void);
void i2c2_configuration(void);

void exint_config(confirm_state onoff);
void DeepSleepModeClockInit(void);

uint16_t getUsartData( uint8_t* data );
uint8_t sendUsartData( uint8_t* data, uint8_t len );

#ifdef __cplusplus
}
#endif

#endif

