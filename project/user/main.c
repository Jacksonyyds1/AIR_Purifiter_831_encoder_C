/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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
#include "at32f403a_407_int.h"
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"

#include "ihastek.h"
#include "TimeEvent.h"

/*************************
Notice��ES0002_AT32F403A_407_Errata_Sheet_V2.0.12_CH
**************************/
// #define HARDWARE_VERSION               "V1.0.0"
// #define SOFTWARE_VERSION               "V0.1.0"

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
void usr_handler(void);
void UI_Show(void);

int main(void)
{
    system_clock_config();
    at32_board_init();
    fw_timer_event_CancelAllTimerEvent();
    platform_init();

    while(1)
    {
        wdt_counter_reload();
        fw_timer_event_Handler();
        platformCycle();
        usr_handler();
    }
}
