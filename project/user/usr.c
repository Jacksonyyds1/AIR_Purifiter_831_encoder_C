#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "spiLCD_flash.h"
#include "lcd.h"
#include "TimeEvent.h"
#include "flash.h"
#include "ihastek.h"

User_Runtime_t user_runtime;

User_Runtime_t *user_get_runtime(void)
{
    return &user_runtime;
}

void light_panel_handler(void)
{
    drive_LCD_BackLight_set(70); // Set backlight to 70% duty cycle
}

void work_handler(void)
{
    static bool first_run = true;
    if (first_run) {
        UI_Show();
        first_run = false;
    }
}

void usr_handler(void)
{
    work_handler();
    light_panel_handler();
}
