/**
  **************************************************************************
  * @file     at32f403a_407_board.c
  * @brief    set of firmware functions to manage leds and push-button.
  *           initialize delay function.
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

#include "at32f403a_407_board.h"
#include "define.h"
#include "spiLCD_flash.h"
/** @addtogroup AT32F403A_407_board
  * @{
  */

/** @defgroup BOARD
  * @brief onboard periph driver
  * @{
  */

/* delay macros */
#define STEP_DELAY_MS                    50

/* delay variable */
static __IO uint32_t fac_us;
static __IO uint32_t fac_ms;

#if 1  // 改为 1 来启用这段代码
/* support printf function, usemicrolib is unnecessary */
#if (__ARMCC_VERSION > 6000000)
  __asm (".global __use_no_semihosting\n\t");
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
  FILE __stdout;
#else
 #ifdef __CC_ARM
  #pragma import(__use_no_semihosting)
  struct __FILE
  {
    int handle;
  };
  FILE __stdout;
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
 #endif
#endif

#if defined (__GNUC__) && !defined (__clang__)
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
  while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
  usart_data_transmit(PRINT_UART, (uint16_t)ch);
  while(usart_flag_get(PRINT_UART, USART_TDC_FLAG) == RESET);
  return ch;
}

#if (defined (__GNUC__) && !defined (__clang__)) || (defined (__ICCARM__))
#if defined (__GNUC__) && !defined (__clang__)
int _write(int fd, char *pbuffer, int size)
#elif defined ( __ICCARM__ )
#pragma module_name = "?__write"
int __write(int fd, char *pbuffer, int size)
#endif
{
  for(int i = 0; i < size; i ++)
  {
    while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(PRINT_UART, (uint16_t)(*pbuffer++));
    while(usart_flag_get(PRINT_UART, USART_TDC_FLAG) == RESET);
  }

  return size;
}
#endif
#endif



/**
  * @brief  initialize uart
  * @param  baudrate: uart baudrate
  * @retval none
  */
void uart_print_init(uint32_t baudrate)
{
    gpio_init_type gpio_init_struct;

#if defined (__GNUC__) && !defined (__clang__)
    setvbuf(stdout, NULL, _IONBF, 0);
#endif

    /* enable the uart and gpio clock */
    crm_periph_clock_enable(PRINT_UART_CRM_CLK, TRUE);
    crm_periph_clock_enable(PRINT_UART_TX_GPIO_CRM_CLK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the uart tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = PRINT_UART_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(PRINT_UART_TX_GPIO, &gpio_init_struct);

    /* configure uart param */
    usart_init(PRINT_UART, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(PRINT_UART, TRUE);
    usart_enable(PRINT_UART, TRUE);
}

/**
  * @brief  board initialize interface init led and button
  * @param  none
  * @retval none
  */
void DisplayLCD_Init(void);
void at32_board_init()
{
    /* initialize delay function */
    delay_init();
    /* configure button in at_start board */
    at32_GPIO_init();
#if DEBUG_PRINT_ENABLE
    uart_print_init(921600);
#endif
    DeepSleepModeClockInit();
    rtc_config();
    wdt_Init();
    Timer1_Init();//LCD BL PWM
    Timer2_Init();//MOTOR PWM
    Timer5_Init();//Stepper Motor Timer with 32-bit counter
    Timer8_Init();//used for systemtick
    // spim_init();
    spiLCD_init();//SPI1
    DMA_LCD_Init();
    DMA_Data_Init();
    DisplayLCD_Init();
#if(!DEBUG_PRINT_ENABLE) && USE_TE
    usart1_configuration();//TE
#endif
    usart5_configuration();
    usart2_configuration();//DIPC LIB uart
    i2c2_configuration();//I2C2 for PB10/PB11
}

volatile unsigned int sys_tick = 0, system_tick = 0;

//volatile usartState usartIntState = USART_IDLE;
uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
uint8_t usart2_tx_buffer[USART2_TX_BUFFER_SIZE];
volatile uint8_t usart2_tx_counter = 0x00;
volatile uint16_t usart2_rx_counter = 0x00;
uint8_t rx_circular_buffer[USART2_RX_BUFFER_SIZE];
uint16_t rx_pointer_head = 0;
uint16_t rx_pointer_tail = 0;
uint8_t usart2_tx_buffer_size;
uint16_t usart2_rx_buffer_size;

void DeepSleepModeClockInit(void)
{
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);//deepsleep mode
    crm_periph_clock_enable(CRM_BPR_PERIPH_CLOCK, TRUE);
}
/**
  * @brief  configure button gpio
  * @param  button: specifies the button to be configured.
  * @retval none
  */
void at32_GPIO_init(void)
{
    gpio_init_type gpio_init_struct;
    /* enable gpio clock */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

    //INPUT PIN CONFIGURATION
    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);
    /* configure button pin as input with pull-up/pull-down */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = KEY_POWER_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOC, &gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = IR_GATE_NOZZLE_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = FG_PIN | IR_GATE_BASE_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);
    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);

    //OUTPUT PIN CONFIGURATION
    gpio_default_para_init(&gpio_init_struct);
    /* configure the led gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = LCD_CS_PIN | UV_EN_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_default_para_init(&gpio_init_struct);
    /* configure the led gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = LCD_RES_PIN | LCD_DS_PIN | ION_EN_PIN | IR_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_default_para_init(&gpio_init_struct);
    /* configure the air sensor gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = CO2_SENSOR_EN_PIN | AIR_SENSOR_EN_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOC, &gpio_init_struct);

    gpio_default_para_init(&gpio_init_struct);
    /* configure the nozzle stepper motor gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = MOTOR_NOZZLE_A_PLUS_PIN | MOTOR_NOZZLE_A_MINUS_PIN | MOTOR_NOZZLE_B_PLUS_PIN | MOTOR_NOZZLE_B_MINUS_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(MOTOR_NOZZLE_PORT, &gpio_init_struct);

    gpio_default_para_init(&gpio_init_struct);
    /* configure the base stepper motor gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = MOTOR_BASE_A_PLUS_PIN | MOTOR_BASE_A_MINUS_PIN | MOTOR_BASE_B_PLUS_PIN | MOTOR_BASE_B_MINUS_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(MOTOR_BASE_PORT, &gpio_init_struct);

    gpio_bits_reset(LCD_CS_PORT, LCD_CS_PIN);
    gpio_bits_reset(LCD_RES_PORT, LCD_RES_PIN);
    gpio_bits_reset(LCD_DS_PORT, LCD_DS_PIN);

    gpio_bits_reset(MOTOR_NOZZLE_PORT, MOTOR_NOZZLE_A_PLUS_PIN);
    gpio_bits_reset(MOTOR_NOZZLE_PORT, MOTOR_NOZZLE_B_PLUS_PIN);
    gpio_bits_reset(MOTOR_NOZZLE_PORT, MOTOR_NOZZLE_A_MINUS_PIN);
    gpio_bits_reset(MOTOR_NOZZLE_PORT, MOTOR_NOZZLE_B_MINUS_PIN);

    gpio_bits_reset(MOTOR_BASE_PORT, MOTOR_BASE_A_PLUS_PIN);
    gpio_bits_reset(MOTOR_BASE_PORT, MOTOR_BASE_B_PLUS_PIN);
    gpio_bits_reset(MOTOR_BASE_PORT, MOTOR_BASE_A_MINUS_PIN);
    gpio_bits_reset(MOTOR_BASE_PORT, MOTOR_BASE_B_MINUS_PIN);
}

/**
  * @brief  initialize delay function
  * @param  none
  * @retval none
  */
void delay_init()
{
    /* configure systick */
    systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
    fac_us = system_core_clock / (1000000U);
    fac_ms = fac_us * (1000U);
}

#ifndef UNIT_TESTING
/**
  * @brief  inserts a delay time.
  * @param  nus: specifies the delay time length, in microsecond.
  * @retval none
  */
void delay_us(uint32_t nus)
{
    uint32_t temp = 0;
    SysTick->LOAD = (uint32_t)(nus * fac_us);
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;
    do
    {
        temp = SysTick->CTRL;
    } while((temp & 0x01) && !(temp & (1 << 16)));

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
}

/**
  * @brief  inserts a delay time.
  * @param  nms: specifies the delay time length, in milliseconds.
  * @retval none
  */
void delay_ms(uint16_t nms)
{
    uint32_t temp = 0;
    while(nms)
    {
        if(nms > STEP_DELAY_MS)
        {
            SysTick->LOAD = (uint32_t)(STEP_DELAY_MS * fac_ms);
            nms -= STEP_DELAY_MS;
        }
        else
        {
            SysTick->LOAD = (uint32_t)(nms * fac_ms);
            nms = 0;
        }
        SysTick->VAL = 0x00;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        do
        {
            temp = SysTick->CTRL;
        } while((temp & 0x01) && !(temp & (1 << 16)));

        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0x00;
    }
}
#endif
/**
  * @brief  inserts a delay time.
  * @param  sec: specifies the delay time, in seconds.
  * @retval none
  */
void delay_sec(uint16_t sec)
{
    uint16_t index;
    for(index = 0; index < sec; index++)
    {
        delay_ms(500);
        delay_ms(500);
    }
}

/**
  * @brief  this function handles watchdog initial handler
  * @param  none
  * @retval none
  */
void wdt_Init(void)
{
    if(crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)
    {
        /* reset from wdt */
        crm_flag_clear(CRM_WDT_RESET_FLAG);
    }

    /* disable register write protection */
    wdt_register_write_enable(TRUE);

    /* set the wdt divider value */
    wdt_divider_set(WDT_CLK_DIV_32);

    /* set reload value

     timeout = reload_value * (divider / lick_freq )    (s)

     lick_freq    = 40000 Hz
     divider      = 4
     reload_value = 3000

     timeout = 3000 * (4 / 40000 ) = 0.3s = 300ms
    */
    wdt_reload_value_set(8000 - 1);

    /* reload wdt counter */
    wdt_counter_reload();

    /* enable wdt */
    wdt_enable();
}
/**
  * @brief  rtc configuration.
  * @param  none
  * @retval none
  */
void rtc_config(void)
{
    exint_init_type exint_init_struct;

    /* config the exint line of the rtc alarm */
    exint_init_struct.line_select   = EXINT_LINE_17;
    exint_init_struct.line_enable   = TRUE;
    exint_init_struct.line_mode     = EXINT_LINE_INTERRUPT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);

    /* enable the battery-powered domain write operations */
    pwc_battery_powered_domain_access(TRUE);

    /* reset battery-powered domain register */
    bpr_reset();

    /* enable the lick */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);

    /* wait lick is ready */
    while(crm_flag_get(CRM_LICK_STABLE_FLAG) == RESET);

    /* select the rtc clock source */
    crm_rtc_clock_select(CRM_RTC_CLOCK_LICK);

    /* enable rtc clock */
    crm_rtc_clock_enable(TRUE);

    /* wait for rtc registers update */
    rtc_wait_update_finish();

    /* set rtc divider: set rtc period to 1sec */
    rtc_divider_set(32767);

    /* wait for the register write to complete */
    rtc_wait_config_finish();

    /* enable alarm interrupt */
    rtc_interrupt_enable(RTC_TA_INT, TRUE);

    /* wait for the register write to complete */
    rtc_wait_config_finish();

    /* configure and enable rtc alarm interrupt */
    nvic_irq_enable(RTCAlarm_IRQn, 0, 0);
}
/**
  * @brief  this function handles timer1 initial handler
  * @param  none
  * @retval none
  */
void Timer1_Init(void)
{
    uint16_t timer_period = 0;
    uint16_t channel1_pulse = 0;
    tmr_output_config_type tmr_output_struct;
    gpio_init_type  gpio_init_struct = {0};

    crm_clocks_freq_type crm_clocks_freq_struct = {0};//for timer5 init
    /* get system clock */
    crm_clocks_freq_get(&crm_clocks_freq_struct);

    /* enable tmr1 clock */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_init_struct.gpio_pins = GPIO_PINS_13;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);

    timer_period = (crm_clocks_freq_struct.ahb_freq / 10000) - 1;
    /* compute ccr1 value to generate a duty cycle at 50% for channel 1 and 1n */
    channel1_pulse = 0;//(uint16_t)(((uint32_t) 10 * (timer_period - 1)) / 10);
    /* tmr1 configuration */
    /* time base configuration */
    /* systemclock/24000/10000 = 1hz */
    tmr_base_init(TMR1, timer_period, 0);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

    /* overflow interrupt enable */
    //tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

    /* tmr1 overflow interrupt nvic init */
    //nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    //nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);

    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
    tmr_output_struct.oc_idle_state = TRUE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_idle_state = FALSE;

    /* channel 1 */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, channel1_pulse);

    //gpio_pin_remap_config(TMR1_MUX_01,TRUE);
    /* output enable */
    tmr_output_enable(TMR1, TRUE);
    /* enable tmr1 */
    tmr_counter_enable(TMR1, TRUE);
}

/**
  * @brief  this function handles timer2 initial handler
  * @param  none
  * @retval none
  */
void Timer2_Init(void)
{
    uint16_t timer_period = 0;
    uint16_t channel2_pulse = 0;
    tmr_output_config_type tmr_output_struct;
    gpio_init_type  gpio_init_struct = {0};

    crm_clocks_freq_type crm_clocks_freq_struct = {0};//for timer2 init
    /* get system clock */
    crm_clocks_freq_get(&crm_clocks_freq_struct);

    /* enable tmr1 clock */
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_init_struct.gpio_pins = GPIO_PINS_1;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);

    timer_period = (crm_clocks_freq_struct.ahb_freq / FAN_PWM_FRE) - 1;
    /* compute ccr1 value to generate a duty cycle at 50% for channel 1 and 1n */
    channel2_pulse = (crm_clocks_freq_struct.ahb_freq / FAN_PWM_FRE / FAN_PWM_DUTY_MAX) - 1;
    /* tmr1 configuration */
    /* time base configuration */
    /* systemclock/24000/10000 = 1hz */
    tmr_base_init(TMR2, channel2_pulse, timer_period);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);

    /* overflow interrupt enable */
    //tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

    /* tmr1 overflow interrupt nvic init */
    //nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    //nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);

    tmr_output_default_para_init(&tmr_output_struct);
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
    tmr_output_struct.oc_idle_state = TRUE;
    tmr_output_struct.occ_output_state = TRUE;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_idle_state = FALSE;

    /* channel 2 */
    tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_2, channel2_pulse);

    //gpio_pin_remap_config(TMR1_MUX_01,TRUE);
    /* output enable */
    tmr_output_enable(TMR2, TRUE);
    /* enable tmr1 */
    tmr_counter_enable(TMR2, TRUE);
}

/**
  * @brief  this function handles timer5 initial handler for stepper motor
  * @param  none
  * @retval none
  */
void Timer5_Init(void)
{
    uint32_t timer_period = 0;

    crm_clocks_freq_type crm_clocks_freq_struct = {0};//for timer5 init
    /* get system clock */
    crm_clocks_freq_get(&crm_clocks_freq_struct);

    /* enable tmr5 clock */
    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);

    /* timer period calculation for stepper motor control
    Using 32-bit counter capability of Timer5 */
    timer_period = (crm_clocks_freq_struct.ahb_freq / 2000000) - 1;

    /* tmr5 configuration */
    /* time base configuration */
    tmr_base_init(TMR5, 999, timer_period);
    tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);

    /* overflow interrupt enable */
    tmr_interrupt_enable(TMR5, TMR_OVF_INT, TRUE);

    /* tmr5 overflow interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR5_GLOBAL_IRQn, 1, 0);

    /* enable tmr5 */
    tmr_counter_enable(TMR5, TRUE);
}

/**
  * @brief  this function handles timer8 initial handler
  * @param  none
  * @retval none
  */
void Timer8_Init(void)
{
    crm_clocks_freq_type crm_clocks_freq_struct = {0};//for timer5 init
    /* get system clock */
    crm_clocks_freq_get(&crm_clocks_freq_struct);

    /* enable tmr1 clock */
    crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);

    /* tmr1 configuration */
    /* time base configuration */
    /* systemclock/24000/10000 = 1hz */
    tmr_base_init(TMR8, 124, 180 - 1/*(crm_clocks_freq_struct.ahb_freq / 100) - 1*/);
    tmr_cnt_dir_set(TMR8, TMR_COUNT_UP);

    /* overflow interrupt enable */
    tmr_interrupt_enable(TMR8, TMR_OVF_INT, TRUE);

    /* tmr1 overflow interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR8_OVF_TMR13_IRQn, 0, 0);

    /* enable tmr1 */
    tmr_counter_enable(TMR8, TRUE);
}

/**
  * @brief  init the spim flash
  * @param  none
  * @retval none
  */
void spim_init(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the clock */
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    /* init spim io */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init(GPIOA, &gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_1 | GPIO_PINS_6 | GPIO_PINS_7 | GPIO_PINS_10 | GPIO_PINS_11;
    gpio_init(GPIOB, &gpio_init_struct);

    /* enable spim, and select pb10, pb11 as spim io */
    gpio_pin_remap_config(EXT_SPIM_GMUX_1001, TRUE);

    /* in this example, use on-board en25qh128a as spim flash */
    flash_spim_model_select(FLASH_SPIM_MODEL2);

    /* unlock the spim flash program erase controller */
    while(flash_flag_get(FLASH_SPIM_OBF_FLAG));
    flash_spim_unlock();
#ifndef UNIT_TESTING
    while(FLASH->ctrl3_bit.oplk);
#endif
    /* if the data written to spim flash need to be scrambled, please specify the scrambled range */
    flash_spim_encryption_range_set(0);

    return;
}

/**
  * @brief  config usart
  * @param  none
  * @retval none
  */
void usart1_configuration(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the usart3 and gpio clock */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the usart1 tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* configure the usart1 rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);

    //gpio_pin_remap_config(UART1_GMUX_0001,TRUE);
    /* config usart nvic interrupt */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(USART1_IRQn, 4, 0);

    /* configure usart1 param */
    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);

    /* enable usart1 interrupt */
    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    usart_enable(USART1, TRUE);

    //usart_interrupt_enable(UART5, USART_TDC_INT, TRUE);
}

/**
  * @brief  config usart
  * @param  none
  * @retval none
  */
void usart2_configuration(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the usart3 and gpio clock */
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the usart2 tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_2;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    /* configure the usart2 rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_3;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_init_struct);

    //gpio_pin_remap_config(UART5_GMUX_0001,TRUE);
    /* config usart nvic interrupt */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(USART2_IRQn, 0, 0);

    /* configure usart2 param */
    usart_init(USART2, 921600, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART2, TRUE);
    usart_receiver_enable(USART2, TRUE);

    /* enable usart2 interrupt */
    usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
    // usart_interrupt_enable(USART2, USART_IDLE_INT, TRUE);
    usart_enable(USART2, TRUE);

    //usart_interrupt_enable(UART5, USART_TDC_INT, TRUE);
}
void usart5_configuration(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the usart3 and gpio clock */
    crm_periph_clock_enable(CRM_UART5_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the usart5 tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_9;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    /* configure the usart5 rx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = GPIO_PINS_8;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_pin_remap_config(UART5_GMUX_0001, TRUE);
    /* config usart nvic interrupt */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(UART5_IRQn, 0, 0);

    /* configure usart5 param */
    usart_init(UART5, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(UART5, TRUE);
    usart_receiver_enable(UART5, TRUE);

    /* enable usart5 interrupt */
    usart_interrupt_enable(UART5, USART_RDBF_INT, TRUE);
    usart_enable(UART5, TRUE);

    //usart_interrupt_enable(UART5, USART_TDC_INT, TRUE);
}

/**
  * @brief  config i2c2
  * @param  none
  * @retval none
  */
void i2c2_configuration(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the i2c2 and gpio clock */
    crm_periph_clock_enable(CRM_I2C2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the i2c2 scl pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_10;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio_init_struct);

    /* configure the i2c2 sda pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_11;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio_init_struct);

    /* configure i2c2 param */
    i2c_init(I2C2, I2C_FSMODE_DUTY_2_1, 100000);

    /* set own address1 */
    i2c_own_address1_set(I2C2, I2C_ADDRESS_MODE_7BIT, 0x00);

    /* enable i2c2 */
    i2c_enable(I2C2, TRUE);
}

void exint_config(confirm_state onoff)
{
    exint_init_type exint_init_struct;

    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, onoff);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

    gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOA, GPIO_PINS_SOURCE12);

    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_select = EXINT_LINE_12;
    exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    exint_init(&exint_init_struct);

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_0);
    nvic_irq_enable(EXINT15_10_IRQn, 0, 0);
}

/**
  * @brief Fill the circular buffer with data received from USART2.
  * @details This function copies data from the temporary USART2 receive buffer (`usart2_rx_buffer`)
  *          into the circular buffer (`rx_circular_buffer`). It handles buffer wrap-around and
  *          detects potential overflow conditions where new data might overwrite unread data.
  *          If an overflow is detected, it will skip the unread data to prevent corruption.
  *
  * @note This function should be called periodically to process incoming data.
  * @note This function will modify global variables: `rx_pointer_head`, `rx_pointer_tail`, `rx_circular_buffer`, `usart2_rx_counter` and `usart2_rx_buffer`.
  *
  * @warning This function does not protect against concurrent access to shared resources.
  *          Appropriate synchronization mechanisms (e.g., disabling interrupts) should be
  *          used if this function is called from multiple contexts.
  *
  * @pre The USART2 peripheral must be properly initialized and configured to receive data.
  * @pre `usart2_rx_counter` must be updated correctly in `USART2_IRQHandler`.
  * @pre The `rx_circular_buffer` must be allocated with a suitable size.
  */
void fillRxBuffer(void)
{
    if((rx_pointer_head + usart2_rx_counter) <= sizeof(rx_circular_buffer))
    {
        // Copy to the end of the buffer
        memcpy(&rx_circular_buffer[rx_pointer_head], &usart2_rx_buffer[0], usart2_rx_counter);
    }
    else if((rx_pointer_head + usart2_rx_counter) > sizeof(rx_circular_buffer))
    {
        // Copy to the end of the buffer
        memcpy(&rx_circular_buffer[rx_pointer_head], &usart2_rx_buffer[0], sizeof(rx_circular_buffer) - rx_pointer_head);
        // Copy the rest to the beginning of the buffer
        memcpy(&rx_circular_buffer[0], &usart2_rx_buffer[sizeof(rx_circular_buffer) - rx_pointer_head], usart2_rx_counter - (sizeof(rx_circular_buffer) - rx_pointer_head));
    }

    rx_pointer_head += usart2_rx_counter;
    if(rx_pointer_head >= sizeof(rx_circular_buffer))
    {
        rx_pointer_head -= sizeof(rx_circular_buffer);
    }
}

/**
  * @brief Retrieve a complete DIPC message from the circular buffer.
  * @details This function searches for a valid DIPC message within the `rx_circular_buffer`.
  *          A valid message is identified by the `DIPC_DELIMITER` at the beginning and end,
  *          If a valid message is found, it is copied to the user-provided buffer.
  *
  * @param pData Pointer to the buffer where the extracted message will be stored.
  * @return uint16_t The length of the extracted message (including the header) if a valid
  *                 message is found, or 0 if no valid message is found.
  *
  * @note This function will modify global variables: `rx_pointer_tail` and `rx_circular_buffer`
  *
  * @pre The circular buffer (`rx_circular_buffer`) must be properly filled by the `fillRxBuffer` function.
  * @pre `rx_pointer_head` and `rx_pointer_tail` should be properly initialized.
  * @pre `pData` should point to a valid buffer large enough to store the potential message.
  *
  * @warning This function does not protect against concurrent access to shared resources.
  *          Appropriate synchronization mechanisms (e.g., disabling interrupts) should be
  *          used if this function is called from multiple contexts.
  */
uint8_t partialDataWaitCount = 0;
uint16_t retrieveRxBuffer(uint8_t *pData)
{
    static bool partialMessage = false; // Flag to indicate a partial message
    static uint16_t partialStartPos = 0; // Store the start position of the partial message
    uint8_t startDelimiterFound = 0;
    uint16_t bufferSize = 0;
    uint16_t startIdx = 0;
    uint16_t startPos = 0;
    uint16_t i = 0;

    // Check if there is something in the buffer
    if(rx_pointer_head == rx_pointer_tail)
    {
        partialMessage = false; // Reset partial message flag if buffer is empty
        return 0;
    }

    // Calculate the number of byte to read inside the circular buffer
    if(rx_pointer_head >= rx_pointer_tail)
    {
        bufferSize = rx_pointer_head - rx_pointer_tail;
    }
    else
    {
        bufferSize = sizeof(rx_circular_buffer) - rx_pointer_tail + rx_pointer_head;
    }

    // If we have a partial message, start searching from the stored position
    if(partialMessage)
    {
        startPos = partialStartPos;
        startDelimiterFound = 1; // We already found the start delimiter
    }
    else
    {
        //1. Search for the first delimiter.
        for(; startIdx < bufferSize; startIdx++)
        {
            if(rx_circular_buffer[(rx_pointer_tail + startIdx) % sizeof(rx_circular_buffer)] == DIPC_DELIMITER)
            {
                startDelimiterFound = 1;
                break;
            }
        }
    }

    // no start delimiter.
    if(!startDelimiterFound)
    {
        // If we were in a partial message state, discard it
        if(partialMessage)
        {
            partialMessage = false;
            rx_pointer_tail = rx_pointer_head;
        }
        return 0;
    }
    // start searching from the first delimiter
    if(!partialMessage)
    {
        startPos = (rx_pointer_tail + startIdx) % sizeof(rx_circular_buffer);
    }

    i = DIPC_HEADER_LENGTH;
    for(; i < bufferSize; i++)
    {
        // check if we found the end delimiter
        if(rx_circular_buffer[(startPos + i) % sizeof(rx_circular_buffer)] == DIPC_DELIMITER)
        {
            // Found the end delimiter.
            uint16_t messageLength = i + 1;

            // copy the message into the pData buffer
            uint16_t j = 0;
            for(; j < messageLength; j++)
            {
                pData[j] = rx_circular_buffer[(startPos + j) % sizeof(rx_circular_buffer)];
            }
            // Move the tail pointer.
            rx_pointer_tail = (startPos + messageLength) % sizeof(rx_circular_buffer);
            partialMessage = false; // Reset partial message flag

            return messageLength; // Return the length of the complete message
        }
    }
    if(partialDataWaitCount >= 2)
    {
        partialMessage = false;
        rx_pointer_tail = rx_pointer_head;
        partialDataWaitCount = 0;
        return 0; // No valid message found
    }
    // If we reach here, it means we found a start delimiter but no end delimiter
    partialMessage = true;
    partialStartPos = startPos;
    partialDataWaitCount++;

    return 0; // No valid message start found
}

/**
 * @brief Retrieves a complete DIPC message from the USART2 circular buffer.
 * @details This function acts as an intermediary to gather data from the USART2 peripheral.
 * It first attempts to fill the circular buffer with any new incoming data from the USART2 receive buffer if the message start with `DIPC_DELIMITER`.
 * Subsequently, it attempts to extract a complete DIPC message from the circular buffer.
 * If the extraction is successful, it clears the USART2 receive buffer and resets the associated counter.
 * If no valid message was found, it will reset the head and tail of the buffer.
 *
 * @param data Pointer to the buffer where the extracted message will be stored.
 * @return uint16_t The length of the extracted message (including the header and delimiters) if a valid
 *                 message is found, or 0 if no valid message is found.
 */
uint16_t getUsartData(uint8_t *data)
{
    uint16_t messageLength = 0; // message length
    // copy rx data into circular buffer
    if(usart2_rx_buffer[0] == DIPC_DELIMITER)
    {
        fillRxBuffer();
    }
    // Copy the retrived rx into data buffer
    messageLength = retrieveRxBuffer(data);

    // Only clear when whole message is done reading
    if(messageLength > 0)
    {
        usart2_rx_counter = 0U;
        if(rx_pointer_head == rx_pointer_tail)
        {
            memset(&usart2_rx_buffer[0], 0U, sizeof(usart2_rx_buffer));
            rx_pointer_head = 0;
            rx_pointer_tail = 0;
        }
    }
    else
    {
        // Do not reset pointer here, might still have pending message without end delimiter
    }
    // enable again the Read interrupt
    usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
    //usart_interrupt_enable(USART2, USART_IDLE_INT, TRUE);
    return messageLength;
}
uint8_t sendUsartData(uint8_t *data, uint8_t len)
{
    uint8_t writtenBytes = 0;
    if((len +  usart2_tx_counter) > sizeof(usart2_tx_buffer))
    {
        writtenBytes = sizeof(usart2_tx_buffer) - usart2_tx_counter;
    }
    else
    {
        writtenBytes = len;
    }
    usart2_tx_buffer_size = writtenBytes;
    if(writtenBytes > 0)
    {
        memcpy(&usart2_tx_buffer[usart2_tx_counter], data, writtenBytes);
    }
    usart_interrupt_enable(USART2, USART_TDBE_INT, TRUE);
    usart_interrupt_enable(USART2, USART_TDC_INT, TRUE);

    return writtenBytes;
}
