#include "at32f403a_407_board.h"
#include "at32f403a_407_int.h"
#include "TimeEvent.h"
#include "stepper_motor.h"
#include "define.h"
#include "driver.h"
#include "scd40.h"

/**
  * @brief  this function handles timer1 overflow interrupt handler.
  * @param  none
  * @retval none
  */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    if(tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
    {
        fw_timer_event_isr_1ms();
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    }
}

/**
  * @brief  this function handles timer5 overflow interrupt handler.
  * @param  none
  * @retval none
  */
void TMR5_GLOBAL_IRQHandler(void)
{
    if(tmr_interrupt_flag_get(TMR5, TMR_OVF_FLAG) != RESET)
    {
        // 步进电机控制逻辑在这里实现
        stepper_motor_handler(); // 调用步进电机步进函数
        
        tmr_flag_clear(TMR5, TMR_OVF_FLAG);
    }
}

void TMR8_OVF_TMR13_IRQHandler(void)
{
    static uint8_t timer8count = 0;
#if USE_FAN_FG
    static unsigned char low_cnt, hi_cnt;
    static unsigned char b_fg_low;
#endif

    if(tmr_interrupt_flag_get(TMR8, TMR_OVF_FLAG) != RESET)
    {
        if(++timer8count >= 8)
        {
            timer8count = 0;
            sys_tick++;
            if(sys_tick && sys_tick % 1000 == 0)
            {
                Set_System_TickInc();
            }
            fw_timer_event_isr_1ms();
        }
#if USE_FAN_FG
        if(gpio_input_data_bit_read(FG_PORT, FG_PIN)) //FG���
        {
            hi_cnt++;
            low_cnt = 0;
            if(hi_cnt > 2)
            {
                hi_cnt = 0;
                if(b_fg_low)
                {
                    b_fg_low = 0;
                    HAL_FG_CNT_INC();
                }
            }
        }
        else
        {
            hi_cnt = 0;
            low_cnt++;
            if(low_cnt > 2)
            {
                low_cnt = 0;
                b_fg_low = 1;
            }
        }
#endif
#if USE_IR
        extern void core_ir_sig_handle(uint8_t sig);
        if(gpio_input_data_bit_read(IR_PORT, IR_PIN) == SET)
        {
            core_ir_sig_handle(1);
        }
        else
        {
            core_ir_sig_handle(0);
        }
#endif
        tmr_flag_clear(TMR8, TMR_OVF_FLAG);
    }
}

void USART1_IRQHandler(void)
{
    if(usart_interrupt_flag_get(USART1, USART_RDBF_FLAG) != RESET)
    {
        usart_flag_clear(USART1, USART_RDBF_FLAG);
        /* clear rdbf flag */
#if USE_TE
        core_te_rec_byte(usart_data_receive(USART1));
#endif
    }
    if(usart_interrupt_flag_get(USART1, USART_TDC_FLAG) != RESET)
    {
        usart_flag_clear(USART1, USART_TDC_FLAG);
    }
}

void UART5_IRQHandler(void)
{
    if(usart_interrupt_flag_get(UART5, USART_RDBF_FLAG) != RESET)
    {
        usart_flag_clear(UART5, USART_RDBF_FLAG);
        /* clear rdbf flag */
        driver_SCD40_UART_Receive_Handler(usart_data_receive(UART5));
    }
    if(usart_interrupt_flag_get(UART5, USART_TDC_FLAG) != RESET)
    {
        usart_flag_clear(UART5, USART_TDC_FLAG);
    }
}

/**
  * @brief  this function handles dma1 channel1 handler.
  * @param  none
  * @retval none
  */
extern volatile uint8_t usart2_tx_counter;
extern volatile uint16_t usart2_rx_counter;
extern uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
extern uint8_t usart2_tx_buffer[USART2_TX_BUFFER_SIZE];
extern volatile usartState usartIntState;
extern uint8_t usart2_tx_buffer_size;

void USART2_IRQHandler(void)
{
    // Receive
    if(usart_interrupt_flag_get(USART2, USART_RDBF_FLAG) != RESET)
    {
        /* RDBF = Read Data Buffer Full register
           read one byte from the receive data register */
        usart2_rx_buffer[usart2_rx_counter++] = usart_data_receive(USART2);
        // Filter out when first byte not 0x12
        if(usart2_rx_buffer[0] != DIPC_DELIMITER)
        {
            usart2_rx_counter = 0;
            memset(&usart2_rx_buffer[0], 0U, sizeof(usart2_rx_buffer));
        }
    }

    // Transmit
    if(usart_interrupt_flag_get(USART2, USART_TDBE_FLAG) != RESET)
    {
        /*TDBE: Transmit Data Buffer Empty
        write one byte to the transmit data register */
        if(usart2_tx_buffer_size > usart2_tx_counter)
        {
            usart_data_transmit(USART2, usart2_tx_buffer[usart2_tx_counter++]);
        }
    }

    if(usart_interrupt_flag_get(USART2, USART_TDC_FLAG) != RESET)
    {
        // transmit is completed
        // usart2_tx_buffer_size = usart2_tx_counter; // keep the successfully transmitted amount
        usart2_tx_buffer_size = 0;
        usart2_tx_counter = 0;
        memset(&usart2_tx_buffer[0], 0U, sizeof(usart2_tx_buffer));
        usart_interrupt_enable(USART2, USART_TDBE_INT, FALSE);
        usart_interrupt_enable(USART2, USART_TDC_INT, FALSE);
    }

}

/**
  * @brief  config DMA2_CH1 IRQ
  * @param  none
  * @retval none
  */
__IO uint32_t DMA2_CHA_SendDataEndFlag = 0x01; /* this variable should not be initialized to 0 */
void DMA2_Channel1_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA2_FDT1_FLAG) != RESET)
    {
        DMA2_CHA_SendDataEndFlag = 0;
        dma_flag_clear(DMA2_FDT1_FLAG);
    }
}

void EXINT15_10_IRQHandler(void)
{
    if(exint_interrupt_flag_get(EXINT_LINE_12) != RESET)
    {
        exint_flag_clear(EXINT_LINE_12);
    }
}

void RTCAlarm_IRQHandler(void)
{
    if(rtc_interrupt_flag_get(RTC_TA_FLAG) != RESET)
    {
        /* clear exint line flag */
        exint_flag_clear(EXINT_LINE_17);

        /* toggle led */
        // at32_led_toggle(LED4);

        /* wait for the register write to complete */
        rtc_wait_config_finish();

        /* clear alarm flag */
        rtc_flag_clear(RTC_TA_FLAG);

        /* wait for the register write to complete */
        rtc_wait_config_finish();
        core_get_tek_power()->deepsleepModeFlag = 1;
    }
}
