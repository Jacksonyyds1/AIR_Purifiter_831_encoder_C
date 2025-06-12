/**
  **************************************************************************
  * @file     spi_LCD.c
  * @brief    spi_LCD source code
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

#include "spiLCD_flash.h"
#include "lcd.h"
#include "ihastek.h"

spi_init_type spi_init_struct;
dma_init_type dma_init_struct;
/**
  * @brief  spi configuration.
  * @param  none
  * @retval none
  */

void spiLCD_init(void)
{
    gpio_init_type gpio_initstructure;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

    //  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    //  gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
    //  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
    //  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    //  gpio_initstructure.gpio_pins           = GPIO_PINS_0;
    //  gpio_init(GPIOA, &gpio_initstructure);
    //
    //
    gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_initstructure.gpio_pins           = GPIO_PINS_4 | GPIO_PINS_5;
    gpio_init(GPIOC, &gpio_initstructure);
    /* software cs, pb12 as a general io to control flash cs */
    gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_initstructure.gpio_pins           = GPIO_PINS_4;
    gpio_init(GPIOA, &gpio_initstructure);

    /* sck */
    gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
    gpio_initstructure.gpio_pins           = GPIO_PINS_5;
    gpio_init(GPIOA, &gpio_initstructure);

    /* miso */
    gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
    gpio_initstructure.gpio_pins           = GPIO_PINS_6;
    gpio_init(GPIOA, &gpio_initstructure);

    /* mosi */
    gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode           = GPIO_MODE_MUX;
    gpio_initstructure.gpio_pins           = GPIO_PINS_7;
    gpio_init(GPIOA, &gpio_initstructure);

    LCD_CS_Set();
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_4;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1, TRUE);
}

void spi_LCD_ExchangeDataLengthTo8bits(void)
{
    spi_enable(SPI1, FALSE);
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1, TRUE);
}

void spi_LCD_ExchangeDataLengthTo16bits(void)
{
    spi_enable(SPI1, FALSE);
    spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
    spi_init(SPI1, &spi_init_struct);
    spi_enable(SPI1, TRUE);
}

/**
  * @brief  DAM configuration FOR LCD Transport Display Data
  * @param  none
  * @param  none
  * @retval none
  */
void DMA_LCD_Init(void)
{
    volatile uint8_t dummy_data;

    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    //dma_reset(DMA1_CHANNEL2);
    dma_reset(DMA1_CHANNEL3);

    dma_default_para_init(&dma_init_struct);

    dma_init_struct.buffer_size = 0xfffe;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)(PICTURESAVEADDRSTART);//init spim start address
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)(&SPI1->dt);
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

}

/**
  * @brief  write a byte to flash
  * @param  data: data to write
  * @retval flash return data
  */
void spiLCD_byte_write(uint8_t data)
{
    volatile uint8_t dummy_data = data;

    spi_LCD_ExchangeDataLengthTo8bits();

    dma_init_struct.buffer_size = 1;
    dma_init_struct.memory_base_addr = (uint32_t)&dummy_data;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    //dma_init_struct.memory_inc_enable = FALSE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

    spi_i2s_dma_transmitter_enable(SPI1, TRUE);

    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    while(dma_flag_get(DMA1_FDT3_FLAG) == RESET);
    dma_flag_clear(DMA1_FDT3_FLAG);

    dma_channel_enable(DMA1_CHANNEL3, FALSE);

    spi_i2s_dma_transmitter_enable(SPI1, FALSE);

}

/**
  * @brief  write data continuously
  * @param  pbuffer: the pointer for data buffer
  * @param  length: buffer length
  * @retval none
  */
void spiLCD_bytes_write(uint8_t *flash_address, uint32_t length)
{
    spi_LCD_ExchangeDataLengthTo16bits();
    LCD_CS_Clr();

    dma_init_struct.buffer_size = length;
    dma_init_struct.memory_base_addr = (uint32_t)flash_address;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

    spi_i2s_dma_transmitter_enable(SPI1, TRUE);
    //spi_i2s_dma_receiver_enable(SPI1, TRUE);

    //dma_channel_enable(DMA1_CHANNEL2, TRUE);
    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    while(dma_flag_get(DMA1_FDT3_FLAG) == RESET);
    dma_flag_clear(DMA1_FDT3_FLAG);

    // dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);

    spi_i2s_dma_transmitter_enable(SPI1, FALSE);
    //spi_i2s_dma_receiver_enable(SPI1, FALSE);

    LCD_CS_Set();
}

/**
  * @brief  write fixed data continuously
  * @param  singleColor: color
  * @param  length: color data length
  * @retval none
  */
void spiLCD_color_Fill(uint16_t singleColor, uint16_t length)
{
    volatile uint16_t num, num1, dummy_data = singleColor;

    spi_LCD_ExchangeDataLengthTo16bits();
    LCD_CS_Clr();
    dma_init_struct.buffer_size = length;
    dma_init_struct.memory_base_addr = (uint32_t)&dummy_data;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = FALSE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

    spi_i2s_dma_transmitter_enable(SPI1, TRUE);
    //spi_i2s_dma_receiver_enable(SPI1, TRUE);

    //dma_channel_enable(DMA1_CHANNEL2, TRUE);
    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    while(dma_flag_get(DMA1_FDT3_FLAG) == RESET);
    dma_flag_clear(DMA1_FDT3_FLAG);

    //dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);

    spi_i2s_dma_transmitter_enable(SPI1, FALSE);

    LCD_CS_Set();
}

/**
  * @brief  DAM configuration FOR Display Data
  * @param  none
  * @param  none
  * @retval none
  */
dma_init_type dma_data_struct;
void DMA_Data_Init(void)
{
    volatile uint8_t dummy_data;

    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
    dma_reset(DMA2_CHANNEL1);

    dma_default_para_init(&dma_data_struct);

    dma_data_struct.buffer_size = 0xfffe;
    dma_data_struct.direction = DMA_DIR_MEMORY_TO_MEMORY;
    dma_data_struct.memory_base_addr = (uint32_t) & (dummy_data); //init spim start address
    dma_data_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_data_struct.memory_inc_enable = TRUE;
    dma_data_struct.peripheral_base_addr = (uint32_t)PICTURESAVEADDRSTART;
    dma_data_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_data_struct.peripheral_inc_enable = TRUE;
    dma_data_struct.priority = DMA_PRIORITY_VERY_HIGH;
    dma_data_struct.loop_mode_enable = FALSE;
    dma_init(DMA2_CHANNEL1, &dma_data_struct);

    /* enable transfer full data interrupt */
    dma_interrupt_enable(DMA2_CHANNEL1, DMA_FDT_INT, TRUE);

    /* dma1 channel1 interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_0);
    nvic_irq_enable(DMA2_Channel1_IRQn, 0, 0);

    dma_channel_enable(DMA2_CHANNEL1, FALSE);

}

void MoveFlashDataToMemory(uint16_t *flashaddress, uint16_t *memoryaddress, uint16_t length)
{
    extern __IO uint32_t DMA2_CHA_SendDataEndFlag;
    dma_data_struct.buffer_size = length;
    dma_data_struct.memory_base_addr = (uint32_t)memoryaddress;
    dma_data_struct.peripheral_base_addr = (uint32_t)flashaddress;
    dma_init(DMA2_CHANNEL1, &dma_data_struct);
    DMA2_CHA_SendDataEndFlag = 1;
    dma_channel_enable(DMA2_CHANNEL1, TRUE);

    while(DMA2_CHA_SendDataEndFlag);

    dma_channel_enable(DMA2_CHANNEL1, FALSE);
}

void spiLCD_AllScreen_write(uint16_t *flash_address)
{
    spi_LCD_ExchangeDataLengthTo16bits();
    LCD_CS_Clr();

    dma_init_struct.buffer_size = LCD_H * LCD_W;
    dma_init_struct.memory_base_addr = (uint32_t)flash_address;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

    spi_i2s_dma_transmitter_enable(SPI1, TRUE);
    //spi_i2s_dma_receiver_enable(SPI1, TRUE);

    //dma_channel_enable(DMA1_CHANNEL2, TRUE);
    dma_channel_enable(DMA1_CHANNEL3, TRUE);

    while(dma_flag_get(DMA1_FDT3_FLAG) == RESET);
    dma_flag_clear(DMA1_FDT3_FLAG);

    // dma_channel_enable(DMA1_CHANNEL2, FALSE);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);

    spi_i2s_dma_transmitter_enable(SPI1, FALSE);
    //spi_i2s_dma_receiver_enable(SPI1, FALSE);

    LCD_CS_Set();
}
