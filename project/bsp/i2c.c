#include "i2c.h"
#include "ihastek.h"

/**
  * @brief  I2C2 send data to slave device with stop control
  * @param  slave_addr: 7-bit slave address
  * @param  data: pointer to data buffer
  * @param  len: data length
  * @retval 0: success, 1: error
  */
uint8_t i2c2_send_data(uint8_t slave_addr, uint8_t *data, uint16_t len)
{
    uint32_t i2cTick;
    uint16_t i;
    
    /* wait until i2c bus is idle */
    i2cTick = Get_Sys_Tick();
    while(i2c_flag_get(I2C2, I2C_BUSYF_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
    if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    
    /* generate start condition */
    i2c_start_generate(I2C2);
    
    /* wait for start condition generated */
    i2cTick = Get_Sys_Tick();
    while(!i2c_flag_get(I2C2, I2C_STARTF_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
    if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    
    /* send slave address for write */
    i2c_7bit_address_send(I2C2, slave_addr, I2C_DIRECTION_TRANSMIT);
    
    /* wait for address sent */
    i2cTick = Get_Sys_Tick();
    while(!i2c_flag_get(I2C2, I2C_ADDR7F_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
    if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    
    /* clear address flag */
    i2c_flag_clear(I2C2, I2C_ADDR7F_FLAG);
    
    /* send data */
    for(i = 0; i < len; i++)
    {
        i2c_data_send(I2C2, data[i]);
        
        /* wait for data register empty */
        i2cTick = Get_Sys_Tick();
        while(!i2c_flag_get(I2C2, I2C_TDBE_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
        if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    }
    
    /* wait for transfer complete */
    i2cTick = Get_Sys_Tick();
    while(!i2c_flag_get(I2C2, I2C_TDC_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
    if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    
    /* generate stop condition */
    i2c_stop_generate(I2C2);
    
    return 0;
}

/**
  * @brief  I2C2 receive data from slave device with stop control
  * @param  slave_addr: 7-bit slave address
  * @param  data: pointer to data buffer
  * @param  len: data length
  * @retval 0: success, 1: error
  */
uint8_t i2c2_receive_data(uint8_t slave_addr, uint8_t *data, uint16_t len)
{
    uint32_t i2cTick;
    uint16_t i;
    
    /* wait until i2c bus is idle */
    i2cTick = Get_Sys_Tick();
    while(i2c_flag_get(I2C2, I2C_BUSYF_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
    if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    
    /* generate start condition */
    i2c_start_generate(I2C2);
    
    /* wait for start condition generated */
    i2cTick = Get_Sys_Tick();
    while(!i2c_flag_get(I2C2, I2C_STARTF_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
    if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    
    /* send slave address for read */
    i2c_7bit_address_send(I2C2, slave_addr, I2C_DIRECTION_RECEIVE);
    
    /* wait for address sent */
    i2cTick = Get_Sys_Tick();
    while(!i2c_flag_get(I2C2, I2C_ADDR7F_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
    if(Get_Sys_Tick() - i2cTick >= 20) return 1;
    
    if(len == 1)
    {
        /* disable acknowledge for single byte */
        i2c_ack_enable(I2C2, FALSE);
        
        /* clear address flag */
        i2c_flag_clear(I2C2, I2C_ADDR7F_FLAG);
        
        /* generate stop condition */
        i2c_stop_generate(I2C2);
    }
    else
    {
        /* enable acknowledge for multiple bytes */
        i2c_ack_enable(I2C2, TRUE);
        
        /* clear address flag */
        i2c_flag_clear(I2C2, I2C_ADDR7F_FLAG);
    }
    
    /* receive data */
    for(i = 0; i < len; i++)
    {
        /* wait for data register full */
        i2cTick = Get_Sys_Tick();
        while(!i2c_flag_get(I2C2, I2C_RDBF_FLAG) && (Get_Sys_Tick() - i2cTick < 20));
        if(Get_Sys_Tick() - i2cTick >= 20) return 1;
        
        if(i == (len - 2))
        {
            /* disable acknowledge for last byte */
            i2c_ack_enable(I2C2, FALSE);
        }
        
        if(i == (len - 1))
        {
            /* generate stop condition */
            i2c_stop_generate(I2C2);
        }
        
        /* read data */
        data[i] = i2c_data_receive(I2C2);
    }
    
    /* re-enable acknowledge */
    i2c_ack_enable(I2C2, TRUE);
    
    return 0;
}
