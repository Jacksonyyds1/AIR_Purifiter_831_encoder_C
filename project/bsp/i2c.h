#ifndef __I2C_H__
#define __I2C_H__

#include "at32f403a_407_board.h"

uint8_t i2c2_send_data(uint8_t slave_addr, uint8_t *data, uint16_t len);
uint8_t i2c2_receive_data(uint8_t slave_addr, uint8_t *data, uint16_t len);

#endif
