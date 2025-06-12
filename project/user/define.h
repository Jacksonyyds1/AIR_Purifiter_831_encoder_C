#ifndef __DEFINE_H__
#define __DEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
//#include "sc32_conf.h"
//#include "Function_Init.h"
#include <stdio.h>
#include "ihastek.h"

#define PICTURESAVEADDRSTART        (0x8400000 + 0x1000U)

#define KEY_MODE_PIN            GPIO_PINS_15
#define KEY_MODE_PORT           GPIOA

#define KEY_INFO_PIN            GPIO_PINS_3
#define KEY_INFO_PORT           GPIOB

#define KEY_POWER_PIN           GPIO_PINS_13//demo C13
#define KEY_POWER_PORT          GPIOC

#define FG_PIN                  GPIO_PINS_0
#define FG_PORT                 GPIOA

#define LCD_BL_PIN              GPIO_PINS_13
#define LCD_BL_PORT             GPIOB

#define LCD_RES_PIN             GPIO_PINS_0
#define LCD_RES_PORT            GPIOB

#define LCD_CS_PIN              GPIO_PINS_4
#define LCD_CS_PORT             GPIOA

#define LCD_DS_PIN              GPIO_PINS_14
#define LCD_DS_PORT             GPIOB

#define CO2_SENSOR_EN_PIN       GPIO_PINS_14
#define CO2_SENSOR_EN_PORT      GPIOC

#define AIR_SENSOR_EN_PIN       GPIO_PINS_15
#define AIR_SENSOR_EN_PORT      GPIOC

#define UV_EN_PIN               GPIO_PINS_3
#define UV_EN_PORT              GPIOA

#define ION_EN_PIN              GPIO_PINS_12
#define ION_EN_PORT             GPIOB

#define IR_PIN                  GPIO_PINS_15
#define IR_PORT                 GPIOB

#define IR_GATE_BASE_PIN        GPIO_PINS_12
#define IR_GATE_BASE_PORT       GPIOA

#define IR_GATE_NOZZLE_PIN       GPIO_PINS_3
#define IR_GATE_NOZZLE_PORT      GPIOB

#define MOTOR_BASE_A_PLUS_PIN       GPIO_PINS_8
#define MOTOR_BASE_B_PLUS_PIN       GPIO_PINS_9
#define MOTOR_BASE_A_MINUS_PIN      GPIO_PINS_10
#define MOTOR_BASE_B_MINUS_PIN      GPIO_PINS_11

#define MOTOR_BASE_PORT             GPIOA

#define MOTOR_NOZZLE_A_PLUS_PIN     GPIO_PINS_4
#define MOTOR_NOZZLE_B_PLUS_PIN     GPIO_PINS_5
#define MOTOR_NOZZLE_A_MINUS_PIN    GPIO_PINS_6
#define MOTOR_NOZZLE_B_MINUS_PIN    GPIO_PINS_7

#define MOTOR_NOZZLE_PORT           GPIOB

#define KEY_POWER           (1 << 0)

#define MOTOR_NOZZLE    0
#define MOTOR_BASE      1

typedef struct {
    unsigned char dummy;
} User_Runtime_t;

User_Runtime_t *user_get_runtime(void);

#ifdef __cplusplus
}
#endif

#endif
