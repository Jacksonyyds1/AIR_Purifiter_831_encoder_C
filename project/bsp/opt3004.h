/**
  ******************************************************************************
  * @file    opt3004.h
  * @author  Generated Driver
  * @brief   OPT3004 Light Sensor Driver Header
  *          This file provides firmware functions to manage the OPT3004 sensor
  ******************************************************************************
  */

#ifndef OPT3004_H
#define OPT3004_H

#include <stdint.h>
#include <stdbool.h>

// OPT3004 I2C Address (7-bit)
#define OPT3004_I2C_ADDRESS         0x44

// OPT3004 Register Addresses
#define OPT3004_REG_RESULT          0x00
#define OPT3004_REG_CONFIG          0x01
#define OPT3004_REG_LOW_LIMIT       0x02
#define OPT3004_REG_HIGH_LIMIT      0x03
#define OPT3004_REG_MANUFACTURER_ID 0x7E
#define OPT3004_REG_DEVICE_ID       0x7F

// Configuration Register Bits
#define OPT3004_CONFIG_RN_AUTO      (0x0C << 12)  // Automatic full-scale range
#define OPT3004_CONFIG_CT_100MS     (0x00 << 11)  // 100ms conversion time
#define OPT3004_CONFIG_CT_800MS     (0x01 << 11)  // 800ms conversion time
#define OPT3004_CONFIG_M_SHUTDOWN   (0x00 << 9)   // Shutdown mode
#define OPT3004_CONFIG_M_SINGLE     (0x01 << 9)   // Single-shot mode
#define OPT3004_CONFIG_M_CONTINUOUS (0x02 << 9)   // Continuous mode
#define OPT3004_CONFIG_OVF          (1 << 8)      // Overflow flag
#define OPT3004_CONFIG_CRF          (1 << 7)      // Conversion ready flag
#define OPT3004_CONFIG_FH           (1 << 6)      // Flag high
#define OPT3004_CONFIG_FL           (1 << 5)      // Flag low
#define OPT3004_CONFIG_L            (1 << 4)      // Latch
#define OPT3004_CONFIG_POL          (1 << 3)      // Polarity
#define OPT3004_CONFIG_ME           (1 << 2)      // Mask exponent
#define OPT3004_CONFIG_FC_1         (0x00)        // Fault count = 1
#define OPT3004_CONFIG_FC_2         (0x01)        // Fault count = 2
#define OPT3004_CONFIG_FC_4         (0x02)        // Fault count = 4
#define OPT3004_CONFIG_FC_8         (0x03)        // Fault count = 8

// Device Identification
#define OPT3004_MANUFACTURER_ID     0x5449        // "TI"
#define OPT3004_DEVICE_ID           0x3001

// Timing Constants
#define OPT3004_STARTUP_TIME_MS     100
#define OPT3004_MEASUREMENT_INTERVAL_MS 1000
#define OPT3004_CONVERSION_TIME_MS  800           // 800ms for high accuracy

// Command execution times
#define OPT3004_CMD_EXEC_TIME_READ  10            // Register read: 10ms
#define OPT3004_CMD_EXEC_TIME_WRITE 10            // Register write: 10ms
#define OPT3004_CMD_EXEC_TIME_CONVERT 800         // Conversion time: 800ms

// Return Values
#define OPT3004_OK                  0
#define OPT3004_ERROR               1

// State definitions for Handler
typedef enum {
    OPT3004_STATE_INITIALIZING = 0,
    OPT3004_STATE_IDLE,
    OPT3004_STATE_CONFIGURING,
    OPT3004_STATE_MEASUREMENT_STARTING,
    OPT3004_STATE_MEASUREMENT_RUNNING,
    OPT3004_STATE_ERROR
} OPT3004_State_t;

// Measurement step for reading process
typedef enum {
    OPT3004_STEP_CHECK_CRF = 0,    // Check conversion ready flag
    OPT3004_STEP_READ_DATA         // Read measurement data
} OPT3004_Measurement_Step_t;

// Sensor data structure
typedef struct {
    uint16_t raw_lux;           // Raw lux reading from sensor
    float lux;                  // Calculated lux value
    uint16_t exponent;          // Range exponent
    uint16_t result_reg;        // Raw result register value
    bool overflow;              // Overflow flag
    bool conversion_ready;      // Conversion ready flag
} OPT3004_Data_t;

// Sensor control structure
typedef struct {
    OPT3004_State_t state;
    bool measurement_running;
    uint32_t last_measurement_time;
    OPT3004_Measurement_Step_t current_step;
} OPT3004_Control_t;

// Public Function Declarations
uint8_t OPT3004_Init(void);
uint8_t OPT3004_Start_Measurement(void);
uint8_t OPT3004_Stop_Measurement(void);
OPT3004_Data_t *OPT3004_Get_Data(void);
float OPT3004_Get_Lux(void);
bool OPT3004_Is_Measurement_Running(void);
void OPT3004_Handler(void);

// Configuration Functions
uint8_t OPT3004_Set_Config(uint16_t config);
uint8_t OPT3004_Get_Config(uint16_t *config);
uint8_t OPT3004_Check_Device_ID(void);

#endif // OPT3004_H