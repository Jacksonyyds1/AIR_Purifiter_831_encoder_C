/**
  ******************************************************************************
  * @file    SEN68.h
  * @author  Generated Driver
  * @brief   Header file for SEN68 Environmental Sensor Driver
  ******************************************************************************
  */

#ifndef SEN68_H
#define SEN68_H

#include <stdint.h>
#include <stdbool.h>
#include "at32f403a_407_crc.h"

// SEN68 I2C Address (7-bit)
#define SEN68_I2C_ADDRESS           0x6B

// SEN68 Command IDs
#define SEN68_CMD_START_MEASUREMENT         0x0021
#define SEN68_CMD_STOP_MEASUREMENT          0x0104
#define SEN68_CMD_GET_DATA_READY            0x0202
#define SEN68_CMD_READ_MEASURED_VALUES      0x0467
#define SEN68_CMD_READ_RAW_VALUES           0x0455
#define SEN68_CMD_READ_NUMBER_CONCENTRATION 0x0316
#define SEN68_CMD_SET_TEMP_OFFSET_PARAMS    0x60B2
#define SEN68_CMD_SET_TEMP_ACCEL_PARAMS     0x6100
#define SEN68_CMD_GET_PRODUCT_NAME          0xD014
#define SEN68_CMD_GET_SERIAL_NUMBER         0xD033
#define SEN68_CMD_READ_DEVICE_STATUS        0xD206
#define SEN68_CMD_READ_CLEAR_DEVICE_STATUS  0xD210
#define SEN68_CMD_GET_VERSION               0xD100
#define SEN68_CMD_DEVICE_RESET              0xD304
#define SEN68_CMD_START_FAN_CLEANING        0x5607
#define SEN68_CMD_ACTIVATE_SHT_HEATER       0x6765
#define SEN68_CMD_GET_SHT_HEATER_MEAS       0x6790
#define SEN68_CMD_GET_VOC_ALGORITHM_PARAMS  0x60D0
#define SEN68_CMD_SET_VOC_ALGORITHM_PARAMS  0x60D0
#define SEN68_CMD_GET_VOC_ALGORITHM_STATE   0x6181
#define SEN68_CMD_SET_VOC_ALGORITHM_STATE   0x6181
#define SEN68_CMD_GET_NOX_ALGORITHM_PARAMS  0x60E1
#define SEN68_CMD_SET_NOX_ALGORITHM_PARAMS  0x60E1

// CRC Constants
#define SEN68_CRC8_POLYNOMIAL       0x31
#define SEN68_CRC8_INIT             0xFF

// Timing Constants
#define SEN68_STARTUP_TIME_MS       1000
#define SEN68_MEASUREMENT_INTERVAL_MS 1000

// Command execution times (from datasheet Table 4.8)
#define SEN68_CMD_EXEC_TIME_DEFAULT         20    // Most commands: 20ms
#define SEN68_CMD_EXEC_TIME_START_MEASURE   50    // Start measurement: 50ms  
#define SEN68_CMD_EXEC_TIME_STOP_MEASURE    1000  // Stop measurement: 1000ms

// Status Register Bits
#define SEN68_STATUS_SPEED_WARNING  (1 << 21)
#define SEN68_STATUS_HCHO_ERROR     (1 << 10)
#define SEN68_STATUS_GAS_ERROR      (1 << 7)
#define SEN68_STATUS_RHT_ERROR      (1 << 6)
#define SEN68_STATUS_FAN_ERROR      (1 << 4)

// Return Values
#define SEN68_OK                    0
#define SEN68_ERROR                 1

// State definitions for Handler
typedef enum {
    SEN68_STATE_INITIALIZING = 0,
    SEN68_STATE_IDLE,
    SEN68_STATE_MEASUREMENT_STARTING,
    SEN68_STATE_MEASUREMENT_RUNNING,
    SEN68_STATE_ERROR
} SEN68_State_t;

// Command execution state for async operation
typedef enum {
    SEN68_CMD_STATE_IDLE = 0,
    SEN68_CMD_STATE_WAITING_RESPONSE,
    SEN68_CMD_STATE_COMPLETED,
    SEN68_CMD_STATE_ERROR
} SEN68_Command_State_t;

// Sensor control structure
typedef struct {
    SEN68_State_t state;
    bool measurement_running;
    uint32_t last_measurement_time;
} SEN68_Control_t;

// Async command structure
typedef struct {
    SEN68_Command_State_t state;
    uint32_t start_time;
    uint32_t timeout;
    uint16_t current_command;
    uint8_t expected_response_len;
    uint8_t response_buffer[64];
    bool result;
} SEN68_AsyncCmd_t;

// Data Structures
typedef struct {
    uint16_t pm1_0;                 // PM1.0 concentration (μg/m³ * 10)
    uint16_t pm2_5;                 // PM2.5 concentration (μg/m³ * 10)
    uint16_t pm4_0;                 // PM4.0 concentration (μg/m³ * 10)
    uint16_t pm10_0;                // PM10.0 concentration (μg/m³ * 10)
    int16_t humidity;               // Relative humidity (% * 100)
    int16_t temperature;            // Temperature (°C * 200)
    int16_t voc_index;              // VOC index (* 10)
    int16_t nox_index;              // NOx index (* 10)
    uint16_t hcho_concentration;    // Formaldehyde concentration (ppb * 10)
} SEN68_Data_t;

typedef struct {
    int16_t raw_humidity;           // Raw humidity (% * 100)
    int16_t raw_temperature;        // Raw temperature (°C * 200)
    uint16_t raw_voc;               // Raw VOC ticks
    uint16_t raw_nox;               // Raw NOx ticks
} SEN68_Raw_Data_t;

typedef struct {
    uint16_t pm0_5;                 // PM0.5 number concentration (particles/cm³ * 10)
    uint16_t pm1_0;                 // PM1.0 number concentration (particles/cm³ * 10)
    uint16_t pm2_5;                 // PM2.5 number concentration (particles/cm³ * 10)
    uint16_t pm4_0;                 // PM4.0 number concentration (particles/cm³ * 10)
    uint16_t pm10_0;                // PM10.0 number concentration (particles/cm³ * 10)
} SEN68_Number_Concentration_t;

typedef struct {
    int16_t index_offset;           // VOC/NOx index offset (1-250)
    int16_t learning_time_offset_hours;  // Learning time offset hours (1-1000)
    int16_t learning_time_gain_hours;    // Learning time gain hours (1-1000)
    int16_t gating_max_duration_minutes; // Gating duration minutes (0-3000)
    int16_t std_initial;            // Standard deviation initial (10-5000)
    int16_t gain_factor;            // Gain factor (1-1000)
} SEN68_Algorithm_Params_t;

typedef struct {
    int16_t offset;                 // Temperature offset (°C * 200)
    int16_t slope;                  // Temperature slope (* 10000)
    uint16_t time_constant;         // Time constant (seconds)
    uint16_t slot;                  // Temperature slot (0-4)
} SEN68_Temp_Offset_Params_t;

// Function Prototypes
SEN68_Data_t *SEN68_Get_Data(void);
uint8_t SEN68_Init(void);
uint8_t SEN68_Start_Measurement(void);
uint8_t SEN68_Stop_Measurement(void);
uint8_t SEN68_Is_Data_Ready(void);
uint8_t SEN68_Get_Product_Name(void);
uint8_t SEN68_Get_Serial_Number(void);
uint32_t SEN68_Read_Device_Status(void);
uint8_t SEN68_Reset_Device(void);
uint8_t SEN68_Start_Fan_Cleaning(void);
void SEN68_Handler(void);

#endif // SEN68_H