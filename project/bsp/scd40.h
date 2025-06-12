#ifndef __SCD4X_H__
#define __SCD4X_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// SCD40 UART settings
#define SCD4X_UART_BAUDRATE     115200
#define SCD4X_SLAVE_ADDR        0x00

// SCD40 Commands
#define SCD4X_CMD_START_MEASUREMENT     0x00
#define SCD4X_CMD_STOP_MEASUREMENT      0x01
#define SCD4X_CMD_READ_MEASUREMENT      0x03
#define SCD4X_CMD_SELF_TEST             0x30
#define SCD4X_CMD_READ_DEVICE_INFO      0xD0
#define SCD4X_CMD_READ_VERSION          0xD1
#define SCD4X_CMD_RESET                 0xD3

// Frame control bytes
#define SCD4X_FRAME_START_STOP          0x7E
#define SCD4X_ESCAPE_BYTE               0x7D

// Error codes
#define SCD4X_ERROR_NO_ERROR            0x00
#define SCD4X_ERROR_WRONG_DATA_LENGTH   0x01
#define SCD4X_ERROR_UNKNOWN_COMMAND     0x02
#define SCD4X_ERROR_ILLEGAL_PARAMETER   0x04
#define SCD4X_ERROR_NO_DATA_AVAILABLE   0x20
#define SCD4X_ERROR_NOT_READY           0x21
#define SCD4X_ERROR_SELF_TEST_PROGRESS  0x22
#define SCD4X_ERROR_RECALIBRATION_FAIL  0x23
#define SCD4X_ERROR_COMMAND_NOT_ALLOWED 0x43
#define SCD4X_ERROR_GENERAL             0x7F

// SCD40 sensor states
typedef enum {
    SCD4X_STATE_IDLE,
    SCD4X_STATE_INITIALIZING,
    SCD4X_STATE_MEASUREMENT_STARTING,
    SCD4X_STATE_MEASUREMENT_RUNNING,
    SCD4X_STATE_SELF_TEST,
    SCD4X_STATE_ERROR
} scd4x_state_t;

// SCD40 data structure
typedef struct {
    uint16_t co2_ppm;
    bool data_ready;
    uint8_t error_code;
    scd4x_state_t state;
    uint32_t last_measurement_time;
} scd4x_data_t;

// Function declarations
void SCD40_Handler(void);
void SCD40_Init(void);
bool SCD40_Start_Measurement(void);
bool SCD40_Stop_Measurement(void);
bool SCD40_Read_Measurement(uint16_t *co2_ppm);
bool SCD40_Self_Test(void);
bool SCD40_Reset(void);
scd4x_data_t* SCD40_Get_Data(void);
void driver_SCD40_UART_Receive_Handler(uint8_t received_byte);

#ifdef __cplusplus
}
#endif

#endif
// End of SCD4X.h