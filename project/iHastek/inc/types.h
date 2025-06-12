#ifndef _TYPES_H
#define _TYPES_H
#ifdef __cplusplus
extern "C" {
#endif
#include "user_config.h"
#include "stdbool.h"
#include "stdint.h"

//#inlcude "WIFI_Command.h"

#define LongToBin(n) \
    (                    \
                         ((n >> 21) & 0x80) | \
                         ((n >> 18) & 0x40) | \
                         ((n >> 15) & 0x20) | \
                         ((n >> 12) & 0x10) | \
                         ((n >>  9) & 0x08) | \
                         ((n >>  6) & 0x04) | \
                         ((n >>  3) & 0x02) | \
                         ((n      ) & 0x01)   \
    )
#define Bin(n) LongToBin(0x##n##l)

typedef struct
{

    uint16_t buildno; //!< Build number in little endian format e.g. build number 0005 is sent as 0500

    uint8_t minor; //!< Minor version number
    uint8_t major; //!< Major version number
} tDipcMsg_version;

/*��λ���*/
typedef enum
{
    SPEEDMODE_AUTO = 0,
    SPEEDMODE_DND,
    SPEEDMODE_MANUAL,
    SPEEDMODE_MAX
} SPEEDMODE;

typedef enum
{
    SPEEDLEVEL_1 = 0,
    SPEEDLEVEL_2,
    SPEEDLEVEL_3,
    SPEEDLEVEL_4,
    SPEEDLEVEL_5,
    SPEEDLEVEL_6,
    SPEEDLEVEL_7,
    SPEEDLEVEL_8,
    SPEEDLEVEL_9,
    SPEEDLEVEL_10,
    SPEEDLEVEL_MAX
} SPEEDLEVEL;
typedef enum
{
    SPEEDCHANGETYPE_USER = 0,
    SPEEDCHANGETYPE_APP,
    SPEEDCHANGETYPE_MAX
} SPEEDCHANGETYPE;
typedef struct
{
    SPEEDMODE mode;
    SPEEDLEVEL lev;
} SPEEDPARA_t; //
typedef struct
{
    SPEEDCHANGETYPE changetype;
    unsigned long tick;
    SPEEDPARA_t speedpara[2];//0:user;1:app
} SPEED_t; //

typedef struct
{
    uint8_t count;
    uint16_t dataArray[12];
} SENSORDATA_t; //sensor data save

typedef struct
{
    uint8_t aMinDataCounter;//max 60
    uint8_t twentyMinDataCounter;//max 20 min
    int16_t twentyMinDataArray[20];
    int16_t pm25twentyMinDataArray[20];
    int16_t pm10twentyMinDataArray[20];
    int8_t secsofAutoMode[20];
    int8_t speedmode[20];
    int8_t secsoFanOn[20];
    int8_t speedlevel[20];
} OneMinData_t;
typedef union
{
    int8_t groupdataArray[24 * 3][20];
    int8_t dataArray[24 * 3 * 20];
} SENSOR_8b_uion;

typedef union
{
    int16_t groupdataArray[24 * 3][20];
    int16_t dataArray[24 * 3 * 20];
} SENSOR_16b_uion;
typedef struct
{
    uint8_t groupCount;
    SENSOR_16b_uion envdata;
    SENSOR_16b_uion pm25data;
    SENSOR_16b_uion pm10data;
    SENSOR_8b_uion AutoModeSecs;
    SENSOR_8b_uion SpeedModeIndex;
    SENSOR_8b_uion FanOnSecs;
    SENSOR_8b_uion FanSpeedLev;
} SENSORDATAFORAPP_t; //sensor data save for app

typedef struct
{
    uint32_t secsofAQPage;
    uint32_t secsofPM25Page;
    uint32_t secsofPM10Page;
    uint32_t secsofFilterLifePage;
    uint32_t secsofWIFIStatusPage;
    uint32_t numofPowerButton;
    uint32_t numofModeButton;
    uint32_t numofInfoButton;
} PROCUCTDATA_t;
typedef enum
{
    WIFISTATUS_DISABLE = 0,
    WIFISTATUS_ENABLE,
    //WIFISTATUS_DISCONNECTED,
    //WIFISTATUS_CONNECTED,
    WIFISTATUS_ERROR
} WIFISTATUS;

typedef struct
{
    unsigned long wifigettimeTick;
    uint8_t wifigettimeValue[4];
} WIFITIME_t;

#if USE_KEY
typedef enum
{
    KEYLONGPRESSMODE_NULL = 0,
    KEYLONGPRESSMODE_HEPARESET,
    KEYLONGPRESSMODE_CARBONRESET,
    KEYLONGPRESSMODE_FACTORYRESET,
} KEYLONGPRESSMODE;
#endif

typedef enum
{
    LCDBLLEV_OFF = 0,
    LCDBLLEV_LOW,
    LCDBLLEV_HIGH,
} LCDBLLEV;

typedef enum//reasonForMode
{
    RFM_NONE = 0,// the current product mode reason is unknown
    RFM_PUI,// the current product mode was set via the product user interface
    RFM_PRC,// the current product mode was set via the product remote control
    RFM_LAPP,// the current product mode was set via the Local APP
    RFM_RAPP,// the current product mode was set via the Remote APP
    RFM_LSCH,// the current product mode was set via the local scheduler
    RFM_RSCH,// the current product mode was set via the remote scheduler
    RFM_ALXA,// the current product mode was set via an Alexa equipment
    RFM_TMAL,// the current product mode was set via TMALL Genie Voice applet service
    RFM_BAID,// the current product mode was set via Baidu Voice applet service
    RFM_MAPP,// the current product mode was set via We-Chat mini program
    RFM_SIRI,// the current product mode was set via iOS voice control service
    RFM_STMR,// the current product mode was set via Sleep Timer Expiry
    RFM_PTPL,// the current product mode was set via Product Topple
    RFM_FRST,// the current product mode was set via Factory Reset
    RFM_GOGL,//15 the current product mode was set via Google Voice Applet Service
} ENUM_RFM;

typedef enum
{
    CJ_NONE = 0,
    CJ_TIMEOUT,
    CJ_FAIL,
    CJ_SUCCESS,
} ENUM_CJ;

typedef enum
{
    EMENUM_FIRSTPAGE = 0,
    EMENUM_SECONDPAGE,
    EMENUM_THIRDPAGE,
    EMENUM_MAXPAGE,
} ENUM_EM;
typedef enum
{
    EMPAGEENUM_FIRMWAREVERSION = 0,
    EMPAGEENUM_SKUINFO,
    EMPAGEENUM_FAULTSCODES,
    EMPAGEENUM_FUNCTIONALSELFTEST,
    EMPAGEENUM_MODESELFTEST,
    EMPAGEENUM_FILTERSELFTEST,
    EMPAGEENUM_ENGINEERINGMODERESET,
    EMPAGEENUM_MAXPAGE,
} ENUM_EMPAGE;
typedef struct
{
    ENUM_EM emmenuPageIndex;
    uint8_t emIndex;
    uint8_t emIndexSecondPage: 1;
    unsigned long emTick;
    ENUM_EMPAGE emPage;
    uint8_t emModeSelfTestIndex;
} EM_STRUCT;

typedef enum
{
    OTASTATUS_UPTODATE = 0,//OTA Status Up to Date
    OTASTATUS_DOWNLOADING,//OTA Status Downloading//useless
    OTASTATUS_WAITINGAUTHORISATION,//OTA Status Waiting Authorisation
    OTASTATUS_WAITINGCONVENIENTTIME,//OTA Status Waiting Convenient Time
    OTASTATUS_STARTINSTALLFW,//OTA Status Start Install Firmware for OTA Primary MCU
    OTASTATUS_STARTREVERTBACKTORECOVERYFW,//OTA Status Start Revert back to Recovery Firmware
    OTASTATUS_WAITINGFWSTABLEAFTERINSTALL,//OTA Status waiting Firmware Stable after install
    OTASTATUS_INSTALLFWFORSECONDARYMCU = 0x08,//OTA Status Install Firmware for OTA Secondary MCU
    OTASTATUS_DOWNLOADINCOMPLETE = 0x0D,//OTA Status Download Incomplete
    OTASTATUS_INSTALLFWFORPRIMARYMCU = 0x0E,//OTA Status Installed Firmware for OTA Primary MCU
} OTASTATUS_ENUM;

typedef struct
{
    OTASTATUS_ENUM otaStatus;
    unsigned long otaExFlashDownAddr;
    unsigned long otaReceivedDataTick;
} OTA_t;

typedef struct
{
    bool wifiModeDorPStatus;
    uint32_t rssi;
    uint8_t wifiMAC[18];
    uint8_t bleMAC[18];
    uint8_t wifiSN[16];
} WIFICONNECTDATA_t;
typedef struct
{
#if USE_POWER
    uint8_t on: 1;
    uint8_t deepsleepModeFlag: 1;
    uint8_t deepsleepModeFirstWakeupFlag: 1;
    unsigned long dcONTick;
    unsigned long DSModeTick;
    uint8_t hadDisplayLOGO: 1;
    uint8_t needSaveImmediately: 1;
    uint8_t aconFlagForSendWIFIEnable: 1; //
    unsigned long facModeTick;
    uint8_t donotFeedWDTFlag: 1;
    ENUM_CJ CJ;
    EM_STRUCT emStruct;
    uint8_t needupdateInEMPage: 1;

    unsigned long on_secs;
#endif
#if USE_KEY
    KEYLONGPRESSMODE keyLongPressMode;
    unsigned long countdownTick;
#endif
    SPEED_t speed;
#if SPEED_LEVEL_MAX
    //uint8_t speed;
    uint8_t real_speed;//�����ת��λ
    //uint8_t auto_speed:1;
#if SPEED_LEVEL_SLEEP_ENABLE
    uint8_t sleep: 1;
#endif
    uint16_t rpmofspeed;
#endif
#if USE_UV
    uint8_t uv: 1;
#endif
#if USE_ION
    uint8_t ion: 1;
#endif
} TEKPower_t;

typedef struct
{
    const uint16_t *table;
    uint8_t  size;
    uint16_t  max;
    uint16_t  fre;

} TEKSpeed_t;

typedef struct
{
#if USE_FAN_ERR
    uint8_t  Err_Cnt;
    uint8_t  Err_En_Cnt;
    uint8_t  Err;
    uint8_t  Err_old;
#endif

#if USE_TWO_FG || USE_THREE_FG
#if USE_FAN_ERR
    uint8_t  Err_Cnt2;
    uint8_t  Err_En_Cnt2;
    uint8_t  Err2;
    uint8_t  Err_old2;
    uint8_t  Err_Cnt3;
    uint8_t  Err_En_Cnt3;
    uint8_t  Err3;
    uint8_t  Err_old3;
#endif
    uint8_t   TrueHigh2;
    uint16_t  PWMD2;
    uint16_t  Count2;
    uint8_t   TrueHigh3;
    uint16_t  PWMD3;
    uint16_t  Count3;
#endif
    uint8_t   TrueHigh;
    uint16_t  Error;
    uint16_t  NoChangeTime;
    uint16_t  Step;
    uint16_t  Count;
    uint16_t  TimeBase;
    uint16_t  PWMD;
    //uint16_t  PWD_TAR;
    uint16_t  PWMD_old;
    uint16_t  En;
    uint16_t  set_duty_time;
    const uint16_t *rpm_table;
    uint8_t  step_all;
} TEKFan_t;

typedef struct
{
    uint16_t max;
    uint16_t duty;
#if USE_TWO_FG || USE_THREE_FG
    uint16_t duty2;
    uint16_t duty3;
#endif
    uint16_t target_duty;
    uint16_t fre;
    uint8_t te_set;
} TEKPWM_t;

typedef struct
{
    uint8_t pin;
    uint16_t cnt;
    unsigned long tick;
#if USE_THREE_FG
    uint8_t pin2;
    uint16_t cnt2;
    unsigned long tick2;
    uint8_t pin3;
    uint16_t cnt3;
    unsigned long tick3;
#elif USE_TWO_FG
    uint8_t pin2;
    uint16_t cnt2;
    unsigned long tick2;
#endif
} TEKFG_t;

typedef struct
{
    uint8_t LPType: 2;
    uint8_t KeyValue: 3;
    uint8_t Status: 3;
} KEYHOLDAFTERLP_t;

typedef struct
{
    uint8_t now;
    uint8_t last;
    uint8_t verify;
    uint16_t te;
    unsigned long keep_tick;
    unsigned long keep_thirdly_tick;

    uint8_t ok;
    uint8_t up;
    uint8_t keep;
    uint8_t second_keep;
    uint8_t third_keep;
    uint8_t once_up;
    uint8_t delay_release;
    unsigned long once_up_tick;
    KEYHOLDAFTERLP_t KeyHoldAfterLP;
} TEKKey_t;

#if USE_UART_PROTOCOL||USE_TE
typedef struct
{
    uint8_t *ptx;
    uint8_t *prx;
    uint16_t tx_size;
    uint16_t rx_size;
    uint16_t tx_rear;
    uint16_t tx_top;
    uint16_t rx_rear;
    uint16_t rx_top;
} TEKUart_t;
#endif

#if UV_FREQUENCE_DETECT
typedef struct
{
    uint8_t uv_fault: 1;
    unsigned long ErrorCnt;
} UV_TypeDef;
#endif

#if USE_CHORD_MUSIC
typedef struct
{
    uint8_t  Type;
    uint8_t  Start;
    uint16_t StartTime;
    uint16_t Time;
    uint8_t  Step;
    uint8_t   En;
    unsigned long  TimerBase10ms;
    unsigned long  TimerBase100ms;
    uint8_t   int_num;
} Buzz_TypeDef;

typedef enum
{
    chord_SD = 0x81,
    chord_KJ = 0x82,
    chord_KEY = 0x83
} CHORD_ENUM;
#endif

typedef struct
{
    uint16_t num;
    uint8_t on;
    uint8_t force;

    uint8_t aqi_level;
    uint8_t aqi_level_size;

    //volatile uint16_t *low;
    uint8_t low_register;
    uint8_t ppd42_per;
    uint16_t ram;
    //uint16_t last;

    //uint8_t err_cnt;
    //uint8_t Err;
} TEKPM25_t;

typedef struct
{
    uint16_t pm25;
    uint16_t auto_speed;
} TEK_PM25_AUTO_SPEED_t;

typedef struct
{
    unsigned long writeCounter;
#if USE_TE
    uint16_t TEAddress;
#endif

#if HEPA_TIMEOUT
    unsigned long HEPAactionSecs;
#if HEPA2_TIMEOUT
    unsigned long HEPA2actionSecs;
#endif
#if HEPA3_TIMEOUT
    unsigned long HEPA3actionSecs;
#endif
#endif
} TEKFlashData_t;

typedef union
{
    TEKFlashData_t userSaveFlashData;
    uint8_t userSaveDataArray[sizeof(TEKFlashData_t)];
} SaveDataUnion;

#if USE_TIMER
typedef struct
{
    uint8_t onoff: 1;//homics private app need
    unsigned long start_tick;
    unsigned long onesec_tick;
    unsigned long left_secs;
    unsigned long all_secs;
    uint8_t  step;
    uint8_t  step_last;
    uint8_t  hours;
    uint16_t step_setupmin;
    uint16_t step_min;
    //uint8_t timerDisplay: 1;
} TEKTime_t;
#endif

#if USE_HUMIDIFIER
typedef struct
{
    uint8_t low_water: 1;
    uint8_t burn: 1;
    uint8_t const_humi: 1;
    uint8_t warm_fog: 1;
    uint8_t sleep: 1;
    uint8_t night: 1;
    uint8_t power_off_delay: 1;
    uint8_t b_wink: 1;
    uint8_t wink_time;
    uint8_t const_humi_value;//��ʪ�趨ֵ
    uint8_t humidity;
    uint8_t temperature;
#if USE_WATER_LACK_SINGLE_PIN
    uint16_t burn_pluse;
#endif
    unsigned long wink_tick;
} TEKHumiditifier_t;
#endif

#if HEPA_TIMEOUT
typedef struct
{
    uint8_t time_out: 1;
    uint8_t reset: 1;
    uint8_t human: 1;
    uint8_t res: 5;

    uint8_t   left_percent;

    uint8_t   flash_time;

    unsigned long  secs;
    unsigned long  all_secs;  //run
    unsigned long  cnt;
    unsigned long  all;  //timer out

    unsigned long  start_secs;
    unsigned long  start_all_secs;
    unsigned long  start_tick;
    unsigned long  sec_tick;

    unsigned long  flash_tick;
    unsigned long  left_days;

} TEKHepa_t;
#endif
typedef struct
{
    uint8_t hepa;
    uint8_t hepa2;
    uint8_t addr;
} TEKFlashWrite_t;
typedef enum
{
    TEK_ENUM_HEPA,
    TEK_ENUM_HEPA2,
    TEK_ENUM_HEPA3,
    TEK_ENUM_ADDR,
    TEK_ENUM_RESERVE
} TEKFlash_e;

typedef struct
{
    uint8_t fre;
    uint8_t Low: 1;
    uint8_t High: 1;
    uint8_t pin: 1;
    uint8_t res: 5;

    uint16_t Fre_CNT;
    uint8_t Hi_num;
    uint8_t Lo_num;

    uint8_t  OneSecond_CNT;

    unsigned long  seconds;

} TEKZERO_t;

typedef struct
{
    uint16_t AD;
    uint16_t  Ratio;
    uint8_t  Es_Air_Condition;
    uint8_t Air_Condition;
    uint8_t quantity_select;
    uint16_t Rs;
    uint16_t Ro;

} TEKAir_t;

typedef struct
{
    uint8_t  Type;
    uint8_t  Start;
    uint16_t StartTime;
    uint16_t Time;
    uint8_t  Step;
    uint8_t   En;
    unsigned long  TimerBase10ms;
    unsigned long  TimerBase100ms;
    uint8_t   int_num;
} Buzz_TypeDef;

typedef enum
{
    chord_SD = 0x81,
    chord_KJ = 0x82,
    chord_KEY = 0x83

} CHORD_ENUM;


typedef struct
{
    uint8_t bitcnt;
    uint8_t rec_ok;
    uint8_t frame_head;
    uint8_t sig_high;
    uint16_t key_last;
    uint32_t rec_data;
} IR_TypeDef;

#if USE_TE
#if USE_UV
//  typedef struct
//  {
//          uint16_t UV_Frequency_Value;
//          uint8_t UV_NorFrequency_MINValue;//unit:kHz
//          uint8_t UV_NorFrequency_MAXValue;
//  }TEKTeUV_t;

typedef struct
{

    uint8_t int_flag: 1;
    uint8_t detect_ok: 1;
    uint8_t res: 6;

    uint16_t err_cnt;
    uint16_t cnt;

} UV_TypeDef;
#endif
typedef struct
{
    TEKPower_t *power;
    //  TEKTime_t  *time;
#if HEPA_TIMEOUT
    TEKHepa_t  *hepa;
#endif
#if HEPA2_TIMEOUT
    TEKHepa_t  *hepa2;
#endif
    //    TEKHepa_t  *hepa3;
    TEKPM25_t  *pm25;
    TEKPM25_t   *pm10;
    //   TEKHumiditifier_t *humi;
    //   TEKLed_t    *led;
    //   TEKBuzzer_t *buz;
    TEKKey_t    *key;
    TEKFG_t     *fg;
    //   TEKZERO_t   *zero;
    //  TEKAir_t    *air;
    TEKPWM_t    *pwm;
    TEKTime_t  *time;
    uint8_t   auto_report;
    uint8_t   close_debug;

} TEKTe_t;
#endif

typedef enum
{
    FANSPEEDLEVEL_0 = 0,
#if FAN_SPEEDLEVEL>0
    FANSPEEDLEVEL_1,
#endif
#if FAN_SPEEDLEVEL>1
    FANSPEEDLEVEL_2,
#endif
#if FAN_SPEEDLEVEL>2
    FANSPEEDLEVEL_3,
#endif
#if FAN_SPEEDLEVEL>3
    FANSPEEDLEVEL_4,
#endif
#if FAN_SPEEDLEVEL>4
    FANSPEEDLEVEL_5,
#endif
#if FAN_SPEEDLEVEL>5
    FANSPEEDLEVEL_6,
#endif
} FANSPEEDLEVEL;

#ifdef __cplusplus
}
#endif
#endif
