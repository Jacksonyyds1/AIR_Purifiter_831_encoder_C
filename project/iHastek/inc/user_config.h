#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define MCU_MODEL                 "AT32F403A"

#define SWVERSION_BUILDNUM   "0302"//02 is the picture Bin Version
#define SWVERSION_MINOR          "03"
#define SWVERSION_MAJOR          "01"
#define SW_VERSION                "V"SWVERSION_MAJOR"."SWVERSION_MINOR"."SWVERSION_BUILDNUM

#define HW_VERSION                "A6"
#define DOC_VERSION               "A2"
#define DEV_MODEL                 "240180"             //MODEL������ʵ���޸ģ�TE�� 

#define DEV_VERSION               MCU_MODEL"_"SW_VERSION   //�汾�ţ�����ʵ���޸ģ�TE��

#define HARDWARE_VER  "HW-"HW_VERSION".00"
#define SOFTWARE_VER  "SW"SW_VERSION

#define  USE_COMPILE_TIME         1  //0����ʹ�ñ���ʱ��  1���򿪱���ʱ��  
#define  USER_COMPILE_TIME       "2025-04-21 19:30" //����ʱ�� ��Ϊ������ ����USE_COMPILE_TIME��Ϊ1��ʱ���ö����ʹ����Ϊ�����ʱ�䣬ע�͵�ʹ��ϵͳ����ʱ��

/****************************   debug       *************/
#define DEBUG_PRINT_ENABLE                  0

/****************************   POWER ON    *************/
#define USE_POWER                                       1

/****************************   OTA    *************/
#define USE_OTA                                     0

/****************************   SPEED    *************/
#define SPEED_LEVEL_MAX                     3//��λ��������˯�ߵ�λ
#define SPEED_LEVEL_SLEEP_ENABLE    0

/****************************   printf    *************/
#define DEBUG_PRINT                                 1
/****************************      TE       *************/
#define USE_TE                                          0
#if USE_TE
#define TE_UART_TX_BUF_LEN        64
#define TE_UART_RX_BUF_LEN        64
#endif

/****************************   DATA SAVE     *************/
#define USE_STATUS_WRITE                0//�ϵ������Ƿ񱣴�

/****************************      HEPA       *************/
#define HEPA_TIMEOUT              (219000UL*3600UL)  //����Ҫ��ע��
#define HEPA2_TIMEOUT             (4380UL*3600UL)  //����Ҫ��ע��
#define HEPA_WRITE_FLAG           (0X5A)
//#define USE_HEPA                                      1
//  #if USE_HEPA
//      #define HEPA_SAVE_TIME          3600        //unit ��
//      #define HEPA_TIMEOUT            4280        //unit Сʱ
//      #if HEPA_TIMEOUT
//          #define HEPA_DATA_BACKUP    0//
//      #endif
//      #define HEPA2_TIMEOUT           4100
//      #if HEPA2_TIMEOUT
//          #define HEPA2_DATA_BACKUP   0
//      #endif
//      //#define HEPA3_TIMEOUT         0
//      #if HEPA3_TIMEOUT
//          #define HEPA3_DATA_BACKUP   1
//      #endif
//  #endif

/****************************      KEY       ************/
#define  USE_KEY                                        1
#if USE_KEY
#define  USE_TOUCH_FUNCTION                         0
#define  KEY_DEBOUNCING                                 3
#define  KEY_PRESS_VALID_TIME   50  //���������ķ���ʱ��
#define  USE_KEY_RELEASE          1   // 0�� ��ʹ�õ�����  1�� ʹ�õ�����
#define  USE_KEY_TOUCH            1   // 0�� ��ʹ�ð��´�������  1�� ʹ�ð��´�������
#define KEY_LONG_PRESS_TIME         500//
#define KEY_SECOND_LONG_PRESS_TIME    5000
#define KEY_THIRDLY_LONG_PRESS    10000
#if USE_KEYACTION_RELEASE
#define  KEY_LONG_PRESS_TIME_NUM    0//���֧��5�ֲ�ͬʱ��
#if (KEY_LONG_PRESS_TIME_NUM>0)
#define  KEY_LONG_PRESS_TIME     1000    //MS  ����ʱ�� 
#if (KEY_LONG_PRESS_TIME_NUM>1)
#define KEY_LONG_PRESS_TIME2   2000 //�ڶ��ֳ���ʱ��
#endif
#if (KEY_LONG_PRESS_TIME_NUM>2)
#define KEY_LONG_PRESS_TIME3   3000 //�����ֳ���ʱ��
#endif
#if (KEY_LONG_PRESS_TIME_NUM>3)
#define KEY_LONG_PRESS_TIME4   4000 //�����ֳ���ʱ��
#endif
#if (KEY_LONG_PRESS_TIME_NUM>4)
#define KEY_LONG_PRESS_TIME5   5000 //�����ֳ���ʱ��
#endif
#endif
#endif
#endif

/****************************      UART       *************/
#define USE_UART_PROTOCOL                                0
#if USE_UART_PROTOCOL
#define UART_TX_BUF_LEN        20
#define UART_RX_BUF_LEN        2060U

#define USE_UART_SELF_PROTOCOL 1
#define UART_DELAY_TIME        100//UNIT MS
#endif

/****************************      FAN       *************/
#define USE_FAN_PWM                                     1
#if USE_FAN_PWM
#define USE_PWM             1
#define FAN_PWM_FRE         20000   //20KHz
#define FAN_PWM_DUTY_MAX    1000    //240MHz / FAN_PWM_FRE / 12K = 1000
#define FAN_PWMDUTY_ADD_SLOWLY  0
#define USE_FAN_FG              1
#if USE_FAN_FG
//#define USE_TIM_COUNTER       1//fg,uv detect
#define FAN_RPM_STABILIZE_DURINGTIME  1
#define FAN_FG_PLUSE_PER_CIRCLE       15
#define FAN_FG_RPM_TOLERATION          5
#define USE_FAN_SPEED_FIX              1
#endif
#endif

/****************************      UV       *************/
#define USE_UV                                          1
#if USE_UV
#define  UV_FREQUENCE_DETECT                    0
#if UV_FREQUENCE_DETECT
#ifndef USE_TIM_COUNTER
#define USE_TIM_COUNTER             1
#endif
#define UV_NORMAL_FREQUENCY_MIN     15000
#define UV_NORMAL_FREQUENCY_MAX     20000
#endif
#endif

/****************************      ION       *************/
#define USE_ION                                         1

/****************************      LOCK       *************/
#define USE_LOCK                                            0

/****************************      BUZZER   *************/
#define USE_BUZZER                                      0
#define BUZZER_PWM_FRE      4000

/****************************      TM1668       *************/
#define USE_TM1668                                      0

/****************************      TIMER       *************/
#define USE_TIMER                                       1
#define AUTO_OFF_STEP_SECONDS     60UL
/****************************      HUMIDIFIER       *************/
#define USE_HUMIDIFIER                                  0

/****************************      IIC       *************/
#define USE_IIC                                         0

/****************************      IR       *************/
#define USE_IR                                          1

#ifdef __cplusplus
}
#endif
#endif
