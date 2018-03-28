#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>


#define LED2        2
#define LED3        3
#define LED4        4
#define LED5        5

#define KEY_NONE        0
#define KEY1            1
#define KEY2            2
#define KEY3            3
#define KEY4            4
#define KEY_MAX         5

typedef enum e_KeyEvent{
    KEY_EVENT_SHORT_PRESS = 1,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_DOUBLE_PRESS,
    KEY_EVENT_MAX,
}E_KEY_EVENTS;

enum U_COLOR{
    RED = 0,
    GREEN,
    BLUE,
    YELLOW,//R+G
    PURPLE,//R+B ×ÏÉ«
    CYAN,//B+G ÇàÉ«
    OFF,
    COLOR_MAX,
};

typedef enum e_BatState{
    BAT_CONNECT = 0,
    BAT_CHARGING,
    BAT_CHARGE_DONE,
    BAT_UNKNOWN,
}E_BAT_STATE;


typedef struct {
    uint8_t key_led_en[16];
    uint8_t adc_clib_en[16];
}tester_t;

typedef struct{
    uint8_t  IsJoined;
    uint8_t  DeviceState;
    uint8_t  recv_rssi_en;
    uint16_t up_cnt;
    uint16_t down_cnt;
}lora_system_t;

typedef enum eDeviceState{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

#define ADC_MEASURE_NUM     10
							
#define AD2MV(ad) ((ad*33*1000/4096)/10)

void lorabutton_init(void);
void lorabutton_param_init(void);
void lorabutton_app_loop(void);
void led_ctrl_all(uint8_t rgb);
void join_success_cb(void);
void join_fail_cb(void);
void confirmed_success_cb(void);
void confirmed_fail_cb(void);
#endif
