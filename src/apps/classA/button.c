#include <time.h>
#include <string.h>
#include "board.h"
#include "LoRaMac.h"

TimerEvent_t KeyHandleTimer;
TimerEvent_t FailLedTimer;
TimerEvent_t JoinLedTimer;


lora_system_t     g_lora_system;
extern DeviceState g_DeviceState;

static uint32_t sleep_TS = 0 ;
static uint32_t pwroff_TS = 0 ;
static uint32_t app_loop_TS = 0 ;

static  E_BAT_STATE bat_state = BAT_CONNECT;
static  uint8_t ledCtrlOn = 0;
static  uint8_t key_down = 0, key_valid = 0;
static  uint8_t bat_per = 0;
static  uint8_t appData[16];
static  uint8_t HandleKey = 0;

 uint8_t failClor = RED;
static  uint8_t sendFlg = 0; // 1 sending 0 done/idle

void setFutureEventTs(void);
void confirmed_success_cb(void);
void confirmed_fail_cb(void);
void join_fail_cb(void);
void join_success_cb(void);

Gpio_t  holdPower;
Adc_t   BatteryAdc;
Gpio_t  LED2Red;
Gpio_t  LED2Green;
Gpio_t  LED2Blue;

Gpio_t  LED3Red;
Gpio_t  LED3Green;
Gpio_t  LED3Blue;

Gpio_t  LED4Red;
Gpio_t  LED4Green;
Gpio_t  LED4Blue;

Gpio_t  LED5Red;
Gpio_t  LED5Green;
Gpio_t  LED5Blue;

Gpio_t  ButtonKey1;
Gpio_t  ButtonKey2;
Gpio_t  ButtonKey3;
Gpio_t  ButtonKey4;

void io_init1(void)
{
    /* hold power */
//    GPIO_Open(GPIOB, GPIO_PMD_PMD3_OUTPUT, GPIO_PMD_PMD3_MASK);
//    GPIO_EnablePullup(GPIOB, 3);
//    GPIO_SetBit(GPIOB, 3);
    
    GpioInit( &holdPower, RAK811_PIN26, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
      
    /* LED2 RED */
//    GPIO_Open(GPIOA, GPIO_PMD_PMD12_OUTPUT, GPIO_PMD_PMD12_MASK);
//    GPIO_EnablePullup(GPIOA, 12);
//    GPIO_SetBit(GPIOA, 12);
    
    GpioInit( &LED2Red, RAK811_PIN8, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    
}

void io_init2(void)
{
    //sys_jtag_off();
    
    /* ButtonKey1 */   
    GpioInit( &ButtonKey1, RAK811_PIN3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
          
    /* ButtonKey2 */
    GpioInit( &ButtonKey2, RAK811_PIN4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    
    /* ButtonKey3 */
    GpioInit( &ButtonKey3, RAK811_PIN9, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    
    /* key4 */
    GpioInit( &ButtonKey4, RAK811_PIN5, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    
    /* LED2 */
    /* RED */
    GpioInit( &LED2Red, RAK811_PIN8, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    
    /* GREEN */
    GpioInit( &LED2Green, RAK811_PIN10, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 1 );
    
    /* BLUE */
    GpioInit( &LED2Blue, RAK811_PIN13, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 1 );
    
    /* LED3 */
    /* RED */
    GpioInit( &LED3Red, RAK811_PIN14, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 1 );
    
    /* GREEN */
    GpioInit( &LED3Green, RAK811_PIN15, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 1 );
    
    /* BLUE */
    GpioInit( &LED3Blue, RAK811_PIN16, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    
    /* LED4 */
    /* RED */ 
    GpioInit( &LED4Red, RAK811_PIN18, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    
    /* GREEN */
    GpioInit( &LED4Green, RAK811_PIN19, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    /* BLUE */
    GpioInit( &LED4Blue, RAK811_PIN20, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
 
    /* LED5 */
    /* RED */ 
    GpioInit( &LED5Red, RAK811_PIN22, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        
    /* GREEN */
    GpioInit( &LED5Green, RAK811_PIN23, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    
    /* BLUE */
    GpioInit( &LED5Blue, RAK811_PIN25, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    
}

void pwr_on()
{
    GpioWrite( &holdPower, 1); 
}

void pwr_off()
{
    printf("Pwr Off\r\n");
    DelayMs(10);
    
#if 1
    GpioWrite( &holdPower, 0); 
    DelayMs(5000);
    GpioWrite( &holdPower, 1); 
#endif
    
}

uint8_t read_key_value(uint8_t Key)
{
  uint8_t keyVal = 0; 
    
  if (Key == KEY1) {
      keyVal = GpioRead( &ButtonKey1); 
  }
  else if (Key == KEY2) {
      keyVal = GpioRead( &ButtonKey2); 
  }
  else if (Key == KEY3) {
      keyVal = GpioRead( &ButtonKey3); 
  }
  else if (Key == KEY4) {
      keyVal = GpioRead( &ButtonKey4); 
  } else {
  
  }
  return keyVal;
}

uint8_t read_key1_value()
{
  return GpioRead( &ButtonKey1);
}

uint8_t read_key2_value()
{
  return GpioRead( &ButtonKey2);
}

uint8_t read_key3_value()
{
  return GpioRead( &ButtonKey3);
}

uint8_t read_key4_value()
{
  return GpioRead( &ButtonKey4);
}

uint8_t read_charge_value()
{
  return GpioRead( &LED3Red);
}

uint8_t read_standby_value()
{
  return GpioRead( &LED3Green);
}

uint32_t getTimeStampMS()
{
    return TimerGetCurrentTime();
}


//key scan interval 20 ms
#define SHORT_PRESS_NUM_LIMIT   (2)
#define LONG_PRESS_NUM_LIMIT    (250)  //  above 5S

void KeyEvent_NotifyCB(uint8_t key, E_KEY_EVENTS event);

void key_scan(void)
{
    static  uint8_t key_hold_num[KEY_MAX]={0};
           
    for(uint8_t i=KEY1; i<KEY_MAX; i++)
    {
      if (read_key_value(i) == 0) {
          key_hold_num[i]++;
          if (key_hold_num[i] >= LONG_PRESS_NUM_LIMIT) {
              KeyEvent_NotifyCB(i, KEY_EVENT_LONG_PRESS);
          }
      } else {
          if (key_hold_num[i] >= SHORT_PRESS_NUM_LIMIT) {
              KeyEvent_NotifyCB(i, KEY_EVENT_SHORT_PRESS);
              key_hold_num[i] = 0;
          } else {
            key_hold_num[i] = 0;
          }
      } 
    }
}

void led_ctrl(uint8_t led_num, uint8_t rgb)
{
    /**** update battery status only when led all off ***/
    if (rgb == OFF) {
      ledCtrlOn = 0;
    } else {
      ledCtrlOn = 1;
    }
    
    switch(led_num) {
        case LED2:
        if(rgb == RED) {
            GpioWrite( &LED2Red, 0); 
            GpioWrite( &LED2Green, 1);
            GpioWrite( &LED2Blue, 1);
        } 
        else if(rgb == GREEN) {
            GpioWrite( &LED2Red, 1); 
            GpioWrite( &LED2Green, 0);
            GpioWrite( &LED2Blue, 1);
        } 
        else if(rgb == BLUE) {
            GpioWrite( &LED2Red, 1); 
            GpioWrite( &LED2Green, 1);
            GpioWrite( &LED2Blue, 0);
        } 
        else if(rgb == YELLOW) {
            GpioWrite( &LED2Red, 0); 
            GpioWrite( &LED2Green, 0);
            GpioWrite( &LED2Blue, 1);
        } 
        else if(rgb == PURPLE) {
            GpioWrite( &LED2Red, 0); 
            GpioWrite( &LED2Green, 1);
            GpioWrite( &LED2Blue, 0);
        } 
        else if(rgb == CYAN) {
            GpioWrite( &LED2Red, 1); 
            GpioWrite( &LED2Green, 0);
            GpioWrite( &LED2Blue, 0);
        }
        else if(rgb == OFF) {
            GpioWrite( &LED2Red, 1); 
            GpioWrite( &LED2Green, 1);
            GpioWrite( &LED2Blue, 1);
        }
        break;
        
        case LED3:
        if(rgb == RED) {
            if (GpioRead( &LED3Red)) {
               GpioWrite( &LED3Red, 0);
            }
            GpioWrite( &LED3Green, 1);
            GpioWrite( &LED3Blue, 1);
        } 
        else if(rgb == GREEN) {
            GpioWrite( &LED3Red, 1);
            if (GpioRead( &LED3Green)) {
                GpioWrite( &LED3Green, 0);
            }
            GpioWrite( &LED3Blue, 1);
        } 
        else if(rgb == BLUE) {
            GpioWrite( &LED3Red, 1);
            GpioWrite( &LED3Green, 1);
            GpioWrite( &LED3Blue, 0);
        } 
        else if(rgb == YELLOW) {
            if (GpioRead( &LED3Red)) {
               GpioWrite( &LED3Red, 0);
            }
            if (GpioRead( &LED3Green)) {
                GpioWrite( &LED3Green, 0);
            }
            GpioWrite( &LED3Blue, 1);
        }
        else if(rgb == PURPLE) {
            if (GpioRead( &LED3Red)) {
               GpioWrite( &LED3Red, 0);
            }
            GpioWrite( &LED3Green, 1);
            GpioWrite( &LED3Blue, 0);
        }
        else if(rgb == CYAN) {
            GpioWrite( &LED3Red, 1);
            if (GpioRead( &LED3Green)){
                GpioWrite( &LED3Green, 0);
            }
            GpioWrite( &LED3Blue, 0);
        }
        else if(rgb == OFF) {
             GpioWrite( &LED3Red, 1);
             GpioWrite( &LED3Green, 1);
             GpioWrite( &LED3Blue, 1);
        }
        break;
        
        case LED4:
        if(rgb == RED) {
             GpioWrite( &LED4Red, 0);
             GpioWrite( &LED4Green, 1);
             GpioWrite( &LED4Blue, 1);
        } 
        else if(rgb == GREEN) {
             GpioWrite( &LED4Red, 1);
             GpioWrite( &LED4Green, 0);
             GpioWrite( &LED4Blue, 1);
        } 
        else if(rgb == BLUE) {
             GpioWrite( &LED4Red, 1);
             GpioWrite( &LED4Green, 1);
             GpioWrite( &LED4Blue, 0);
        } 
        else if(rgb == YELLOW) {
             GpioWrite( &LED4Red, 0);
             GpioWrite( &LED4Green, 0);
             GpioWrite( &LED4Blue, 1);
        } 
        else if(rgb == PURPLE) {
             GpioWrite( &LED4Red, 0);
             GpioWrite( &LED4Green, 1);
             GpioWrite( &LED4Blue, 0);
        } 
        else if(rgb == CYAN) {
             GpioWrite( &LED4Red, 1);
             GpioWrite( &LED4Green, 0);
             GpioWrite( &LED4Blue, 0);
        } 
        else if(rgb == OFF) {
             GpioWrite( &LED4Red, 1);
             GpioWrite( &LED4Green, 1);
             GpioWrite( &LED4Blue, 1);
        }
        break;
        
        case LED5:
        if(rgb == RED) {
             GpioWrite( &LED5Red, 0);
             GpioWrite( &LED5Green, 1);
             GpioWrite( &LED5Blue, 1);
        } 
        else if(rgb == GREEN) {
             GpioWrite( &LED5Red, 1);
             GpioWrite( &LED5Green, 0);
             GpioWrite( &LED5Blue, 1);
        } 
        else if(rgb == BLUE) {
             GpioWrite( &LED5Red, 1);
             GpioWrite( &LED5Green, 1);
             GpioWrite( &LED5Blue, 0);
        } 
        else if(rgb == YELLOW) {
             GpioWrite( &LED5Red, 0);
             GpioWrite( &LED5Green, 0);
             GpioWrite( &LED5Blue, 1);
        } 
        else if(rgb == PURPLE) {
             GpioWrite( &LED5Red, 0);
             GpioWrite( &LED5Green, 1);
             GpioWrite( &LED5Blue, 0);
        } 
        else if(rgb == CYAN) {
             GpioWrite( &LED5Red, 1);
             GpioWrite( &LED5Green, 0);
             GpioWrite( &LED5Blue, 0);
        } 
        else if(rgb == OFF) {
             GpioWrite( &LED5Red, 1);
             GpioWrite( &LED5Green, 1);
             GpioWrite( &LED5Blue, 1);
        }
        break;
        
        default:
        break;
    }
}


void led_ctrl_all(uint8_t rgb)
{
  led_ctrl(LED2, rgb);
  led_ctrl(LED3, rgb);
  led_ctrl(LED4, rgb);
  led_ctrl(LED5, rgb);   
}

void led_ctrl_toggle(uint8_t led_num)
{
  static uint8_t led2 = 0, led3 = 0, led4 = 0, led5 = 0;
    
  switch(led_num) {
        case LED2:
          {
            led_ctrl(LED2, led2);
            led2 ++;
            if (led2 == COLOR_MAX)
              led2 = 0;
          }
          break;
        case LED3:
          {
            led_ctrl(LED3, led3);
            led3 ++;
            if (led3 == COLOR_MAX)
              led3 = 0;
          }
          break;  
        case LED4:
          {
            led_ctrl(LED4, led4);
            led4 ++;
            if (led4 == COLOR_MAX)
              led4 = 0;
          }
          break; 
        case LED5: 
         {
            led_ctrl(LED5, led5);
            led5 ++;
            if (led5 == COLOR_MAX)
              led5 = 0;
          }
          break;
        default:
        break;
  }
}


void test_handler(tester_t *tester)
{
    //uint16_t xiaodou1=0, xiaodou2=0, xiaodou3=0, xiaodou4= 0;
    uint16_t led_cnt = 0;
    bool first_key = true;
    uint8_t key_down=0, key_valid=0;
    
    while(1) {
        
        if(led_cnt == 50) {
            led_ctrl_all(RED);
        }
        else if(led_cnt == 100) { //led_cnt >= 50 && led_cnt < 100
            led_ctrl_all(GREEN);            
        }
        else if(led_cnt == 150) {  //led_cnt >= 100 && led_cnt < 150
            led_ctrl_all(BLUE); 
        }
        
        led_cnt++;
        if(led_cnt >= 151) {
            led_cnt = 0;
        }
#if 1      
        if(read_key1_value() == 0) {
            if(first_key) {
                first_key = false;
            } else {
                led_ctrl(LED2, OFF);
                key_down |= (1<<0);
            }
        } else {
            if(key_down&(1<<0)) {
                key_valid |= (1<<0);
                key_down &= ~(1<<0);
                printf("Key1 press\r\n");
            }
        }
        
        if(read_key2_value() == 0) {
            led_ctrl(LED4, OFF);
            key_down |= (1<<1);
        } else {
            if(key_down&(1<<1)) {
                key_valid |= (1<<1);
                key_down &= ~(1<<1);
                printf("Key2 press\r\n");
            }
        }
        if(read_key3_value() == 0) {
            led_ctrl(LED5, OFF);
            key_down |= (1<<2);
        } else {
            if(key_down&(1<<2)) {
                key_valid |= (1<<2);
                key_down &= ~(1<<2);
                printf("Key3 press\r\n");
            }
        }
        if(read_key4_value() == 0) {
            led_ctrl(LED3, OFF);
            key_down |= (1<<3);
        } else {
            if(key_down&(1<<3)) {
                key_valid |= (1<<3);
                key_down &= ~(1<<3);
                printf("Key4 press\r\n");
            }
        }
#endif        
        DelayMs(20);
        
        if(key_valid == 0x0f) {
            memset(tester->key_led_en, 0xff, 16);
            //FLASH_SafeEraseSector((image_use==IMG1_USE?FLASH_ADDR_TESTER_EN1:FLASH_ADDR_TESTER_EN2), FLASH_SIZE_SECTOR);
            //FLASH_WriteBuffer((uint8_t *)tester, (image_use==IMG1_USE?FLASH_ADDR_TESTER_EN1:FLASH_ADDR_TESTER_EN2), sizeof(tester_t));
            break;
        }
    }
    
    led_ctrl_all(OFF); 
    pwr_off();
    
    while(1) {
        DelayMs(1000);
    }
}

void adc_init(void)
{     
   AdcInit( &BatteryAdc, RAK811_PIN2);
}

void adc_deinit(void)
{ 
   AdcDeInit(&BatteryAdc);
}

int adc_read_val(void)
{ 
   return AdcReadChannel( &BatteryAdc, ADC_CHANNEL_18 );
}

int GetMedianNum(int * bArray, int iFilterLen)
{
  int i,j;// 循环变量
  int bTemp;
  
  // 用冒泡法对数组进行排序
  for (j = 0; j < iFilterLen - 1; j ++)
  {
    for (i = 0; i < iFilterLen - j - 1; i ++)
    {
      if (bArray[i] > bArray[i + 1])
      {
        // 互换
        bTemp = bArray[i];
        bArray[i] = bArray[i + 1];
        bArray[i + 1] = bTemp;
      }
    }
  }
  
#if 0
  for(i=0;i<iFilterLen;i++) {
	  printf("%x ", bArray[i]);
  }
#endif
  // 计算中值
  if ((iFilterLen & 1) > 0)
  {
    // 数组有奇数个元素，返回中间一个元素
    bTemp = bArray[(iFilterLen + 1) / 2];
  }
  else
  {
    // 数组有偶数个元素，返回中间两个元素平均值
    bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 - 1]) / 2;
    
#if 0
    printf("med=%x\r\n", bTemp);
#endif
  }
  
  return bTemp;
}

int get_adc_value(int32_t *adc_int, uint16_t *adc_hex)
{
    int adc_buf[ADC_MEASURE_NUM];   
    memset(adc_buf, 0, sizeof(adc_buf));

    adc_init();
    DelayMs(50);
    
#if 0
#if ADC_CALIBRATION
	sys_adc_calibration(0, &offset, &gain);
	printf("ADC:offset = 0x%x, gain = 0x%x\n", offset, gain);
	if((offset==0xFFFF) || (gain==0xFFFF))
#endif
    {
        offset = OFFSET;
        gain = GAIN_DIV;
        printf("ADC:offset = 0x%x, gain = 0x%x\n", offset, gain);
    }
#endif
    
    for(int i=0; i < ADC_MEASURE_NUM; i++) {      
        adc_buf[i] = adc_read_val();
        //DPRINTF("AD2:%x\n", adc_buf[i]);
        DelayMs(10);
    }

    if(adc_hex) {
        *adc_hex = GetMedianNum(adc_buf, ADC_MEASURE_NUM);
        //DPRINTF("AD2:%x\n", *adc_hex);
    }
     
    if(adc_int) {
        *adc_int = AD2MV(*adc_hex);    
        printf("AD2=%d mv\n", *adc_int);
    } 
    
    adc_deinit();
    
    return 0;
}

int get_battery()
{
    int32_t adc_int = 0;
    uint16_t adc_hex = 0;
    uint8_t battery_percent = 0;
    
    get_adc_value(&adc_int, &adc_hex);
    adc_int = adc_int*3/2;
    
    if(adc_int < 3450) {                          //3.45
        battery_percent = 10;
    } else if(adc_int >= 3450 && adc_int < 3606) {  //3.53
        battery_percent = 20;
    } else if(adc_int >= 3606 && adc_int < 3645) {  //3.62
        battery_percent = 30;
    } else if(adc_int >= 3645 && adc_int < 3674) {  //3.66
        battery_percent = 40;
    } else if(adc_int >= 3674 && adc_int < 3710) {  //3.69
        battery_percent = 50;
    } else if(adc_int >= 3710 && adc_int < 3762) {  //3.73
        battery_percent = 60;
    } else if(adc_int >= 3762 && adc_int < 3818) {  //3.79
        battery_percent = 70;
    } else if(adc_int >= 3818 && adc_int < 3894) {  //3.85
        battery_percent = 80;
    } else if(adc_int >= 3894 && adc_int < 3993) {  //3.94
        battery_percent = 90;
    } else if(adc_int >= 3993 && adc_int < 4153) {  //4.07
        battery_percent = 100;
    } else if(adc_int >= 4153) {
        battery_percent = 100;
    }
    return battery_percent;
}

void fail_Led_handle(void)
{   
  static uint8_t flash_num = 0;
  static uint8_t toggle = 0;
  
  TimerStop( &FailLedTimer );
  led_ctrl_all(OFF);
      
  if (flash_num++ < 6) {
    if ( toggle ) {
      led_ctrl_all(failClor);
      toggle = 0;
    } else {
      led_ctrl_all(OFF); 
      toggle = 1;
    }
    TimerSetValue( &FailLedTimer, 200 );
    TimerStart( &FailLedTimer );      
  } else {
    flash_num = 0;
    toggle = 0;
  }
}

void join_Led_handle(void)
{   
  static uint8_t toggle = 0;
  
  TimerStop( &JoinLedTimer );
  led_ctrl_all(OFF);
  
  if (g_lora_system.IsJoined == 0) {
    if ( toggle ) {
      led_ctrl_all(RED);
      toggle = 0;
    } else {
      led_ctrl_all(OFF); 
      toggle = 1;
    }
    TimerSetValue( &JoinLedTimer, 1000 );
    TimerStart( &JoinLedTimer ); 
  } else {
    toggle = 0;
  }
}


void key_handle(void)
{   
    int ret;
    int32_t Bat_val;
    Bat_val = get_battery();
    printf("Battery state: %d , per: %d%%\r\n", bat_state, Bat_val);
    
    if (g_lora_system.IsJoined == 0) {
    	printf("Not joined\r\n");
       g_DeviceState = DEVICE_STATE_JOIN;
       return;
    }
    appData[0] = 0x53;
    appData[1] = 0x01;
    appData[2] = (HandleKey ==1); // key1 ch 1
    appData[3] = (HandleKey ==2);
    appData[4] = (HandleKey ==3);
    appData[5] = (HandleKey ==4);
    appData[6] = (bat_state == BAT_CHARGING); 
    appData[7] = Bat_val;      
    ret =rw_LoRaTxData(1, 8, 8, appData);
    if (ret != LORAMAC_STATUS_OK) {
    	printf("Tx Err: %d", ret);
      confirmed_fail_cb( );
    } else {
    	printf("Tx OK: ");
      dump_hex2str(appData , 8);
      sendFlg = 1;
    }
      
}


void setFutureEventTs(void) 
{
  pwroff_TS = getTimeStampMS() + 60*1000;
  sleep_TS  = getTimeStampMS() + 2000;
}

void check_battery_status(void)
{
    uint8_t charge_val, standby_val;
    
    if ( ledCtrlOn ) {
       return ;
    }
    
    charge_val  = read_charge_value();
    standby_val = read_standby_value();
    
    if ( charge_val && standby_val ) {
       bat_state = BAT_CONNECT;
    } else if ( (charge_val== 0) && standby_val) {
       bat_state = BAT_CHARGING;
    } else if ( (standby_val== 0) && charge_val) {
       bat_state = BAT_CHARGE_DONE;
    } else {
       bat_state = BAT_UNKNOWN;
    }
}

void ResetLedFlash(uint8_t color)
{
   for(uint8_t i=0; i<3; i++)
   {
     led_ctrl_all(color);
     DelayMs(150);
     led_ctrl_all(OFF);
     DelayMs(150);
   }
}

void KeyEvent_NotifyCB(uint8_t key, E_KEY_EVENTS event)
{

  if (key == KEY1 && event == KEY_EVENT_SHORT_PRESS) {
	  printf("Key1 press\r\n");
    if (g_lora_system.IsJoined) {
      led_ctrl_all(RED);
      failClor = RED;
    }
    setFutureEventTs();
    HandleKey = KEY1;
    TimerSetValue( &KeyHandleTimer, 5);
    TimerStart( &KeyHandleTimer ); 
  }
  else if (key == KEY2 && event == KEY_EVENT_SHORT_PRESS)
  {
	  printf("Key2 press\r\n");
    if (g_lora_system.IsJoined) {
      led_ctrl_all(GREEN);
      failClor = GREEN;
    }
    setFutureEventTs();
    HandleKey = KEY2;
    TimerSetValue( &KeyHandleTimer, 5);
    TimerStart( &KeyHandleTimer ); 
  }
  else if (key == KEY3 && event == KEY_EVENT_SHORT_PRESS)
  {
	  printf("Key3 press\r\n");
    if (g_lora_system.IsJoined) {
      led_ctrl_all(BLUE); 
      failClor = BLUE;
    }
    setFutureEventTs();
    HandleKey = KEY3;
    TimerSetValue( &KeyHandleTimer, 5 );
    TimerStart( &KeyHandleTimer );     
  }
  else if (key == KEY4 && event == KEY_EVENT_SHORT_PRESS)
  {
	  printf("Key4 press\r\n");
    if (g_lora_system.IsJoined) {
      led_ctrl_all(YELLOW);
      failClor = YELLOW;
    }
    setFutureEventTs();
    HandleKey = KEY4;
    TimerSetValue( &KeyHandleTimer, 5 );
    TimerStart( &KeyHandleTimer );     
  } else {
  
  }
  
  if (event == KEY_EVENT_LONG_PRESS) {
	  printf("Key%d long Reset...\r\n", key);
    ResetLedFlash(key-1);
    rw_ResetMCU();
  }

    
}


void lorabutton_app_loop(void)
{
    if ( getTimeStampMS() < app_loop_TS ) {
      return;
    }
    app_loop_TS = getTimeStampMS() + 20 ;
     
    /****** read battery state *****/
    check_battery_status();
    
    /****** read keys state *****/
    key_scan( );
      
    /****** check power off time if only battery suply *****/
    if ( (getTimeStampMS() > pwroff_TS) && (bat_state == BAT_CONNECT)) {
        led_ctrl_all(OFF); 
        setFutureEventTs(); 
        pwr_off();  
    }

}


void lorabutton_init(void)
{
    io_init1();
    io_init2();   
}

void lorabutton_param_init(void)
{
    TimerInit( &KeyHandleTimer, key_handle);
    TimerInit( &FailLedTimer, fail_Led_handle);
    TimerInit( &JoinLedTimer, join_Led_handle);

    app_loop_TS =  getTimeStampMS();
    setFutureEventTs();
}


void join_success_cb(void)
{
	printf("Join Success\r\n");
  g_lora_system.IsJoined = 1;
  g_DeviceState = DEVICE_STATE_SEND;
  TimerStop( &JoinLedTimer );
  led_ctrl_all(OFF);
}

void join_fail_cb(void)
{
	printf("Fail\r\n");
  g_DeviceState = DEVICE_STATE_SLEEP;
  TimerStop( &JoinLedTimer ); 
  led_ctrl_all(OFF); 
  TimerSetValue( &FailLedTimer, 200 );
  TimerStart( &FailLedTimer ); 
  sendFlg = 0;  
}

void confirmed_fail_cb(void)
{ 
	printf("Fail\r\n");
  led_ctrl_all(failClor); 
  TimerSetValue( &FailLedTimer, 200 );
  TimerStart( &FailLedTimer ); 
  sendFlg = 0;
}

void confirmed_success_cb(void)
{
	printf("Confirmed Success\r\n");
  led_ctrl_all(OFF);
  sendFlg = 0;
}
