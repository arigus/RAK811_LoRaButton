/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32l1xx.h"
#include "stm32l1xx_hal.h"
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "gps-board.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#include "uart-board.h"
#include "button.h"
#include "partition.h"

#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif

#ifdef LORAWAN_DEBUGGER

#define DPRINTF(...)  printf(__VA_ARGS__) 
#else
#define DPRINTF(...)

#endif

/*!
 * Define indicating if an external IO expander is to be used
 */
//#define BOARD_IOE_EXT

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif


/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PB_13
#define RADIO_XTAL_EN                               PH_1

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5
#define RADIO_NSS                                   PB_0

#define RADIO_DIO_0                                 PA_11
#define RADIO_DIO_1                                 PB_1
#define RADIO_DIO_2                                 PA_3
#define RADIO_DIO_3                                 PH_0
#define RADIO_DIO_4                                 PC_13

#define RADIO_RF_CRX_RX                             PB_6  //CRF3
#define RADIO_RF_CBT_HF                             PB_7  //CRF2 HF 
#define RADIO_RF_CTX_PA                             PA_4  //CRF1 PA 

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

//#define USB_DM                                      PA_11
//#define USB_DP                                      PA_12

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define BAT_LEVEL_PIN                               PA_2
#define BAT_LEVEL_CHANNEL                           ADC_CHANNEL_2

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

/*!
 * Button pin define
 */
#define RAK811_PINS_MASK                            (0x076EE19E) //  byte0: 10011110
                                                                 //  byte1: 11100001
                                                                 //  byte2: 01101110
                                                                 //  byte3: 00000111

#define RAK811_ADC_PINS_MASK                        (0x0068000E) //  byte0: 00001110
                                                                 //  byte1: 00000000
                                                                 //  byte2: 01101000
                                                                 //  byte3: 00000000

#define RAK811_PIN2                                 PB_12       //ADC_IN18

#define RAK811_PIN3                                 PB_14      //ADC_IN20

#define RAK811_PIN4                                 PB_15       //ADC_IN21

#define RAK811_PIN5                                 PA_8

#define RAK811_PIN8                                 PA_12

#define RAK811_PIN9                                 PB_4
#define RAK811_PIN10                                PA_13
#define RAK811_PIN13                                PA_14

#define RAK811_PIN14                                 PA_15

#define RAK811_PIN15                                 PB_3

#define RAK811_PIN16                                 PB_5

#define RAK811_PIN18                                 PB_8  //I2C1_SCL

#define RAK811_PIN19                                 PB_9  //I2C1_SDA

#define RAK811_PIN20                                 PA_2  //ADC_IN2

#define RAK811_PIN22                                 PA_1  //ADC_IN1

#define RAK811_PIN23                                 PA_0  //ADC_IN0

#define RAK811_PIN25                                 PB_10

#define RAK811_PIN26                                 PB_11

#define RAK811_PIN27                                 PB_2


const static PinNames RAK811_pin_array[32] ={ NC, RAK811_PIN2, RAK811_PIN3, RAK811_PIN4, RAK811_PIN5, NC, NC, RAK811_PIN8,
                                              RAK811_PIN9, NC, NC, NC, NC, RAK811_PIN14, RAK811_PIN15, RAK811_PIN16,
                                              NC, RAK811_PIN18, RAK811_PIN19, RAK811_PIN20, NC, RAK811_PIN22, RAK811_PIN23, NC,
                                              RAK811_PIN25, RAK811_PIN26, RAK811_PIN27, NC, NC, NC, NC, NC};

const static uint32_t RAK811_adc_pin_map[32] ={0, ADC_CHANNEL_18, ADC_CHANNEL_20, ADC_CHANNEL_21, 0, 0, 0, 0,  //1-8
                                               0, 0, 0, 0, 0, 0, 0, 0,                                         //9-16
                                               0, 0, 0, ADC_CHANNEL_2, 0, ADC_CHANNEL_1, ADC_CHANNEL_0, 0,     //7-24
                                               0, 0, 0, 0, 0, 0, 0, 0};                                        //25-32



/*!
 * MCU objects
 */
extern Adc_t Adc;
extern I2c_t I2c;
extern Uart_t Uart1;
extern Uart_t GpsUart;

/*!
 * Possible power sources
 */
enum BoardPowerSources
{
    USB_POWER = 0,
    BATTERY_POWER,
};

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Measure the Battery voltage
 *
 * \retval value  battery voltage in volts
 */
uint32_t BoardGetBatteryVoltage( void );
uint16_t BoardBatteryMeasureVolage( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source [0: USB_POWER, 1: BATTERY_POWER]
 */
uint8_t GetBoardPowerSource( void );

void rw_ResetMCU(void);

int rw_LoRaTxData(bool confirm, uint8_t app_port, uint16_t app_len, uint8_t *app_data);

void dump_hex2str(uint8_t *buf, uint8_t len);

#endif // __BOARD_H__
