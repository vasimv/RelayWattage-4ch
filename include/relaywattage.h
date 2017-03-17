/* High power AC relays + wattage measurement module based on STM32F373CBT6, 4 channels version
 *
 *    Copyright (C) 2017  Vasim V.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with this program. */

// Dump debug information through USB virtual com-port (may create weird effects, DON'T USE if you don't need)
//#define USB_DEBUG

// Debug information through serial port
#define SERIAL_DEBUG

#ifdef USB_DEBUG
// Extended debug (very noisy and may hung up)
#define EXTENDED_DEBUG
#endif

#if defined(SERIAL_DEBUG) || defined(USB_DEBUG)
#define DEBUG
#endif

// Collect and print out data for some period (length of measures)
// #define DEBUG_COLLECT 2048

// Divider for AC voltage calculation (transformer+resistor divider ratio)
#define AC_DIVIDER 10

// Milliwolts per Amper (*10)
#define CURRENT_MV_PER_AMP 132

// Write configuration to flash after last change (in milliseconds)
#define UPDATE_FLASH_AFTER 60000

// AC channels used
#define MY_AC_CHANNELS 4

// Total SDADC2 channels
#define MY_SDADC_CHANNELS 2

// Total ADC1 channels
#define MY_ADC_CHANNELS 11

// Maximum command length
#define MAX_COMMAND_LENGTH 16

// Minimum ACS759 ADC value counted as "zero" for averaging
#define MIN_ADC_ZERO 2000

// Maximum ACS759 ADC value counted as "zero" for averaging
#define MAX_ADC_ZERO 2100

// Size of buffer for multiple ADC readings (averaging)
#define MAX_ADC_COUNTS 16

// GPIO of relays and stuff
#define CHN0_PINA_PORT	GPIOF
#define CHN0_PINA_PIN	GPIO_PIN_7
#define CHN0_PINB_PORT	GPIOF
#define CHN0_PINB_PIN	GPIO_PIN_6
#define CHN0_ADC_NUM	0

#define CHN1_PINA_PORT	GPIOD
#define CHN1_PINA_PIN	GPIO_PIN_8
#define CHN1_PINB_PORT	GPIOA
#define CHN1_PINB_PIN	GPIO_PIN_8
#define CHN1_ADC_NUM	1

#define CHN2_PINA_PORT	GPIOB
#define CHN2_PINA_PIN	GPIO_PIN_7
#define CHN2_PINB_PORT	GPIOB
#define CHN2_PINB_PIN	GPIO_PIN_6
#define CHN2_ADC_NUM	2

#define CHN3_PINA_PORT	GPIOB
#define CHN3_PINA_PIN	GPIO_PIN_15
#define CHN3_PINB_PORT	GPIOB
#define CHN3_PINB_PIN	GPIO_PIN_14
#define CHN3_ADC_NUM	3

#define ALARM_PORT		GPIOB
#define ALARM_PIN		GPIO_PIN_2

#define TESTBTN_PORT	GPIOC
#define TESTBTN_PIN		GPIO_PIN_15


// Defined in main.c
extern SDADC_HandleTypeDef hsdadc1;
extern SDADC_HandleTypeDef hsdadc2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart1;

#ifdef USB_DEBUG
#include "usbd_def.h"
// Defined in usb_device.c
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

#define VREF_CORRECTION 3300

// STM32F37x datasheet addresses
//Temperature sensor raw value at 30 degrees C, VDDA=3.3V
#define TS_CAL1 (*((uint16_t *) 0x1FFFF7B8))
//Temperature sensor raw value at 110 degrees C, VDDA=3.3V
#define TS_CAL2 (*((uint16_t *) 0x1FFFF7C2))
//Internal voltage reference raw value at 30 degrees C, VDDA=3.3V
#define VREFINT_CAL (*((uint16_t *) 0x1FFFF7BA))

// Debug output routine (over USB if USB_DEBUG defined or serial port if SERIAL_DEBUG)
void debug(char *fmt, ...);
// Initialization (after HAL inits)
void Setup();
// Main loop
void loop();
// Measure interrupt routine
void measure_int();
// low priority interrupt routine (reports wattage, sets/resets relays periodically)
void timer_int();
