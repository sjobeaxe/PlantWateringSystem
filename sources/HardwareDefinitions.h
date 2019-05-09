/**
 * \file HardwareDefinitons.h
 * \brief Defines hardware related constants
 *
 * Originator: Anders Sjoberg
 * Creation date: 2017-07-05
 *
 * Definitions for various hardware connected to MCU and 
 * other hardware related constants.
 */

#ifndef HARDWARE_DEFINITIONS_H
#define	HARDWARE_DEFINITIONS_H
 
#include <xc.h>
#include <stdint.h>

/**
 * \def HW_OSC_FREQ
 * \brief The configured oscillator speed.
 * 
 * This project uses the internal oscillator of the MCU.
 * See HardwareInitialize(). Configured to 16 Mhz
 */
#define HW_OSC_FREQ 8000000UL

/**
 * \def HW_OSC_PLL_MULT
 * \brief The PLL multiplier, which is either 1 (off) or 4 (on).
 */
#define HW_OSC_PLL_MULT 1UL

/**
 * \def HW_CLOCK_FREQ
 * \brief This defines the final MCU core clock speed.
 */
#define HW_CLOCK_FREQ ( HW_OSC_PLL_MULT * HW_OSC_FREQ )

/**
 * \def LED_PIN
 * \brief General purpose debugging/indicator LED
 */
#define LED_PIN LATAbits.LA0

/**
 * \def LCD_BACKLIGHT
 * \brief Controls the LCD backlight on or off.
 */
#define LCD_BACKLIGHT_PIN LATAbits.LA4

/**
 * \def RELAY_PIN
 * \brief Controls the relay output driver.
 */
#define RELAY_PIN LATCbits.LC3

/**
 * \def CHARGER_PIN
 * \brief Controls the constant current source on or off.
 */
#define CHARGER_PIN LATCbits.LC5

/**
 * \def USB_UART_BAUD
 * \brief Defines the baudrate of serial port connected to USB.
 */
#define USB_UART_BAUD 57600UL
#define UART1_BRG16BHS (( (HW_CLOCK_FREQ) / (4UL * USB_UART_BAUD) ) -1)

/**
 * \def BUTTON_UP
 * \brief User-interface button.
 */
#define BUTTON_UP !PORTBbits.RB3

/**
 * \def BUTTON_MIDDLE
 * \brief User-interface button.
 */
#define BUTTON_MIDDLE !PORTBbits.RB4

/**
 * \def BUTTON_DOWN
 * \brief User-interface button.
 */
#define BUTTON_DOWN !PORTBbits.RB5

/**
 * \def ADC_CHANNEL_TEMP
 * \brief ADC channel for temperature sensor
 */
#define ADC_CHANNEL_TEMP 3

/**
 * \def ADC_CHANNEL_INPUT
 * \brief ADC channel for input/charger voltage
 */
#define ADC_CHANNEL_INPUT 16

/**
 * \def ADC_CHANNEL_BATTERY
 * \brief ADC channel for battery voltage
 */
#define ADC_CHANNEL_BATTERY 2

/**
 * \def ADC_CHANNEL_WATER_SENSOR
 * \brief ADC channel for battery voltage
 */
#define ADC_CHANNEL_WATER_SENSOR 12



#endif	/* HARDWARE_DEFINITIONS_H */
