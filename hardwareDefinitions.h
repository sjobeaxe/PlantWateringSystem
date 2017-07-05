/**
 * \file hardwareDefinitons.h
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

// MCU clock speed, using internal oscillator, see HardwareInitialize()
#define HW_OSC_FREQ 16000000UL // 16 Mhz
#define HW_OSC_PLL_MULT 1UL // 4 if PLL enabled

// Clock speed is the oscillator multiplied by PLL
#define HW_CLOCK_FREQ ( HW_OSC_PLL_MULT * HW_OSC_FREQ )

// UART speed settings (TODO: implement UART routines)
#define USB_UART_BAUD 57600
#define USB_BRG16BHS (((OSC_FREQ) / (4UL * (SHW_UART_BAUD))) -1)


// LED connected directly to MCU I/O
#define LED LATAbits.LA0

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (HS oscillator (medium power 4-16 MHz))
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (Power up timer enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))


#endif	/* HARDWARE_DEFINITIONS_H */
