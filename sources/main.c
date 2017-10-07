/*
 * File:   main.c
 * Author: sjobe
 *
 * Created on 3. heinäkuuta 2017, 21:08
 */


#include "main.h"
#include "ConfigurationBits.h"
#include "HardwareDefinitions.h"
#include "TickTimer.h"
#include "RtcSwTimer.h"
#include "LcdDriver.h"
#include <stdio.h>

void main(void)
{
    uint8_t buff20[20]; // LCD buffer
    HardwareInitialize();
    TickTimerInit();
    RtcInit();
    
    /**
     * LCD seems to need a delay before it can be initialized.
     * Testing on a LCD it seemed 10 ms was enough, double that 
     * for good measure. Also after init, LCD seems to needs a 
     * short delay too.
     */
    BlockingDelay(20);
    LcdInit();
    
    /** LCD needs a while after init. */
    BlockingDelay(1);
    LcdWrite("PlantWateringSys"); // Splash-screen!
    
    LCD_BACKLIGHT_PIN = 1;
    
    /**
     * Start of the infinite super-loop.
     * This is the "operating system" of this project
     */
    while(1)
    {
      if (0 == TickTimerGet(T_PERIODIC_1S))
      {
        TickTimerSet(T_PERIODIC_1S, tickMs(1000));
      
        LED_PIN = !LED_PIN;
        
        /** Update clock - Just for debug */
        LcdGoto(LCD_LINE2);
        sprintf(buff20,"%02i:%02i:%02i", RtcTime.hour, RtcTime.min, RtcTime.sec);
        LcdWrite(buff20);
      }
    }
	
    return;
}

/**
 * \fn void putch(char data)
 * \brief Transmits one char on the USB UART.
 * \param data character to send
 *  
 * Function which defines where printf prints data. 
 * 
 * TODO: Make this selectable between UART1 and UART2
 */
void putch(char data)
{
    // Block while previous transmitting
    while (!PIR1bits.TX1IF);
    TXREG1 = data;
    return;
}

/**
 * \fn void BlockingDelay(uint16_t delay)
 * \brief A blocking delay.
 * \param delay time to delay in milli-seconds
 *  
 * Stalls CPU for a while, DO NOT USE. Only for debug or testing purposes.
 */
void BlockingDelay(uint16_t delay)
{
    for (; delay > 0; delay--) 
    {
        // One millisecond delay
        _delay( (unsigned long)(HW_CLOCK_FREQ/4000UL) );
    }
}

/**
 * \fn void HardwareInitialize(void)
 * \brief Handles the basic initialization of the PIC microcontroller.
 */
void HardwareInitialize(void)
{
    // Clear port latches
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    
    /** 
     * Configure I/O-ports.
     * Ports A, B and C. Set analog pins and pull-ups.
     */
    
    // Port A: LED, moisture sens drive, LCD back light, USB sense
    TRISA  = 0b11101110;
    ANSELA = 0b00001100; // AN2: battery voltage, AN3: temp sensor
    
    // Port B: aux I/O, buttons, I2C bus
    TRISB  = 0b11111111;
    ANSELB = 0b00000001; // AN12: moisture sens
    
    // Enable weak pull-ups for buttons.
    WPUB = 0b00111000; // RB5,RB4,RB3 has buttons
    INTCON2bits.nRBPU = 0; // Enable pull-ups    
   
    // Port C: RTC clock, pump PWM, relay, charge control, USB UART
    TRISC  = 0b11001001;
    ANSELC = 0b00010000; // AN16: charger voltage
    
    /**
     * Configure oscillator.
     * Tune speed and power consumption here.
     */
    OSCCON = 0b01110000;    // 16 MHz, consumes ~4.2mA in RUN
    //OSCCON = 0b01100000;    // 8 MHz, consumes ~2.8mA in RUN
    //OSCCON = 0b00110000;    // 1 MHz, consumes ~1.3mA in RUN
    OSCCON2 = 0b00000000;   // Shut down MFINTOSC, SOSC off (unless requested), main osc drive off
    OSCTUNEbits.PLLEN = 0;  // Disable 4*PLL
    OSCCONbits.IDLEN = 1;   // Go to Idle on sleep
    while (!OSCCONbits.HFIOFS); // Wait for oscillator to stabilize
    
    /** 
     * Configure UART1.
     * Connected to USB-Serial converter on RC7 and RC6.
     */
    BAUDCON1 = 0b00001000; // Not inverted, BRG16
    SPBRGH1 = (UART1_BRG16BHS >> 8) & 0xFF;
    SPBRG1  = UART1_BRG16BHS & 0xFF;
    TXSTA1 = 0b00100100;   // 8-bit tx, TX enabled, Async, High speed
    RCSTA1 = 0b10010000;   // Enable serial port, 8-bit rx
    //PIE1bits.RC1IE = 1;  // Enable receive interrupt.
    //PIE1bits.TX1IE = 1;  // Enable transmit interrupt.
    
    /**
     * Configure Timer 1.
     * Used for RTC functionality. A clock crystal is connected to the
     * secondary oscillator.
     */
    T1CON = 0b10001000; // Use secondary oscillator, no scaling, 8 bit.
    T1GCON = 0x00;      // No gate control.
    TMR1H = 0x80;       // Write MSB to cause a roll-over in 1 second (32768 Hz).
    TMR1L = 0x00;
    PIE1bits.TMR1IE = 1; // Enable TMR1 interrupt.
    T1CONbits.TMR1ON = 1; // Start timer
    
    /**
     * Configure Timer 6.
     * Used for tick-timer functionality. 
     */
    PR6 = TICK_TIMER_PERIOD - 1; // Timer period is 250
    TMR6 = 0;                    // Reset timer
    PIE5bits.TMR6IE = 1;         // Enable timer 6 interrupts.
    T6CON = 0b01110111;          // Post scale: 15, pre scale: 16, timer on
    
    /**
     * Configure MSSP2 module.
     * Configured to I2C master mode.
     */
    SSP2STAT = 0x80;
    SSP2ADD = 39; //39 = 100khz @ 16Mhz, (Period = (ADD<7:0>+1*4)/Fosc)
    SSP2CON1 = 0b00101000;
    SSP2CON2 = 0;
    SSP2CON3 = 0;
    
    /**
     * Enable interrupts.
     */
    INTCONbits.PEIE = 1; 
    INTCONbits.GIE = 1; 
    
    return;
}