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

adcResult BatteryVoltage; 
adcResult InputVoltage;
adcResult Temperature;

void main(void)
{
  uint32_t wateringPeriod;
  int16_t pumpOntime;
  uint8_t pumpPWM;
  
  char LCDBuf[20]; // LCD buffer
  
  // UI state
  uint8_t displayMode;
  
  // Waterer state
  watererStates watererState = WAITING;
  
  // Temporary variables
  uint32_t ul;
  uint16_t ui;
  
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

  LCD_BACKLIGHT_PIN = 0;
  LED_PIN = 0;
  
  
  
  /**
   * Restore settings.
   */
  pumpPWM = intEEread(0);
  pumpOntime = ((uint16_t) intEEread(1) << 8);
  pumpOntime |= (uint16_t) intEEread(2);
  wateringPeriod = ((uint32_t) intEEread(3) << 24);
  wateringPeriod |= ((uint32_t) intEEread(4) << 16);
  wateringPeriod |= ((uint32_t) intEEread(5) << 8);
  wateringPeriod |= ((uint32_t) intEEread(6));

  
  printf("WOO!!!\n\r");
  printf("%i\n\r", TICK_TIMER_DT_MAX / TICK_TIMER_FOSC );
  printf("%i\n\r", tickMs(LONGPRESS_TIMEOUT) );
  printf("%i\n\r", T_MAX_TIMERS );
  
  
  /**
   * Start of the infinite super-loop.
   * This is the "operating system" of this project
   */
  while (1)
  {
    if (0 == TickTimerGet(T_PERIODIC_1S))
    {
      TickTimerSet(T_PERIODIC_1S, tickMs(1000));
      
      printf("%02i:%02i:%02i\n\r", RtcTime.hour, RtcTime.min, RtcTime.sec);
      
      printf("InputRaw %i\n\r", InputVoltage.raw);
      printf("InputFilt %i\n\r", InputVoltage.filtered);
      printf("InputScaled %i\n\r", InputVoltage.scaled);
      
      
      printf("BatteryRaw %i\n\r", BatteryVoltage.raw);
      printf("BatteryFilt %i\n\r", BatteryVoltage.filtered);
      printf("BatteryScaled %i\n\r", BatteryVoltage.scaled);
      
      
      printf("Temp %i\n\r", Temperature.raw);
      printf("TempFilt %i\n\r", Temperature.filtered);
      printf("TempScaled %i\n\r", Temperature.scaled);
      
      
      LED_PIN = !LED_PIN;
    }
    
    /**
     * Handle ADC tasks
     */
    
    if (0 == TickTimerGet(T_PERIODIC_100MS))
    {
      TickTimerSet(T_PERIODIC_100MS, tickMs(100));
      
      ADCTasks();
      
          
    }
    
    //  ==============================
    //  Manage UI
    //  ==============================
    if(BUTTON_MIDDLE)
    {
      if(!TickTimerGet(T_BUTTON_MIDDLE))
      {
        displayMode++;
        
        TickTimerSet(T_LCD, 0);
        TickTimerSet(T_UI_TIMEOUT, tickMs(MENU_TIMEOUT));
      }
      TickTimerSet(T_BUTTON_MIDDLE, tickMs(100));
    }

    
    if(BUTTON_UP)
    {
      if(!TickTimerGet(T_BUTTON_UP) || !TickTimerGet(T_BUTTON_UP_LP))
      {
        switch(displayMode)
        {
          case 0:
            //TickTimerSetL(T_PERIOD, 0); //Start pumping
            break;
          case 1:
            wateringPeriod += WATERING_PERIOD_INCREMENTS;
            if(wateringPeriod > WATERING_PERIOD_MAX)
            {
              wateringPeriod = WATERING_PERIOD_MAX;
            }
            break;
          case 2:
            pumpOntime += PUMP_ONTIME_INCREMETNS;
            if(pumpOntime > PUMP_ONTIME_MAX)
            {
              pumpOntime = PUMP_ONTIME_MAX;
            }
            break;
          case 3:
            pumpPWM++;
            if(pumpPWM > PUMP_PWM_MAX)
            {
              pumpPWM = PUMP_PWM_MAX;
            }
            break;
        }
        
        TickTimerSet(T_LCD, 0);
        TickTimerSet(T_UI_TIMEOUT, tickMs(MENU_TIMEOUT));
      }
      TickTimerSet(T_BUTTON_UP, tickMs(60));
      
      if(!TickTimerGet(T_BUTTON_UP_LP)) //was it a long press?
      {
        TickTimerSet(T_BUTTON_UP_LP, tickMs(50));
      }
    }
    else
    {
      TickTimerSet(T_BUTTON_UP_LP, tickMs(LONGPRESS_TIMEOUT));
    }
    
    
    if(BUTTON_DOWN)
    {
      if( !TickTimerGet(T_BUTTON_DOWN) || !TickTimerGet(T_BUTTON_DOWN_LP))
      {
        switch(displayMode)
        {
          case 0:
            //TickTimerSetL(T_WATER, 0); //Stop pumping
            break;
            
          case 1:
            wateringPeriod -= WATERING_PERIOD_INCREMENTS;
            if(wateringPeriod < WATERING_PERIOD_MIN)
            {
              wateringPeriod = WATERING_PERIOD_MIN;
            }
            //if(TickTimerGet(T_PERIOD) > wateringPeriod) //if new period is shorter reset period
            //{
            //  TickTimerSet(T_PERIOD, wateringPeriod);
            //}
            break;
            
          case 2:
            pumpOntime -= PUMP_ONTIME_INCREMETNS;
            if(pumpOntime < PUMP_ONTIME_MIN)
            {
              pumpOntime = PUMP_ONTIME_MIN;
            }
            break;
            
          case 3:
            pumpPWM--;
            if(pumpPWM < PUMP_PWM_MIN)
            {
              pumpPWM = PUMP_PWM_MIN;
            }
            break;
        }
        
        TickTimerSet(T_LCD, 0);
        TickTimerSet(T_UI_TIMEOUT, tickMs(MENU_TIMEOUT));
      }
      TickTimerSet(T_BUTTON_DOWN, tickMs(60));
      
      if(!TickTimerGet(T_BUTTON_DOWN_LP)) //was it a long press?
      {
        TickTimerSet(T_BUTTON_DOWN_LP, tickMs(50));
      }
    }
    else 
    {
      TickTimerSet(T_BUTTON_DOWN_LP, tickMs(LONGPRESS_TIMEOUT));
    }


    /**
     * UPDATE LCD
     */
    
    if(!TickTimerGet(T_LCD))
    {
      TickTimerSet(T_LCD, tickMs(300));
      
      if(!TickTimerGet(T_UI_TIMEOUT)) //return to base display if buttons not touched for a while
      {
        displayMode = 0;
      }

      switch(displayMode)
      {
        default:
          displayMode = 0;
          
        case 0:
          switch(watererState)
          {
            default:
              //ui = (bat_avg >> (BAT_N_AVG - 1));
              //sprintf(LCDBuf, "N:%2lih%02lim B%2i.%02iV", TickTimerGetL(T_PERIOD) / 3600, (TickTimerGetL(T_PERIOD) % 3600) / 60, ui / 100, ui % 100);
              sprintf(LCDBuf, "In menu 0, waiting");
              break;

            case WATERING:
              //if(water_low)
              //  sprintf(LCDBuf, "L:%2lim%02lis LoWater ", TickTimerGetL(T_WATER) / 60, tickTimerGetL(T_WATER) % 60);
              //else
              //  sprintf(LCDBuf, "L:%2lim%02lis C%i.%02iA ", TickTimerGetL(T_WATER) / 60, TickTimerGetL(T_WATER) % 60, ui / 100, ui % 100);
              sprintf(LCDBuf, "Menu 0; watering");
              break;

            case LOWBAT:
              //ui = (bat_avg >> (BAT_N_AVG - 1));
              //ui2 = (pan_avg >> (PAN_N_AVG - 1));
              //sprintf(LCDBuf, "LoB P%2i.%iVB%2i.%iV", ui2 / 100, (ui2 % 100) / 10, ui / 100, (ui % 100) / 10);
              sprintf(LCDBuf, "Menu 0; lowbat");
              break;
          }
          break;
          
        case 1:
          ul = wateringPeriod / 60;
          ui = (int) (ul / ((int) 24 * 60));
          ul %= ((int) 24 * 60);
          sprintf(LCDBuf, "Period: %id%02lih%02lim  ", ui, ul / 60, ul % 60);
          break;
          
        case 2:
          if(pumpOntime == 0)
          {
            sprintf(LCDBuf, "Pump off!           ");
          }
          else
          {
            sprintf(LCDBuf, "On time: %im%02is   ", pumpOntime / 60, pumpOntime % 60);
          }
          break;
          
        case 3:
          sprintf(LCDBuf, "Pump power: %3i%%", pumpPWM);
          break;
          
        case 4:
          /*
          ui = (pan_avg >> (PAN_N_AVG - 1));
          if(CHARGE)
            sprintf(LCDBuf, "P:%2i.%02iV Chrging  ", ui / 100, ui % 100);
          else
          {
            if(pan_avg > bat_avg)
              sprintf(LCDBuf, "P:%2i.%02iV BatFull  ", ui / 100, ui % 100);
            else
              sprintf(LCDBuf, "P:%2i.%02iV No Sun   ", ui / 100, ui % 100);
          }
          */
          sprintf(LCDBuf, "Chrg dispplay   ");
          break;
          
        case 5:
          sprintf(LCDBuf, "Settings saved! ");
          /*
          intEEwrite(pumpPWM, 0);
          intEEwrite(pumpOntime >> 8, 1);
          intEEwrite(pumpOntime, 2);
          intEEwrite((wateringPeriod >> 24) & 0xFF, 3);
          intEEwrite((wateringPeriod >> 16) & 0xFF, 4);
          intEEwrite((wateringPeriod >> 8) & 0xFF, 5);
          intEEwrite((wateringPeriod) & 0xFF, 6);
*/
          TickTimerSet(T_LCD, tickMs(1000));
          displayMode = 0;
          break;
          
      }

      //write buffer to lcd
      LcdGoto(LCD_LINE1);
      LcdWrite(LCDBuf);
      
      // Show a clock
      LcdGoto(LCD_LINE2);
      sprintf(LCDBuf, "%02i:%02i:%02i", RtcTime.hour, RtcTime.min, RtcTime.sec);
      LcdWrite(LCDBuf);
    }
    //DONE!
    
    
    //SLEEP();
    
  } // while(1))

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
    _delay((unsigned long) (HW_CLOCK_FREQ / 4000UL));
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
  TRISC  = 0b11010001;
  ANSELC = 0b00010000; // AN16: charger voltage

  /**
   * Configure oscillator.
   * Tune speed and power consumption here.
   */
  //OSCCON = 0b01110000;  // 16 MHz, consumes ~5.1mA in RUN (~3.0mA using sleep)
  OSCCON = 0b01100000;    // 8 MHz, consumes ~3.6mA in RUN (~2.2mA using sleep)
  //OSCCON = 0b01010000;  // 4 MHz, consumes ~2.7mA in RUN (~1.9mA using sleep)
  //OSCCON = 0b00110000;  // 1 MHz, consumes ~1.3mA in RUN (~1.5mA using sleep)
  OSCCON2 = 0b00000000; // Shut down MFINTOSC, SOSC off (unless requested), main osc drive off
  OSCTUNEbits.PLLEN = 0; // Disable 4*PLL
  OSCCONbits.IDLEN = 1; // Go to Idle on sleep
  while (!OSCCONbits.HFIOFS); // Wait for oscillator to stabilize

  /**
   * Configure ADC
   * Measure battery, input and other voltages.
   */
  ADCON0 = 0x00; // Channel zero, ADC off.
  ADCON1 = 0b00000000; // Reference is supply.
  ADCON2 = 0b10111001; // Right justify, ACQT: 20TAD, Fosc/8.
  ADCON0bits.ADON = 1; // Turn ADC on.
  
  /** 
   * Configure UART1.
   * Connected to USB-Serial converter on RC7 and RC6.
   */
  BAUDCON1 = 0b00001000; // Not inverted, BRG16
  SPBRGH1 = (UART1_BRG16BHS >> 8) & 0xFF;
  SPBRG1 = UART1_BRG16BHS & 0xFF;
  TXSTA1 = 0b00100100; // 8-bit tx, TX enabled, Async, High speed
  RCSTA1 = 0b10010000; // Enable serial port, 8-bit rx
  //PIE1bits.RC1IE = 1;  // Enable receive interrupt.
  //PIE1bits.TX1IE = 1;  // Enable transmit interrupt.

  /**
   * Configure Timer 1.
   * Used for RTC functionality. A clock crystal is connected to the
   * secondary oscillator.
   */
  T1CON = 0b10001000; // Use secondary oscillator, no scaling, 8 bit.
  T1GCON = 0x00; // No gate control.
  TMR1H = 0x80; // Write MSB to cause a roll-over in 1 second (32768 Hz).
  TMR1L = 0x00;
  PIE1bits.TMR1IE = 1; // Enable TMR1 interrupt.
  T1CONbits.TMR1ON = 1; // Start timer

  /**
   * Configure Timer 6.
   * Used for tick-timer functionality. 
   */
  PR6 = TICK_TIMER_PERIOD - 1; // Timer period is 250
  TMR6 = 0; // Reset timer
  PIE5bits.TMR6IE = 1; // Enable timer 6 interrupts.
  T6CON = 0b00100111; // Post scale: 5, pre scale: 16, timer on

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




/**
 * Blocking read of ADC.
 */
uint16_t readAnalog(uint8_t channel)
{
   ADCON0bits.CHS = channel;
 
  BlockingDelay(1);
  ADCON0bits.GO = 1; //start convert
  while(ADCON0bits.GO); //wait for completion

  return (uint16_t) ADRESH << 8 | ADRESL;
}


void setPWM(uint16_t pwm)
{
  uint8_t temp = ((uint8_t ) pwm << 4) & 0x30; //extract LSB
  CCPR1L = (pwm >> 2);
  CCP1CON = temp | (CCP1CON & ~0x30);
  return;
}

// Read from inernal EEPROM
uint8_t intEEread(uint8_t adr)
{
  EEADR = adr;
  EECON1 = 0b00000000; //CFGS=EEPGD=0
  EECON1bits.RD = 1; //read
  return EEDATA;
}

// Write into inernal EEPROM

void intEEwrite(uint8_t data, uint8_t adr)
{
  EEADR = adr;
  EEDATA = data;
  EECON1 = 0b00000000; //CFGS=EEPGD=0
  EECON1bits.WREN = 1;
  INTCONbits.GIE = 0;
  EECON2 = 0x55;
  EECON2 = 0xAA;
  EECON1bits.WR = 1; //start write
  while(EECON1bits.WR); //wait for completion
  EECON1bits.WREN = 0;
  INTCONbits.GIE = 1;
  return;
}

void ADCTasks(void)
{
  BatteryVoltage.raw = readAnalog(ADC_CHANNEL_BATTERY);
  BatteryVoltage.filtered += BatteryVoltage.raw - (BatteryVoltage.filtered >> 3);
  BatteryVoltage.scaled = (((uint32_t)BatteryVoltage.filtered) << 5) / 79; // Scale by 2^5 /77 = 0.416
  
  InputVoltage.raw = readAnalog(ADC_CHANNEL_INPUT);
  InputVoltage.filtered += InputVoltage.raw - (InputVoltage.filtered >> 3);
  InputVoltage.scaled = (((uint32_t)InputVoltage.filtered) << 5) / 77; // Scale by 2^5 /77 = 0.416
  
  Temperature.raw = readAnalog(ADC_CHANNEL_TEMP);
  Temperature.filtered += Temperature.raw - (Temperature.filtered >> 3);
  Temperature.scaled = (((Temperature.filtered - 655)) << 4) / 51; // Offset by 0.4V then scale by 0.313
}