/**
 * \file LocalUserInterface.c
 * \brief Code for the local user interface (LCD & buttons).
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2019-05-26
 *
 *  Implements the LCD meu.
 * 
 */

#include "LocalUserInterface.h"
#include "DebounceBit.h"
#include "HardwareDefinitions.h"
#include "TickTimer.h"
#include "LcdDriver.h"
#include "GlobalVariables.h"
#include "RtcSwTimer.h"
#include <stdio.h>

// Keeps state of what display to show.
uint8_t LCDDisplayState;

// LCD text buffers
static uint8_t DispLine1[24];
static uint8_t DispLine2[24];

// Variables here for now, needs access from multiple places.
uint32_t wateringPeriod = 6*60*60;
int16_t pumpOntime = 5*60;
uint8_t pumpPWM = 80;

// REMOVE THIS LINE
extern adcResult Temperature;


void LocalUserInterfaceTask(void)
{
  static Counter_t upButtonTimer = 0;
  static Counter_t downButtonTimer = 0;
  static Counter_t middleButtonTimer = 0;
  static Counter_t upButtonLongPressTimer = 0;
  static Counter_t downButtonLongPressTimer = 0;
  
  // Count down all timers every call
  counterCheck(&upButtonTimer);
  counterCheck(&downButtonTimer);
  counterCheck(&middleButtonTimer);
  counterCheck(&upButtonLongPressTimer);
  counterCheck(&downButtonLongPressTimer);
  
  /**
   * Middle button press.
   */
  if(BUTTON_MIDDLE)
  {
    LCD_BACKLIGHT_PIN = 1;
    if( 0 == middleButtonTimer)
    {
      LCDDisplayState++;

      TickTimerSet(T_LCD, 0);
      TickTimerSet(T_UI_TIMEOUT, tickMs(UI_MENU_TIMEOUT));
    }
    middleButtonTimer = UI_BUTTON_MIN_RELEASE_TIME;
  }

  /**
   * Up button press.
   */
  if(BUTTON_UP)
  {
    if( (0 == upButtonTimer) || (0 == upButtonLongPressTimer) )
    {
      switch(LCDDisplayState)
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
      TickTimerSet(T_UI_TIMEOUT, tickMs(UI_MENU_TIMEOUT));
    }
    upButtonTimer = UI_BUTTON_MIN_RELEASE_TIME;

    if( 0 == upButtonLongPressTimer ) //was it a long press?
    {
      upButtonLongPressTimer = UI_BUTTON_LONG_PRESS_REPEAT_RATE;
    }
  }
  else
  {
    upButtonLongPressTimer = UI_BUTTON_LONG_PRESS_TIME;
  }

  /**
   * Down button press.
   */
  if(BUTTON_DOWN)
  {
    if( (0 == downButtonTimer) || (0 == downButtonLongPressTimer) )
    {
      switch(LCDDisplayState)
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
      TickTimerSet(T_UI_TIMEOUT, tickMs(UI_MENU_TIMEOUT));
    }
    downButtonTimer = UI_BUTTON_MIN_RELEASE_TIME;

    if( 0 == downButtonLongPressTimer ) //was it a long press?
    {
      downButtonLongPressTimer = UI_BUTTON_LONG_PRESS_REPEAT_RATE;
    }
  }
  else 
  {
    downButtonLongPressTimer = UI_BUTTON_LONG_PRESS_TIME;
  }
  
  return;
}

/**
 * LCD task.
 */
void LCDUpdateTask(void)
{
  uint32_t ul; // Temp variable 
  uint16_t ui; // Temp variable
  
  if(!TickTimerGet(T_UI_TIMEOUT)) //return to base display if buttons not touched for a while
  {
    LCDDisplayState = 0;
    LCD_BACKLIGHT_PIN = 0;
  }

  switch(LCDDisplayState)
  {
    default:
      LCDDisplayState = 0;

    case 0:
      switch(watererState)
      {
        default:
          //ui = (bat_avg >> (BAT_N_AVG - 1));
          //sprintf(LCDBuf, "N:%2lih%02lim B%2i.%02iV", TickTimerGetL(T_PERIOD) / 3600, (TickTimerGetL(T_PERIOD) % 3600) / 60, ui / 100, ui % 100);
          sprintf(DispLine1, "Base display        ");
          break;

        case WATERING:
          //if(water_low)
          //  sprintf(LCDBuf, "L:%2lim%02lis LoWater ", TickTimerGetL(T_WATER) / 60, tickTimerGetL(T_WATER) % 60);
          //else
          //  sprintf(LCDBuf, "L:%2lim%02lis C%i.%02iA ", TickTimerGetL(T_WATER) / 60, TickTimerGetL(T_WATER) % 60, ui / 100, ui % 100);
          sprintf(DispLine1, "Menu 0; watering");
          break;

        case LOWBAT:
          //ui = (bat_avg >> (BAT_N_AVG - 1));
          //ui2 = (pan_avg >> (PAN_N_AVG - 1));
          //sprintf(LCDBuf, "LoB P%2i.%iVB%2i.%iV", ui2 / 100, (ui2 % 100) / 10, ui / 100, (ui % 100) / 10);
          sprintf(DispLine1, "Menu 0; lowbat");
          break;
      }
      break;

    case 1:
      ul = wateringPeriod / 60;
      ui = (int) (ul / ((int) 24 * 60));
      ul %= ((int) 24 * 60);
      sprintf(DispLine1, "Period: %id%02lih%02lim  ", ui, ul / 60, ul % 60);
      break;

    case 2:
      if(pumpOntime == 0)
      {
        sprintf(DispLine1, "Pump off!           ");
      }
      else
      {
        sprintf(DispLine1, "On time: %im%02is   ", pumpOntime / 60, pumpOntime % 60);
      }
      break;

    case 3:
      sprintf(DispLine1, "Pump power: %3i%% ", pumpPWM);
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
      sprintf(DispLine1, "Charge display    ");
      break;

    case 5:
      sprintf(DispLine1, "Settings saved!  ");
      
      intEEwrite(pumpPWM, 0);
      intEEwrite(pumpOntime >> 8, 1);
      intEEwrite(pumpOntime, 2);
      intEEwrite((wateringPeriod >> 24) & 0xFF, 3);
      intEEwrite((wateringPeriod >> 16) & 0xFF, 4);
      intEEwrite((wateringPeriod >> 8) & 0xFF, 5);
      intEEwrite((wateringPeriod) & 0xFF, 6);

      TickTimerSet(T_LCD, tickMs(1000));
      LCDDisplayState = 0;
      break;

  }

  // Write buffers to LCD
  LcdGoto(LCD_LINE1);
  LcdWrite(DispLine1);

  // Show a clock
  LcdGoto(LCD_LINE2);
  sprintf(DispLine2, "%02i:%02i:%02i  %i.%iC  ", RtcTime.hour, RtcTime.min, RtcTime.sec, Temperature.scaled/10, Temperature.scaled%10);
  LcdWrite(DispLine2);
  return;
}