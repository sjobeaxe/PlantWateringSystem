/**
 * \file RtcSwTimer.c
 * \brief A software implementation of a RTC. 
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-10-06
 *
 *  Runs of Timer 1 low-power oscillator with a 32.768kHz crystal.
 */

#include "RtcSwTimer.h"

/**
 * \def RtcTime
 * \brief Main RTC time. 
 * 
 * Is defined as external and can be read directly. Take care not to 
 * mess it up. 
 */
volatile rtcTime_st RtcTime = {0, 0, 0};
rtcAlarm_st RtcAlarms[A_MAX_ALARMS];

/**
 * \fn void RtcInit( void )
 * \brief RTC setup, initializes alarms.
 * 
 * Initialize alarms. Must be called before setting alarm times!
 */
void RtcInit( void )
{
  /** Clear alarms. */
  for ( ALARM_ID alarm = 0; alarm < A_MAX_ALARMS; alarm++ )
  {
    // All alarms deactivated at start.
    RtcAlarms[alarm].active = 0;
    
    // All seconds and minutes initialized to zero.
    RtcAlarms[alarm].time.sec = 0;
    RtcAlarms[alarm].time.min = 0;
    /* Set hours to 99, which will mean that alarm will
     * not activate unless initialized elsewhere. */
    RtcAlarms[alarm].time.hour = 99;
  }
    
  return;
}

/**
 * \fn void RtcTick( void )
 * \brief Main RTC ticking function. Advanced time and checks alarms.
 * 
 * This function should be called every one second. Can be called 
 * from interrupt.
 */
void RtcTick( void )
{
  /** Advance time */
  if ( ++RtcTime.sec > 59 )
  {
    RtcTime.sec = 0;

    if ( ++RtcTime.min > 59 )
    {
      RtcTime.min = 0;
      if ( ++RtcTime.hour > 23 )
      {
        RtcTime.hour = 0;
      }
    }
  }

  /** Check alarms */
  for ( ALARM_ID alarm = 0; alarm < A_MAX_ALARMS; alarm++ )
  {
    if ( RtcAlarms[alarm].time.hour == RtcTime.hour &&
         RtcAlarms[alarm].time.min  == RtcTime.min  &&
         RtcAlarms[alarm].time.sec  == RtcTime.sec    )
    {
      RtcAlarms[alarm].active = 1;
    }
  }
  return;
}


/**
 * \fn uint8_t RtcCheckAlarm( ALARM_ID alarm )
 * \brief Checks if a alarm is active.
 * \param alarm Alarm id.
 * \return true if alarm is active
 */
uint8_t RtcCheckAlarm( ALARM_ID alarm )
{
  return RtcAlarms[alarm].active;
}

/**
 * \fn void RtcResetAlarm( ALARM_ID alarm )
 * \brief Resets alarm to inactive state.
 * \param alarm Alarm id.
 */
void RtcResetAlarm( ALARM_ID alarm )
{
  RtcAlarms[alarm].active = 0;
  return;
}

/**
 * \fn void RtcTriggerAlarm( ALARM_ID alarm )
 * \brief Manually trigger alarm, i.e. set it to active state.
 * \param alarm Alarm id.
 */
void RtcTriggerAlarm( ALARM_ID alarm )
{
  RtcAlarms[alarm].active = 1;
  return;
}