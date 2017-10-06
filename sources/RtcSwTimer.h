/**
 * \file RtcSwTimer.h
 * \brief A software implementation of a RTC. 
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-10-06
 *
 *  Runs of Timer 1 low-power oscillator with a 32.768kHz crystal.
 */

#ifndef RTC_SW_TIMER1_H
#define	RTC_SW_TIMER1_H

#include <stdint.h>

/**
 * \enum ALARM_ID Alarm names for easy access.
 */
typedef enum
{
  A_TEST_ALARM = 0,
  A_MAX_ALARMS  // Do not remove this
} ALARM_ID;


typedef struct RTCTime_t
{
  int8_t sec;
  int8_t min;
  int8_t hour;
} rtcTime_st;


typedef struct RTCAlarm_t
{
  rtcTime_st time;
  
  // Flag to indicate if alarm is triggered. Set in interrupts.
  volatile uint8_t active;
} rtcAlarm_st;

// Allow others to access these
extern volatile rtcTime_st RtcTime;
//extern rtcAlarm_st RtcAlarms[A_MAX_ALARMS];

// Initialize RTC and alarms
void RtcInit( void );

// Function called from interrupt
void RtcTick( void );

// Checks if alarm is active
uint8_t RtcCheckAlarm( ALARM_ID alarm );

// Reset alarm
void RtcResetAlarm( ALARM_ID alarm );

// Manually trigger alarm
void RtcTriggerAlarm( ALARM_ID alarm );

#endif	/* RTC_SW_TIMER1_H */

