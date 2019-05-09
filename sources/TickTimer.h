/**
 * \file tickTimer.h
 * \brief Software timers driven by one interrupt. 
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2015-09-07
 *
 *  Header file for Tick Timer.
 */

#ifndef	_TICK_TIMER_H_
#define _TICK_TIMER_H_

/**
 * \enum TICK_TIMER_ID Timer names.
 */
typedef enum
{
  T_PERIODIC_1S,
  T_PERIODIC_100MS,
  T_UI_TIMEOUT,
  T_LCD,
  T_BUTTON_MIDDLE,
  T_BUTTON_UP,
  T_BUTTON_UP_LP,
  T_BUTTON_DOWN,
  T_BUTTON_DOWN_LP,
  T_MAX_TIMERS  // Do not remove this
} TICK_TIMER_ID;

// Datatype to use for timer, (8 or 16)
#define TICK_TIMER_BIT_WIDTH 16

#if (TICK_TIMER_BIT_WIDTH == 8)
  #define TICK_TIMER_DT_MAX 256
  #define TICK_TIMER_DT unsigned char
#elif (TICK_TIMER_BIT_WIDTH == 16)
  #define TICK_TIMER_DT_MAX 65536
  #define TICK_TIMER_DT unsigned int
#endif

// Convert milliseconds to ticks.
// Max tick delay depends on data type and timer tick interval
// MAX_DELAY = TICK_TIMER_DT_MAX / TICK_TIMER_FOSC 
#define TICK_TIMER_PERIOD (250UL) // Adjust this for wanted tick time
#define TICK_TIMER_PS     (5UL * 16UL) // Configured in TMR6 HW init
#define TICK_TIMER_FOSC   (HW_CLOCK_FREQ / (TICK_TIMER_PERIOD * TICK_TIMER_PS * 4UL))

// Converts milliseconds to ticks. Divide by 1000 because of ms.
#define tickMs(x)         ( (x) * TICK_TIMER_FOSC / 1000UL ) 

/**
 * \def TICK_TIMER_HW_TIMER_PERIOD 
 * \brief Ticks to count down be
 * 
 * This is the value the timer should be reinitialized with every tick. 
 * Some HW timers have a period registers, those timers do not need to use this
 * macro. Otherwise this macro can be used in the interrupt routine to set the
 * timer to.
 */
#define TICK_TIMER_HW_TIMER_PERIOD (TICK_TIMER_DT_MAX - TICK_TIMER_PERIOD)

/**
 * \def TICK_TIMER_USE_MACRO_TICK
 * \brief Choose between macro or function for the main tick function.
 */
#define TICK_TIMER_USE_MACRO_TICK 0

#if TICK_TIMER_USE_MACRO_TICK
#define tickTimerTick() \
do {\
  unsigned char i; \
  for (i = 0; i < T_MAX_TIMERS; i++) \
		if (timers[i] != 0) timers[i]--; \
} while(0) \

extern volatile TICK_TIMER_DT timers[T_MAX_TIMERS];
#endif

//Initialize timers
void TickTimerInit(void);

#if !TICK_TIMER_USE_MACRO_TICK
//Count timers one down
void inline TickTimerTick(void);
#endif

//Sets value to timer
void TickTimerSet(TICK_TIMER_ID timer, TICK_TIMER_DT ticks);
//Reads values form 
TICK_TIMER_DT TickTimerGet(TICK_TIMER_ID timer);

#endif  //_TICK_TIMER_H_
