/**
 * \file tickTimer.c
 * \brief Software timers driven by one interrupt. 
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2015-09-07
 *
 *  Source file for Tick Timer.
 */

#include "tickTimer.h"

/**
 * \def volatile TICK_TIMER_DT timers[T_MAX_TIMERS];
 * \brief The timer structures.
 */
static volatile TICK_TIMER_DT timers[T_MAX_TIMERS];

/**
 * \fn void tickTimerInit(void)
 * \brief Resets all timers to zero.
 */
void TickTimerInit(void)
{
  unsigned char i;
  for (i = 0; i < T_MAX_TIMERS; i++)
  {
    timers[i] = 0;
  }
  return;
}

/**
 * \fn void inline tickTimerTick(void)
 * \brief Ticks all timer one count down.
 */
#if !TICK_TIMER_USE_MACRO_TICK

void inline TickTimerTick(void)
{
  unsigned char i;
  for (i = 0; i < T_MAX_TIMERS; i++)
  {
    if (timers[i] != 0) timers[i]--;
  }
  return;
}
#endif

/**
 * \fn void tickTimerSet(TICK_TIMER_ID timer, TICK_TIMER_DT ticks)
 * \brief Sets a timer to tick down from a value.
 * \param timer Timer id.
 * \param Ticks to count down from.
 */
void TickTimerSet(TICK_TIMER_ID timer, TICK_TIMER_DT ticks)
{
  timers[timer] = ticks;
  return;
}

/**
 * \fn void TICK_TIMER_DT tickTimerGet(TICK_TIMER_ID timer)
 * \brief Reads current value in timer.
 * \param timer Timer id to get.
 * \return Ticks left to zero.
 */
TICK_TIMER_DT TickTimerGet(TICK_TIMER_ID timer)
{
  return timers[timer];
}