/**
 * \file interrupt.c
 * \brief Interrupt handlers 
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-10-04
 *
 */

#include <xc.h>
#include "tickTimer.h"

void interrupt interruptRoutine(void)
{
  /** Tick timer interrupt */
  if (PIR5bits.TMR6IF)
  {
    PIR5bits.TMR6IF = 0;
    tickTimerTick();
  }
  
  return;
}
