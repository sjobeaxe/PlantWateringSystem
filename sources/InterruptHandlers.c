/**
 * \file interrupt.c
 * \brief Interrupt handlers 
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-10-04
 *
 */

#include <xc.h>
#include "TickTimer.h"
#include "RtcSwTimer.h"

void interrupt InterruptRoutine(void)
{
  /** Tick timer interrupt */
  if (PIR5bits.TMR6IF)
  {
    PIR5bits.TMR6IF = 0;
    TickTimerTick();
  }
  
  /** RTC tick interrupt */
  if ( PIR1bits.TMR1IF )
  {
    PIR1bits.TMR1IF = 0;
    
    /* Highest bit must be set for to get a tick rate of 1 sec with
     * a clock crystal (32768Hz). Timer does not have a period register.
     * 
     * 8 bit writes must be enabled for this to work. Else he TMR1H
     * will never be written.
     */
    TMR1H |= 0x80; // Interrupt every second
    
    RtcTick();
  }
  
  return;
}
