/**********************************************************
 *  Filename: debounceBit.h
 **********************************************************
 *
 *  Functions to debounced boolean variable.
 *
 *  Originator: Anders Sjoberg
 *  Date: 2015-11-23
 *
 *  Description: A basic misc tool.
 *
 **********************************************************/

#include "DebounceBit.h"

// Debounces bit and returns current state.
uint8_t deBoBit(DebouncedBit_st *dbb, uint8_t state)
{
  uint8_t limit;
  
  if (dbb->state == state)
  {
    // States are the same. Clear counter.
    dbb->counter = 0;
  }
  else
  {
    // States differ, increment counter.
    dbb->counter++;

    if (dbb->state)
    {
      // Current state is active, use deactivation counter.
      limit = dbb->deActTimes;
    }
    else
    {
      // Current state is de-active, use activation counter.
      limit = dbb->actTimes;
    }
    
    if (dbb->counter >= limit)
    {
      // Counter is bigger than limit, change state.
      dbb->state = state;
    }
  }
  return dbb->state;
}

// Debounces bit with hysteresis and returns current state.
uint8_t deBoBitHyst(DebouncedBit_st *dbb, uint8_t set, uint8_t clear)
{
  if ( set )
  {
    deBoBit(dbb, 1);
  }
  else if ( clear )
  {
    deBoBit(dbb, 0);
  }
  else
  {
    // Clear counter if we are in between set/clear thresholds.
    dbb->counter = 0;
  }
  
  return dbb->state;
}

// Decrements counter and returns 1 if zero reached
uint8_t counterCheck(Counter_t *counter)
{
  if ( *counter == 0 )
  {
    return 1;
  }
  (*counter)--;
  
  return 0;
}
