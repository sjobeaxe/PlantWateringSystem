/**
 * \file GlobalVariables.h
 * \brief Defines global variables.
 *
 * Originator: Anders Sjoberg
 * Creation date: 2019-57-26
 *
 * Used to share information through global variables.
 * 
 * NOTE: Try to minimize usage of global variables, it is bad programming practice.
 */

#ifndef _GLOBAL_PROJECT_VARIABLES_
#define	_GLOBAL_PROJECT_VARIABLES_

#include <stdint.h>
#include "main.h"

extern uint32_t wateringPeriod;
extern int16_t pumpOntime;
extern uint8_t pumpPWM;

extern watererStates watererState;
extern DeviceStatus_st MainStatus;

#endif	// _GLOBAL_PROJECT_VARIABLES_

