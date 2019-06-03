/**
 * \file LocalUserInterface.h
 * \brief Code for the local user interface (LCD & buttons).
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2019-05-26
 *
 *  Implements the LCD meu.
 * 
 */

#ifndef _LOCAL_USER_INTERFACE_H_
#define	_LOCAL_USER_INTERFACE_H_


// UI constants
#define UI_MENU_TIMEOUT 15000  //in ms
//#define UI_MENU_TIMEOUT 45000  //in ms

// Timer values, multiples of UI button call interval, which is 10ms.
#define UI_BUTTON_MIN_RELEASE_TIME 7
#define UI_BUTTON_LONG_PRESS_TIME 150
#define UI_BUTTON_LONG_PRESS_REPEAT_RATE 4

// Setting maximum and minimum
#define WATERING_PERIOD_MAX ((uint32_t)4*24*60*60)
#define WATERING_PERIOD_MIN ((uint32_t)15*60)
#define WATERING_PERIOD_INCREMENTS (uint16_t)15*60

#define PUMP_ONTIME_MAX ((int16_t)60*60)
#define PUMP_ONTIME_MIN ((int16_t)0)
#define PUMP_ONTIME_INCREMETNS 15

#define PUMP_PWM_MAX 100
#define PUMP_PWM_MIN 15


// Should be called with a 10ms period.
void LocalUserInterfaceTask(void);

void LCDUpdateTask(void);

#endif	/* _LOCAL_USER_INTERFACE_H_ */

