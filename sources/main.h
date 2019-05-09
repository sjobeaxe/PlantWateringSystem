/**
 * \file main.h
 * \brief Main header 
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-07-05
 *
 *  Plant Watering System main header.
 */

#ifndef MAIN_H
#define	MAIN_H
 
#include <xc.h>
#include <stdint.h>
#include <stddef.h>

/**
 * Loosely adhering to semantic versioning http://semver.org/.
 */

/**
 * \def FW_VERSION_MAJOR
 * \brief Major firmware version.
 * 
 * Changes here reflect major changes and does not guarantee backwards 
 * compatibility
 */
#define FW_VERSION_MAJOR 0

/**
 * \def FW_VERSION_MINOR
 * \brief Minor firmware version.
 * 
 * Newer versions bring bug fixes and new features but are backwards compatible.
 */
#define FW_VERSION_MINOR 1

/**
 * \def FW_VERSION_PATCH
 * \brief Patch firmware version.
 * 
 * Backwards compatible bug fixes.
 */
#define FW_VERSION_PATCH 0

typedef enum
{
    WAITING,
    WATERING,
    LOWBAT
} watererStates;

typedef struct
{
    uint16_t raw;
    uint16_t filtered;
    uint16_t scaled;
} adcResult;


// UI constants
#define LONGPRESS_TIMEOUT 1500  //in ms
#define MENU_TIMEOUT 45000  //in ms

#define WATERING_PERIOD_MAX ((uint32_t)4*24*60*60)
#define WATERING_PERIOD_MIN ((uint32_t)15*60)
#define WATERING_PERIOD_INCREMENTS (uint16_t)15*60

#define PUMP_ONTIME_MAX ((int16_t)60*60)
#define PUMP_ONTIME_MIN ((int16_t)0)
#define PUMP_ONTIME_INCREMETNS 15

#define PUMP_PWM_MAX 100
#define PUMP_PWM_MIN 15

void putch(char data);
void BlockingDelay(uint16_t delay);
void HardwareInitialize(void);

uint8_t intEEread(uint8_t adr);
void intEEwrite(uint8_t data, uint8_t adr);
void setPWM(uint16_t pwm);
uint16_t readAnalog(uint8_t channel);
void ADCTasks(void);

#endif	/* MAIN_H */
