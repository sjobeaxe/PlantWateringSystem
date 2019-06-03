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

typedef struct Status_t
{
  uint8_t Charging      : 1;
  uint8_t LowBat        : 1;
  uint8_t PumpOn        : 1;
  uint8_t RelayActive   : 1;
  
  uint8_t LowWater      : 1;
  uint8_t unused0       : 1;
  uint8_t unused1       : 1;
  uint8_t unused2       : 1;
}DeviceStatus_st;

typedef struct
{
    uint16_t raw;
    uint16_t filtered;
    uint16_t scaled;
} adcResult;

void putch(char data);
void BlockingDelay(uint16_t delay);
void HardwareInitialize(void);

uint8_t intEEread(uint8_t adr);
void intEEwrite(uint8_t data, uint8_t adr);
void setPWM(uint16_t pwm);
uint16_t readAnalog(uint8_t channel);
void ADCTasks(void);
void ChargerTask(void);

#endif	/* MAIN_H */
