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


void putch(char data);
void BlockingDelay(uint16_t delay);
void HardwareInitialize(void);

#endif	/* MAIN_H */
