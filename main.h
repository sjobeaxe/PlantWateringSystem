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
 * \def FW_VERSION
 * \brief Firmware version, using only one number
 */
#define FW_VERSION 1


void HardwareInitialize(void);


void BlockingDelay(uint16_t delay);

#endif	/* MAIN_H */
