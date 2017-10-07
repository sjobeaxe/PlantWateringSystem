/**
 * \file I2CDriver.h
 * \brief A driver to to use the MSSP peripheral in I2C master mode.
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-10-07
 *
 *  Simple blocking routines for I2C master.
 */

#ifndef I2C_H
#define	I2C_H

#include <stdint.h>

void I2CWait(void);
void I2CStart(void);
void I2CRestart(void);
void I2CStop(void);
void I2CWrite(uint8_t data);

#endif	/* I2C_H */

