/**
 * \file I2CDriver.c
 * \brief A driver to to use the MSSP peripheral in I2C master mode.
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-10-07
 *
 *  Simple blocking routines for I2C master.
 * 
 *  Using MSSP2.
 */

#include <xc.h>
#include "I2CDriver.h"

/**
 * \fn void I2CWait(void)
 * \brief Waits for a started operation on the I2C to finish.
 * \param alarm Alarm id.
 */
void I2CWait(void)
{
    while ( ( SSP2CON2 & 0x1F ) || ( SSP2STAT & 0x04 ) );
}

/**
 * \fn void I2CStart(void)
 * \brief Send start condition on the I2C bus.
 */
void I2CStart(void)
{
    I2CWait();
    SSP2CON2bits.SEN = 1;
}

/**
 * \fn void I2CRestart(void)
 * \brief Send repeated start condition on the I2C bus.
 */
void I2CRestart(void)
{
    I2CWait();
    SSP2CON2bits.RSEN = 1;
}

/**
 * \fn void I2CStop(void)
 * \brief Stops communication and sends stop condition on the I2C bus.
 */
void I2CStop(void)
{
    I2CWait();
    SSP2CON2bits.PEN = 1;
    return;
}

/**
 * \fn I2CWrite(uint8_t data)
 * \brief Send one byte of data on the bus.
 * \param data The byte to send.
 */
void I2CWrite(uint8_t data)
{
    I2CWait();
    SSP2BUF = data;
    return;
}