/**
 * \file LcdDriver.h
 * \brief Routines to provide basic LCD functionality.
 *
 *  Originator: Anders Sjoberg
 *  Creation date: 2017-10-07
 *
 *  Basic routines to use a LCD connected to I2C bus that use
 *  the ST7032 controller.
 * 
 *  These routines require a separate I2C driver.
 */

#ifndef LCD_I2C_H
#define	LCD_I2C_H

#include <stdint.h>

#define LCD_LINE1 0x00
#define LCD_LINE2 0x40

#define LCD_I2C_ADDRESS 0x3E

void LcdInit(void);
void LcdWrite(char *str);
void LcdGoto(uint8_t pos);

#endif	/* LCD_I2C_H */

