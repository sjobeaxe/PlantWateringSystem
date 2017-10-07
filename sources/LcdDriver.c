/**
 * \file LcdDriver.c
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

#include "I2CDriver.h"
#include "LcdDriver.h"

/**
 * \fn void LcdInit(void)
 * \brief Initializes and configures the LCD.
 */
void LcdInit(void)
{
  I2CStart();
  I2CWrite(LCD_I2C_ADDRESS << 1); // Shift by one since LSB is R/W bit in I2C.
  
  /** 
   * This routine is almost straight from the datasheet example.
   */
  I2CWrite(0x00); // Control byte, RS bit clear
  /** Set instruction table bit = 1. */
  I2CWrite(0x39); // Function set                    0011 1001
  I2CWrite(0x14); // Internal OSC                    0001 0100
  I2CWrite(0x74); // Contrast set                    0111 0100

  /** Voltage booster off (bit3). */
  I2CWrite(0x50); // Power/ICON control/contrast set 0101 0100
  I2CWrite(0x6F); // Follower control                0110 1111
  I2CWrite(0x0C); // Display ON/OFF                  0000 1100
  I2CWrite(0x01); // Clear display                   0000 0001
  I2CStop();
  return;
}

/**
 * \fn void LcdWrite(const char *str)
 * \brief Prints string on the current location on the LCD.
 * \param str Pointer to null-terminated string to be printed.
 */
void LcdWrite(const char *str)
{
  I2CStart();
  I2CWrite(LCD_I2C_ADDRESS << 1);
  I2CWrite(0x40); // Control byte, RS bit set

  while (*str != '\0')
  {
    I2CWrite(*str++);
  }

  I2CStop();
  return;
}

/**
 * \fn void LcdGoto(uint8_t pos)
 * \brief Moves cursor to a specific location.
 * \param pos Position to move cursor to. 
 */
void LcdGoto(uint8_t pos)
{
  I2CStart();
  I2CWrite(LCD_I2C_ADDRESS << 1);
  I2CWrite(0x00); // Control byte, RS bit clear
  I2CWrite(pos |= 0x80); //Set MSB to evoke "DDRAM address counter"
  I2CStop();
  return;
}
