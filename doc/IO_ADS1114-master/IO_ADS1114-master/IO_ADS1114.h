/***************************************************************************
  This is a library for the ADS1114 precision ADC

  Designed to work with all kinds of ADS1114 Breakout

  These sensors use I2C to communicate, 2 pins are required
  to interface.

  Written by Adrien Chapelet for IoThings
 ***************************************************************************/
 
#ifndef __ADS1114_H__
#define __ADS1114_H__

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS1114_ADDRESS                 (0x48)    // 1001 000 (ADDR = GND)
/*=========================================================================*/

class IO_ADS1114
{
 protected:
   // Instance-specific properties

 public:
  void begin(void);
  int16_t readADC_SingleEnded (void);

 private:
};

#endif