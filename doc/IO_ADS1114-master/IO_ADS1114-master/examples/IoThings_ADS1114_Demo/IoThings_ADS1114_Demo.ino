/***************************************************************************
  This is a library for the ADS1114 precision ADC

  Designed to work with all kinds of ADS1114 Breakout

  These sensors use I2C to communicate, 2 pins are required
  to interface.

  Written by Adrien Chapelet for IoThings
 ***************************************************************************/
 
#include <Wire.h>

#include "IO_ADS1114.h"

IO_ADS1114 ads;

void setup() 
{
  Serial.begin(115200);
  ads.begin();
}

void loop() 
{
  int16_t adc_result = ads.readADC_SingleEnded();
  Serial.println(adc_result);
  delay(100);
}