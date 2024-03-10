/***************************************************************************
  This is a library for the ADS1114 precision ADC

  Designed to work with all kinds of ADS1114 Breakout

  These sensors use I2C to communicate, 2 pins are required
  to interface.

  Written by Adrien Chapelet for IoThings
 ***************************************************************************/
 
#include <Arduino.h>
#include <Wire.h>
#include "IO_ADS1114.h"


void IO_ADS1114::begin (void) {
  Wire.begin();
  Wire.beginTransmission(ADS1114_ADDRESS);
  Wire.write(0x01);  // Select configuration register
  Wire.write(0x82);  // AINP = AIN0 and AINN = AIN1, +/- 2.048V  84=default
  Wire.write(0x83);  // Continuous conversion mode, 128 SPS
  Wire.endTransmission();
  delay(300);
}

int16_t IO_ADS1114::readADC_SingleEnded (void) {
  unsigned int data[2];
  Wire.beginTransmission(ADS1114_ADDRESS);
  Wire.write(0x00);  // Select data register
  Wire.endTransmission();

  Wire.requestFrom(ADS1114_ADDRESS, 2);  // Request 2 bytes of data

  if(Wire.available() == 2)
  {
     data[0] = Wire.read();  // Read 2 bytes of data
     data[1] = Wire.read();  // raw_adc msb, raw_adc lsb
  }

  int16_t raw_adc = (data[0] * 256.0) + data[1];  // Convert the data

  return raw_adc;
}
