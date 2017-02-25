#ifndef _RWS_BMP180_h
#define _RWS_BMP180_h
/* V2 has the library code stripped down to bare essentials and may not be compatible with some old code
 * V3 has a new RWS_BMP085 class that doesn't wait for completion, adds the heartbeat() function 
 * V4 changed all the names to BMP180, created separate library
 */
/*************************************************** 
 *  Modified by RWS 2016-11-14 to reduce complexity and put pressures in Pa
 *  then to avoid using delay while waiting on pressure data
 *  
  This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Arduino.h"
#include "Wire.h"

#define BMP180_I2CADDR 0x77

#define BMP180_ULTRALOWPOWER 0            // Pressure sampling modes - 1 sample
#define BMP180_STANDARD      1            // average over 2 samples internally to sensor
#define BMP180_HIGHRES       2            // average over 4 samples
#define BMP180_ULTRAHIGHRES  3            // average over 8 samples is 2^OS
#define BMP180_CONTROL           0xF4 
#define BMP180_TEMPDATA          0xF6
#define BMP180_PRESSUREDATA      0xF6
#define BMP180_READTEMPCMD       0x2E
#define BMP180_READPRESSURECMD   0x34


class RWS_BMP180 {   // define a class for C++ so you can declare it in your program
 public:
  RWS_BMP180();
  bool begin(uint8_t mode = BMP180_ULTRAHIGHRES);  // by default go highres
  float readTemperature(void);
  int32_t readPressure(void);
  int32_t readSealevelPressure(float altitude_meters = 0);
  float readAltitude(float sealevelPressure = 101325); // std atmosphere
  void heartBeat(void);  // call now and then to make sure P,T up to date faster than 5 Hz
  
 private:
  int32_t computeB5(int32_t UT);
  uint8_t read8(uint8_t addr);
  uint16_t read16(uint8_t addr);
  void write8(uint8_t addr, uint8_t data);

  uint8_t oversampling;

  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;

  uint32_t lastT = 0, lastP = 0;  // last readings of raw quantities 
  uint16_t rawT = 0;
  uint32_t rawP = 0;    // last raw values read
};

#endif // _RWS_BMP180_h
