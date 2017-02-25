/*************************************************** 
 *  Modified by RWS 2016-11-14 to reduce complexity and put pressures in Pa
 *  then to avoid using delay while waiting on pressure data
 *  See the .h file for more details.
  
 Originally:  This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp sensor

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

#include "RWS_BMP180.h"

RWS_BMP180::RWS_BMP180() {      // initializer for the class
}

bool RWS_BMP180::begin(uint8_t mode) {
  if (mode > BMP180_ULTRAHIGHRES) 
    mode = BMP180_ULTRAHIGHRES;
  oversampling = mode;
  Wire.begin();
  if (read8(0xD0) != 0x55) return false;  // read Chip-ID for validation check
  /* read calibration data from register locations given in datasheet*/
  ac1 = read16(0xAA);
  ac2 = read16(0xAC);
  ac3 = read16(0xAE);
  ac4 = read16(0xB0);
  ac5 = read16(0xB2);
  ac6 = read16(0xB4);
  b1  = read16(0xB6);
  b2  = read16(0xB8);
  mb  = read16(0xBA);
  mc  = read16(0xBC);
  md  = read16(0xBE);
  readTemperature();
  readPressure();     // give it time to make at least one initialization reading
  delay(50);
  heartBeat();
  return true;
}

void RWS_BMP180::heartBeat(void) {                 // Call when convenient to update T,P values
  static byte pending = 0;
  if(pending == 0){                                // if nothing else going on
    if(micros()-lastT > 50000 || lastT == 0){      // update T every 20th of a second
      write8(BMP180_CONTROL, BMP180_READTEMPCMD);
      pending = 1;                  // waiting for temperature when pending = 1
      lastT = micros();
      }
    else{                           // otherwise start a pressure measurement
      write8(BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));
      pending = 2;                  // waiting for a pressure measurement when pending =2 
      lastP = micros();
    }
  } else if(pending == 1){          // read the temperature if it has been long enough
    if((micros()-lastT)/1000 >= 5){ 
      rawT = read16(BMP180_TEMPDATA);
      pending = 0;                  // back to doing nothing
    }
  } else if(pending == 2){          // read the pressure if it has been long enough
    if((micros()-lastP)/1000 >= 2 + 3 * (1 << oversampling)){    // 5,8,14,26 ms for 1,2,4,8 samples
      rawP = read16(BMP180_PRESSUREDATA);
      rawP <<= 8;
      rawP |= read8(BMP180_PRESSUREDATA+2);
      rawP >>= (8 - oversampling);
      pending = 0;                  // back to doing nothing
    }
  }
}

int32_t RWS_BMP180::readPressure(void) { // Pressure in Pa
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;
  heartBeat();
  while(micros()-lastT > 50000 || lastT == 0) heartBeat(); // Make sure data is recent
  while(micros()-lastP > 50000 || lastP == 0) heartBeat();
  UT = rawT;
  UP = rawP;

  B5 = computeB5(UT);

  // do pressure calcs according to datasheet to extract pressure in Pa
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;
  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * ((uint32_t) 50000UL >> oversampling );
  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + (int32_t)3791)>>4);
  return p;
}

float RWS_BMP180::readTemperature(void) {
  int32_t UT, B5, IT;     // following ds convention
  float temp;
  heartBeat();
  while(micros()-lastT > 50000) heartBeat();
  UT = rawT;
  B5 = computeB5(UT);
  IT = (B5+8) >> 4;
  temp = (float) IT / 10;       // convert from 10ths of a degree 
  return temp;
}

int32_t RWS_BMP180::readSealevelPressure(float altitude_meters) {
  // sea level pressure, e.g. standard barometric pressure based on local altitude
  // formula from datasheet
  float pressure = readPressure();
  return (int32_t)(pressure / pow(1.0-altitude_meters/44330, 5.255));
}

float RWS_BMP180::readAltitude(float sealevelPressure) {
  // altitude in metres based on sea level pressure 
  float altitude;
  float pressure = readPressure();
  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));
  return altitude;
}

int32_t RWS_BMP180::computeB5(int32_t UT) {    
  // a function of the raw temperature defined in the datasheet
  int32_t X1 = ((UT - (int32_t)ac6) * ((int32_t)ac5)) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

/*****Functions to read and write to the I2C port*****************************/

uint8_t RWS_BMP180::read8(uint8_t a) {
  uint8_t ret;
  Wire.beginTransmission(BMP180_I2CADDR); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission
  Wire.beginTransmission(BMP180_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP180_I2CADDR, 1);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  Wire.endTransmission(); // end transmission
  return ret;
}

uint16_t RWS_BMP180::read16(uint8_t a) {
  uint16_t ret;
  Wire.beginTransmission(BMP180_I2CADDR); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission
  Wire.beginTransmission(BMP180_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP180_I2CADDR, 2);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret <<= 8;
  ret |= Wire.read(); // receive DATA
  Wire.endTransmission(); // end transmission
  return ret;
}

void RWS_BMP180::write8(uint8_t a, uint8_t d) {
  Wire.beginTransmission(BMP180_I2CADDR); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.write(d);  // write data
  Wire.endTransmission(); // end transmission
}

