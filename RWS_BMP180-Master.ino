#include "RWS_BMP180.h"

//******************************************************************************  
//******************************************************************************  
// Based on Adafruit example for the BMP085  
//******************************************************************************  
//******************************************************************************  
// Connect VCC of the BMP180 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

RWS_BMP180 bmp;

void setup() {
  Serial.begin(115200);
  if (!bmp.begin(BMP180_ULTRAHIGHRES)) Serial.println("Could not find a valid BMP180 sensor, check wiring!");
}
  
void loop() {
  bmp.heartBeat();    // call the heartbeat function now and then to keep the pressure and temperature up to date
  float T = bmp.readTemperature(); // adjustments to match known conditions
  float P = bmp.readPressure();
  float slP = bmp.readSealevelPressure(100);
  float alt = bmp.readAltitude(slP);
  Serial.print(millis()); Serial.print(" ms, T = ");
  Serial.print(T); Serial.print(" C, P = ");
  Serial.print(P); Serial.print(" Pa, Sea Level P = ");
  Serial.print(slP); Serial.print(" Pa, Altitude = ");
  Serial.print(alt); Serial.print(" m");
  Serial.println();
  bmp.heartBeat(); 
  delay(500);
}
