#include <Arduino.h>

#include "LPS22HH.h"

#define SDA_Pin PB7
#define SCL_Pin PB8

I2Class* i2c_;
LPS22HH* lps;

void setup() {
  Serial.begin(115200);
  Serial.println("Seri Port Baslatildi...");

  i2c_ = new I2Class(SDA_Pin, SCL_Pin);

  lps = new LPS22HH(i2c_);

  Serial.println("LPS Baslatildi...");

  lps->setCTRL_REG1(LPS_OUTPUTDATARATE10HZ, LPS_EN_LPFP::LPS_LPFPENABLE, LPS_BDU_CONT);

  delay(500);

  Serial.println("LPS Ayarlandi...");
}

void loop() {
  float temp = 0, pres = 0, alt = 0;

  temp = lps->getTemperature();
  pres = lps->getPressure();
  alt = lps->getAltitude(pres);

  Serial.print("Sicaklik : "); Serial.println(temp);
  Serial.print("Basinc : "); Serial.println(pres);
  Serial.print("Yukseklik : "); Serial.println(alt);

  Serial.println();

  delay(1000);
}