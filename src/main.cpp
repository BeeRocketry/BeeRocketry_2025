#include <Arduino.h>
#include <HardwareSerial.h>

#include "bmp388Class.h"

#define SDA_Pin PB7
#define SCL_Pin PB8

HardwareSerial usbPort(PA10, PA9);

I2Class* i2c_;
bmp388* bmp;

void setup() {
  usbPort.begin(115200);

  i2c_ = new I2Class(SDA_Pin, SCL_Pin);

  bmp = new bmp388(i2c_);
}

void loop() {
  BMP_SensorData data = bmp->BMPGetData();

  usbPort.print("Sicaklik : "); usbPort.println(data.temperature);
  usbPort.print("Basinc : "); usbPort.println(data.pressure);
  usbPort.print("Yukseklik : "); usbPort.println(data.altitude);

  delay(1000);
}