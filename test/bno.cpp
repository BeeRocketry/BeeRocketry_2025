#include <Arduino.h>
#include <HardwareSerial.h>

#include "bno055.h"

#define SDA_Pin PB7
#define SCL_Pin PB8

//HardwareSerial usbPort(PA10, PA9);

I2Class* i2c_;
Bno055* bno;

void setup() {
  usbPort.begin(115200);

  i2c_ = new I2Class(SDA_Pin, SCL_Pin);

  bno = new Bno055(i2c_);

  bno->BNOInit();
}

void loop() {
  BNO_DOF3_Float acc, gyro, mag;

  acc = bno->getAccData();
  gyro = bno->getGyroData();
  mag = bno->getMagData();
  
  usbPort.print(F("ACC X: "));
  usbPort.print(acc.x); usbPort.print(F("  Y: "));
  usbPort.print(acc.y); usbPort.print(F("  Z: "));
  usbPort.println(acc.z);

  usbPort.print(F("Gyro X: "));
  usbPort.print(gyro.x); usbPort.print(F("  Y: "));
  usbPort.print(gyro.y); usbPort.print(F("  Z: "));
  usbPort.println(gyro.z);

  usbPort.print(F("Mag X: "));
  usbPort.print(mag.x); usbPort.print(F("  Y: "));
  usbPort.print(mag.y); usbPort.print(F("  Z: "));
  usbPort.println(mag.z); usbPort.println();

  delay(1000);
}