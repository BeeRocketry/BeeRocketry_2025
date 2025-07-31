#include <Arduino.h>
#include <HardwareSerial.h>

//HardwareSerial usbPort(PA10, PA9);
int cnt = 0;

#define ANALOG_PIN PB0

void setup() {
  usbPort.begin(115200);

  analogReadResolution(12);
  analogReference(AR_DEFAULT);

  usbPort.println("USB Port Started...");
}

void loop() {
  int adcVal = analogRead(ANALOG_PIN);

  double voltage = adcVal * (3.3 / 4095.0);

  usbPort.print(F("Water Level : %"));
  usbPort.println(voltage / 3.3 * 100);
  usbPort.print(F("ADC Val : "));
  usbPort.println(voltage);

  delay(500);
}