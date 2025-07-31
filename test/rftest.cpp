#include <Arduino.h>
#include <HardwareSerial.h>

#include "e32.h"

#define M0_Pin PA7
#define M1_Pin PA6
#define AUX_Pin PA5

//HardwareSerial usbPort(PA10, PA9);
HardwareSerial rfPort(PA3, PA2);

E32_433T30D* e32Obj = nullptr;

void setup() {
  usbPort.begin(115200);
  usbPort.flush();
  usbPort.println("Seri Port Baslatildi...");
  usbPort.println("RF Baslatiliyor...");

  pinMode(AUX_Pin, INPUT_PULLUP);
  pinMode(M0_Pin, OUTPUT);
  pinMode(M1_Pin, OUTPUT);

  delay(1000);

  e32Obj = new E32_433T30D(&rfPort, AUX_Pin, M0_Pin, M1_Pin, &usbPort);

  e32Obj->RFBegin(0x05, 0x03, RF_FREQ::FREQ_414, RF_UART_PARITY::UARTPARITY_8N1, RF_UART_BAUD::UARTBAUDRATE_115200, 
                 RF_AIR_DATA::AIRDATARATE_24k, RF_TRANS_MODE::FIXEDMODE, RF_IO_MODE::IO_PUSHPULL, RF_WIRELESS::WIRELESSWAKEUP_500, RF_FEC::FEC_ON, RF_TRANS_POWER::TRANSMISSIONPOWER_30);

  e32Obj->viewSettings();
  usbPort.println("RF Baslatildi...");
}

int cnt = 0;

void loop() {

}