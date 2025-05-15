#include <Arduino.h>
#include <HardwareSerial.h>

#include "e32.h"
#include "bno055.h"

#define M0_Pin PA7
#define M1_Pin PA6
#define AUX_Pin PA5

#define RF_UART_TX PA9
#define RF_UART_RX PA10

#define PC_UART_TX PA2
#define PC_UART_RX PA3

#define SDA_Pin PB7
#define SCL_Pin PB6

#define RF_ADDR_LOW 0x03
#define RF_ADDR_HIGH 0x05
#define RF_CHAN RF_FREQ::FREQ_414

#define RF_SEND_ADDR_HIGH 0x01
#define RF_SEND_ADDR_LOW 0x09

#define PACKET_SIZE 36

HardwareSerial usbPort(PC_UART_RX, PC_UART_TX);
HardwareSerial rfPort(RF_UART_RX, RF_UART_TX);

E32_433T30D *e32Obj = nullptr;
I2Class* i2c_ = nullptr;
Bno055 *bnoObj = nullptr;

BNO_DOF3_Float acc, gyro, mag;

void printValues();

uint8_t buffer[PACKET_SIZE];

void setup()
{
    usbPort.begin(115200);
    usbPort.flush();
    usbPort.println("Seri Port Baslatildi...");
    usbPort.println("RF Baslatiliyor...");

    pinMode(AUX_Pin, INPUT_PULLUP);
    pinMode(M0_Pin, OUTPUT);
    pinMode(M1_Pin, OUTPUT);

    delay(1000);

    e32Obj = new E32_433T30D(&rfPort, AUX_Pin, M0_Pin, M1_Pin, &usbPort);

    e32Obj->RFBegin(RF_ADDR_HIGH, RF_ADDR_LOW, RF_CHAN, RF_UART_PARITY::UARTPARITY_8N1, RF_UART_BAUD::UARTBAUDRATE_115200,
                    RF_AIR_DATA::AIRDATARATE_24k, RF_TRANS_MODE::FIXEDMODE, RF_IO_MODE::IO_PUSHPULL, RF_WIRELESS::WIRELESSWAKEUP_500, RF_FEC::FEC_ON, RF_TRANS_POWER::TRANSMISSIONPOWER_30);

    e32Obj->viewSettings();
    usbPort.println("RF Baslatildi...");

    usbPort.println("I2C Baslatiliyor...");
    i2c_ = new I2Class(SDA_Pin, SCL_Pin);
    usbPort.println("I2C Baslatildi...");

    usbPort.println("BNO055 Baslatiliyor...");
    bnoObj = new Bno055(i2c_);

    bnoObj->BNOInit();

    usbPort.println("BNO055 Baslatildi...");

    delay(3000);
}

void loop()
{
    usbPort.println(F("-----------------------------------------------"));
    acc = bnoObj->getAccData();
    gyro = bnoObj->getGyroData();
    mag = bnoObj->getMagData();

    printValues();

    memcpy(buffer, &acc, sizeof(BNO_DOF3_Float));
    memcpy(&buffer[12], &gyro, sizeof(BNO_DOF3_Float));
    memcpy(&buffer[24], &mag, sizeof(BNO_DOF3_Float));

    e32Obj->sendFixedDataPacket(RF_SEND_ADDR_HIGH, RF_SEND_ADDR_LOW, RF_CHAN, buffer, PACKET_SIZE);

    usbPort.println(F("Veri Paketi Gonderildi..."));

    usbPort.println(F("-----------------------------------------------"));
    usbPort.println();
    
    delay(1000);
}

void printValues(){
    usbPort.print(F("ACC X: "));
    usbPort.print(acc.x);
    usbPort.print(F("  Y: "));
    usbPort.print(acc.y);
    usbPort.print(F("  Z: "));
    usbPort.println(acc.z);

    usbPort.print(F("Gyro X: "));
    usbPort.print(gyro.x);
    usbPort.print(F("  Y: "));
    usbPort.print(gyro.y);
    usbPort.print(F("  Z: "));
    usbPort.println(gyro.z);

    usbPort.print(F("Mag X: "));
    usbPort.print(mag.x);
    usbPort.print(F("  Y: "));
    usbPort.print(mag.y);
    usbPort.print(F("  Z: "));
    usbPort.println(mag.z);
    usbPort.println();
}