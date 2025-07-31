#include <Arduino.h>
#include <HardwareSerial.h>

#include "bno055.h"
#include "bmp388Class.h"

#define UART_TX PA2
#define UART_RX PA3

#define SDA_Pin PA7
#define SCL_Pin PA8

#define GPIO1 PB0
#define GPIO2 PB1
#define GPIO3 PB2
#define GPIO4 PB3
#define GPIO5 PB4

#define BuzzerPin PB5

//HardwareSerial usbPort(UART_RX, UART_TX);

I2Class* _i2c;
Bno055* bno;
bmp388* bmp;

typedef enum _States : uint8_t {
    STATE_RAMP,
    STATE_UNSAFE_TAKEOFF,
    STATE_SAFE_TAKEOFF,
    STATE_FIRST_BOMB,
    STATE_SECOND_BOMB
} States;

float rampAlt = 0;
States rocketState = STATE_RAMP;

int rampCnt = 0;
int safeCnt = 0;
int deriCnt = 0;
int accCnt = 0;

long timestamp = 0;
long timestamp2 = 0;

bool buzzerFlag = false;

float buffer[5] = {0};
int bufferIn = 0;

int wrap(int i) {
    return (i + 5) % 5;
}

void buzzerCheck(){
    if(micros() > timestamp + 1000) return;

    if(micros() > timestamp2 + 250){
        timestamp2 = micros();

        if(buzzerFlag == true){
            buzzerFlag = false;
            digitalWrite(BuzzerPin, HIGH);
        }
        else{
            buzzerFlag = true;
            digitalWrite(BuzzerPin, LOW);
        }
    }
}

void setup(){
    usbPort.begin(115200);
    
    pinMode(BuzzerPin, OUTPUT);

    pinMode(GPIO1, OUTPUT);
    pinMode(GPIO2, OUTPUT);
    pinMode(GPIO3, OUTPUT);
    pinMode(GPIO4, OUTPUT);
    pinMode(GPIO5, OUTPUT);

    digitalWrite(GPIO1, LOW);
    digitalWrite(GPIO2, LOW);
    digitalWrite(GPIO3, LOW);
    digitalWrite(GPIO4, LOW);
    digitalWrite(GPIO5, LOW);

    digitalWrite(BuzzerPin, LOW);

    _i2c = new I2Class(SDA_Pin, SCL_Pin);

    bno = new Bno055(_i2c);
    bmp = new bmp388(_i2c);

    bno->BNOInit();
    bmp->BMPInit(BMP_OverSampling_4x, BMP_OverSampling_1x, BMP_IIR_3X, BMP_ODR_40ms);

    float totalAlt = 0;
    for(int i = 0; i < 400; i++){
        BMP_SensorData data = bmp->BMPGetData();

        totalAlt += data.altitude;
        delay(50);
    }
    rampAlt = totalAlt / 400;

    digitalWrite(GPIO1, HIGH);
}

void loop(){
    BMP_SensorData data = bmp->BMPGetData();
    BNO_DOF3_Float acc, gyro, mag;

    bufferIn = bufferIn % 5;
    buffer[bufferIn] = data.altitude;

    acc = bno->getAccData();
    gyro = bno->getGyroData();
    mag = bno->getMagData();

    switch (rocketState)
    {
    case STATE_RAMP:
        if(!(abs(data.altitude - rampAlt) > 20)){
            rampCnt = 0;
            break;
        }

        rampCnt++;

        if(rampCnt > 5){
            rocketState = STATE_UNSAFE_TAKEOFF;
            digitalWrite(GPIO2, HIGH);
        }
        break;
    
    case STATE_UNSAFE_TAKEOFF:
        if(!(abs(data.altitude - rampAlt) > 100)){
            safeCnt = 0;
            break;
        }

        safeCnt++;

        if(safeCnt > 5){
            rocketState = STATE_SAFE_TAKEOFF;
            digitalWrite(GPIO3, HIGH);
        }
        break;

    case STATE_SAFE_TAKEOFF:
        float totalG = sqrtf(powf(acc.x, 2) + powf(acc.y, 2) + powf(acc.z, 2));

        int i0 = wrap(bufferIn - 1);
        int i1 = wrap(bufferIn - 2);
        int i2 = wrap(bufferIn - 3);
        int i3 = wrap(bufferIn - 4);
        int i4 = wrap(bufferIn - 4);

        float res = (25*buffer[i0] - 48*buffer[i1] + 36*buffer[i2] - 16*buffer[i3] + 3*buffer[i4]) / 12*1.0;

        if(res <= 0){
            deriCnt++;
        }

        if(totalG < 0.3){
            accCnt++;
        }

        if(deriCnt > 5 && accCnt > 5){
            rocketState = STATE_FIRST_BOMB;
            digitalWrite(GPIO4, HIGH);
            timestamp = micros();
            timestamp2 = micros();
        }
        break;
    
    case STATE_FIRST_BOMB:
        if(!(micros() > timestamp + 1000)) buzzerCheck();

        digitalWrite(BuzzerPin, LOW);

        if(abs(data.altitude - rampAlt) < 100){
            rocketState = STATE_SECOND_BOMB;
            digitalWrite(GPIO5, HIGH);
        }
        break;

    case STATE_SECOND_BOMB:
        break;
    }
    delay(50);
}