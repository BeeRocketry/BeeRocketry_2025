#include <Arduino.h>
#include <HardwareSerial.h>

#include "bno055.h"
#include "bmp388Class.h"

#define UART_TX PA2
#define UART_RX PA3

#define SDA_Pin PB7
#define SCL_Pin PB8

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
    STATE_SECOND_BOMB,
} States;

float rampAlt = 0;
States rocketState = STATE_RAMP;

int rampCnt = 0;
int safeCnt = 0;
int deriCnt = 0;
int accCnt = 0;

long timestamp = 0;
long timestamp2 = 0;

long veriStamp = 0;

bool buzzerFlag = false;

float buffer[5] = {0};
int bufferIn = 0;

bool flag1 = false;
bool flag2 = false;

float totalG;
double res;
int i0, i1, i2, i3, i4, i5;

int wrap(int i) {
    return (i + 5) % 5;
}

void buzzerCheck(){
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
    delay(250);
    digitalWrite(BuzzerPin, LOW);
    delay(250);
    digitalWrite(BuzzerPin, HIGH);
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
    bmp = new bmp388(_i2c, BMP_OverSampling_4x, BMP_OverSampling_1x, BMP_IIR_3X, BMP_ODR_40ms);

    bno->BNOInit();

    float totalAlt = 0;
    for(int i = 0; i < 200; i++){
        BMP_SensorData data = bmp->BMPGetData();

        totalAlt += data.altitude;
        delay(50);
    }
    rampAlt = totalAlt / 200;

    

    usbPort.print(F("Rampa İrtifa Degeri: "));
    usbPort.println(rampAlt);
    delay(750);

    digitalWrite(GPIO1, HIGH);

    veriStamp = millis();
}

void loop(){
    BMP_SensorData data = bmp->BMPGetData();
    BNO_DOF3_Float acc, gyro, mag;

    bufferIn = bufferIn % 5;
    buffer[bufferIn++] = abs(data.altitude - rampAlt);

    acc = bno->getAccData();
    gyro = bno->getGyroData();
    mag = bno->getMagData();

    if(millis() > veriStamp + 250){
        veriStamp = millis();

        usbPort.println(F("-----------------------------------------------------------"));
        usbPort.print(F("Sicaklik: ")); usbPort.print(data.temperature);
        usbPort.print(F("   Basinc: ")); usbPort.print(data.pressure);
        usbPort.print(F("   Irtifa: ")); usbPort.print(data.altitude);
        usbPort.print(F("   Rampa Irtifa: ")); usbPort.println(abs(data.altitude - rampAlt));

        usbPort.println();

        usbPort.println(F("Ivme"));
        usbPort.print(F("X: ")); usbPort.print(acc.x);
        usbPort.print(F("   Y: ")); usbPort.print(acc.y);
        usbPort.print(F("   Z: ")); usbPort.println(acc.z);

        usbPort.println(F("Acisal Hiz"));
        usbPort.print(F("X: ")); usbPort.print(gyro.x);
        usbPort.print(F("   Y: ")); usbPort.print(gyro.y);
        usbPort.print(F("   Z: ")); usbPort.println(gyro.z);

        usbPort.println(F("Manyetik Alan"));
        usbPort.print(F("X: ")); usbPort.print(mag.x);
        usbPort.print(F("   Y: ")); usbPort.print(mag.y);
        usbPort.print(F("   Z: ")); usbPort.println(mag.z);
        usbPort.println(F("-----------------------------------------------------------"));
        usbPort.println();
    }

    switch (rocketState)
    {
    case STATE_RAMP:
        if(!(abs(data.altitude - rampAlt) > 5)){
            rampCnt = 0;
            break;
        }

        rampCnt++;

        if(rampCnt > 5){
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            usbPort.println(F("Rampa Durum Değişkeni Değişti"));
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            rocketState = STATE_UNSAFE_TAKEOFF;
            digitalWrite(GPIO2, HIGH);
        }
        break;
    
    case STATE_UNSAFE_TAKEOFF:
        if(!(abs(data.altitude - rampAlt) > 25)){
            safeCnt = 0;
            break;
        }

        safeCnt++;

        if(safeCnt > 5){
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            usbPort.println(F("Güvenli Irtifa Durum Değişkeni Değişti"));
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            rocketState = STATE_SAFE_TAKEOFF;
            digitalWrite(GPIO3, HIGH);
        }
        break;

    case STATE_SAFE_TAKEOFF:
        totalG = sqrtf(powf(acc.x, 2) + powf(acc.y, 2) + powf(acc.z, 2));

        i0 = wrap(bufferIn - 1);
        i1 = wrap(bufferIn - 2);
        i2 = wrap(bufferIn - 3);
        i3 = wrap(bufferIn - 4);
        i4 = wrap(bufferIn - 4);

        res = (25.0*buffer[i0] - 48.0*buffer[i1] + 36.0*buffer[i2] - 16.0*buffer[i3] + 3.0*buffer[i4]) / 12*1.0;

        if(res <= -4.5){
            deriCnt++;
            
        }

        if(totalG < 0.6){
            accCnt++;
        }

        if(deriCnt > 3 && flag1 == false){
            flag1 = true;
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            usbPort.println(F("İrtifa Değişim Veri Değişkeni Değişti"));
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            digitalWrite(GPIO4, HIGH);
        }
        
        if(accCnt > 1 && flag2 == false){
            flag2 = true;
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            usbPort.println(F("Mutlak Düşey İvme Değişkeni Değişti"));
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            digitalWrite(GPIO5, HIGH);
        }

        if(deriCnt > 3 && accCnt > 1){
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            usbPort.println(F("Birincil Kurtarma Tetiklendi"));
            usbPort.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            rocketState = STATE_FIRST_BOMB;
            timestamp = millis();
            timestamp2 = millis();
        }
        break;
    
    case STATE_FIRST_BOMB:
        buzzerCheck();

        delay(1000);

        if(abs(data.altitude - rampAlt) < 100){
            rocketState = STATE_SECOND_BOMB;
            digitalWrite(GPIO1, LOW);
            digitalWrite(GPIO2, LOW);
            digitalWrite(GPIO3, LOW);
            digitalWrite(GPIO4, LOW);
            digitalWrite(GPIO5, LOW);
        }
        break;
    }
    delay(70);
}