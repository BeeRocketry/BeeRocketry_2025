#include <Arduino.h>
#include <HardwareSerial.h>

#include "Application.h"

Application app(Haberlesme_Run);

void setup(){
    app.run();
}

void loop(){

}