#include "main.h"

Application *app;

void setup(){
    //MPU_Config();
    //HAL_Init();
    SystemClock_Config();
    //MX_GPIO_Init();
    
    app = new Application(SITSUT_Run);
    app->run();
}

void loop(){

}