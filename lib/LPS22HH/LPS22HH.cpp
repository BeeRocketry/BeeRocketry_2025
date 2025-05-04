#include "LPS22HH.h"

LPS22HH::LPS22HH(I2Class* _i2c) {
    this->i2c = _i2c;
}

uint8_t LPS22HH::lpsReadBytes(uint8_t regadr, uint8_t* temp, uint8_t length, uint16_t timeout){
    return this->i2c->I2CReadBytes(CHIP_ADDRESS, regadr, temp, length, timeout);
}

float LPS22HH::getPressure() {
    uint8_t buffer[3] = {0};
    uint32_t press_data = 0;

    this->lpsReadBytes(PRESSURE_OUT_XL, buffer, 3, TIMEOUT_I2C);
    press_data = buffer[0] | ((uint32_t) buffer[1] << 8) | ((uint32_t) buffer[2] << 16);
    
    return (float)press_data/PRES_SENS;
}

float LPS22HH::getTemperature() {
    uint8_t buffer[2] = {0};
    int16_t temp_data = 0;
    
    this->lpsReadBytes(TEMP_OUT_L, buffer, 2);
    temp_data = buffer[0] | (uint16_t) buffer[1] << 8;
    return (float)temp_data/TEMP_SENS;
}

bool LPS22HH::readWhoAmI() {
    uint8_t buffer = 0;
    this->lpsReadBytes(WHO_AM_I, &buffer, 1);
    return buffer == 0b10110011;
}