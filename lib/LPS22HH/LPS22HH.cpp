#include "LPS22HH.h"
float getPressure() {
    uint8_t buffer[3];
    readBytes(CHIP_ADDRESS, PRESSURE_OUT_XL, buffer, 3);
    uint32_t press_data = 0;
    press_data = buffer[0] | (uint32_t) buffer[1] << 8 | (uint32_t) buffer[2] << 16;
    return (float)press_data/PRES_SENS;
}
float getTemperature() {
    uint8_t buffer[2];
    readBytes(CHIP_ADDRESS, TEMP_OUT_L, buffer, 2);
    int16_t temp_data = 0;
    temp_data = buffer[0] | (uint16_t) buffer[1] << 8;
    return (float)temp_data/TEMP_SENS;
}
uint8_t readWhoAmI() {
    uint8_t buffer[1];
    readBytes(CHIP_ADDRESS, WHO_AM_I, buffer, 1);
    return buffer[0];
}