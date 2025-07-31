#include "AltitudeManager.h"

AltitudeManager::AltitudeManager(I2Class *i2c, time_t updateHz) : lastUpdateTime(0), updateInterval(1000 / updateHz) {
    this->bmp = new Bmp388(i2c);
    this->lps = new LPS22HH(i2c);
}

AltitudeManager::~AltitudeManager() {
    if(bmp != nullptr) {
        delete bmp;
        bmp = nullptr;
    }
    if(lps != nullptr) {
        delete lps;
        lps = nullptr;
    }
}

void AltitudeManager::initiliaze(BMP_Oversampling pressureOversampling, BMP_Oversampling temperatureOversampling, BMP_IIR_Sampling iirSampling, BMP_ODR odrSampling,
                             LPS_OUTPUT_DATA_RATE odrRate, LPS_EN_LPFP lpfp, LPS_BDU bdu, LPS_LOWNOISE lpsLowNoise) {
    bmp->BMPInit(pressureOversampling, temperatureOversampling, iirSampling, odrSampling);
    lps->LPSInit(odrRate, lpfp, bdu, lpsLowNoise);
}

void AltitudeManager::update() {
    time_t currentTime = millis();
    if (currentTime - lastUpdateTime >= updateInterval) {
        this->bmpData = bmp->BMPGetData();
        this->lpsData = lps->LPSGetData();

        calculateValues();
        
        lastUpdateTime = currentTime;
    }
    return;
}

void AltitudeManager::calculateValues(){
    calculateTemp();
    calculatePressure();
    calculateAltitude();
}

void AltitudeManager::calculateTemp(){
    if(abs(bmpData.temperature - lpsData.temperature) > temperatureDifference){
        managerData.temperature = (bmpData.temperature + lpsData.temperature) / 2.0f;
    } else {
        managerData.temperature = lpsData.temperature;
    }

    return;
}

void AltitudeManager::calculatePressure(){
    if(abs(bmpData.pressure - lpsData.pressure) > pressureDifference){
        managerData.pressure = (bmpData.pressure + lpsData.pressure) / 2.0f;
    } else {
        managerData.pressure = lpsData.pressure;
    }

    return;
}

void AltitudeManager::calculateAltitude(){
    if(abs(bmpData.altitude - lpsData.altitude) > altitudeDifference){
        managerData.altitude = (bmpData.altitude + lpsData.altitude) / 2.0f;
    } else {
        managerData.altitude = lpsData.altitude;
    }

    return;
}

float AltitudeManager::getAltitude() {
    return managerData.altitude;
}

float AltitudeManager::getPressure() {
    return managerData.pressure;
}

float AltitudeManager::getTemperature() {
    return managerData.temperature;
}

void AltitudeManager::setAltitudeDifference(float difference) {
    altitudeDifference = difference;
}

void AltitudeManager::setPressureDifference(float difference) {
    pressureDifference = difference;
}

void AltitudeManager::setTemperatureDifference(float difference) {
    temperatureDifference = difference;
}