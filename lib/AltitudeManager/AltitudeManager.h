#ifndef ALTITUDE_MANAGER_H
#define ALTITUDE_MANAGER_H

#include "bmp388.h"
#include "LPS22HH.h"

class AltitudeManager {
public:
    AltitudeManager(I2Class *i2c, time_t updateHz = 20);
    ~AltitudeManager();

    void initiliaze(BMP_Oversampling pressureOversampling = BMP_OverSampling_4x, BMP_Oversampling temperatureOversampling = BMP_OverSampling_1x, BMP_IIR_Sampling iirSampling = BMP_IIR_3X, BMP_ODR odrSampling = BMP_ODR_40ms,
                    LPS_OUTPUT_DATA_RATE odrRate = LPS_OUTPUTDATARATE25HZ, LPS_EN_LPFP lpfp  = LPS_LPFPENABLE, LPS_BDU bdu = LPS_BDU_CONT, LPS_LOWNOISE lpsLowNoise= LPS_LOW_NOISE);
    void update();
    float getAltitude();
    float getPressure();
    float getTemperature();
    BaroData getBaroData() const;

    void setAltitudeDifference(float difference);
    void setPressureDifference(float difference);
    void setTemperatureDifference(float difference);
    
    void bmpPrint();
    void lpsPrint();
private:
    Bmp388* bmp = nullptr;
    LPS22HH* lps = nullptr;;
    BaroData bmpData = {0};
    BaroData lpsData = {0};
    BaroData managerData = {0};

    void calculateTemp();
    void calculatePressure();
    void calculateAltitude();

    void calculateValues();

    time_t lastUpdateTime;
    time_t updateInterval;

    float altitudeDifference = 50.0f;
    float pressureDifference = 20000.0f;
    float temperatureDifference = 1.5f;
};

#endif