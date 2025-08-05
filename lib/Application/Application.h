#ifndef APPLICATION_H
#define APPLICATION_H

#include "AltitudeManager.h"
#include "bno055.h"
#include "e220.h"
#include "L76.h"
#include "PinSetup.h"
#include "debugprinter.h"
#include "ReefwingAHRS.h"
#include <SoftwareSerial.h>

enum ApplicationRunState : uint8_t {
    Normal_Run,
    Haberlesme_Run,
    HaberlesmeReceiver_Run,
    Algoritma_Run,
    AlgoritmaReceiver_Run,
    Fonksiyonel_Run,
    YKI_Run,
    SITSUT_Run,
    KademeAyirma_Run,
    KademeAyirmaBilgisayar_Run,
    GPS_Test,
    Haberlesme_Gorev_Run
};

typedef enum _States : uint8_t {
    STATE_RAMP,
    STATE_UNSAFE_TAKEOFF,
    STATE_SAFE_TAKEOFF,
    STATE_FIRST_BOMB,
    STATE_SECOND_BOMB
} States;

typedef enum SutSitStates : uint8_t {
    STATE_SIT,
    STATE_SUT,
    SUT_SIT_IDLE
} SutSitStates;

class Application {
public:
    Application(ApplicationRunState State);
    ~Application();

    void initialize();
    void run();
private:
    I2Class *I2CBaroBus = nullptr;
    I2Class *I2CImuBus = nullptr;
    HardwareSerial *DebugSerial = nullptr;
    HardwareSerial *GPSSerial = nullptr;
    HardwareSerial *RFSerial = nullptr;
    
    HardwareSerial *RS232Serial = nullptr;
    SoftwareSerial *RFSoftSerial = nullptr;
    SoftwareSerial *GPSSoftSerial = nullptr;

    ReefwingAHRS ahrs;

    AltitudeManager *altitudeManager = nullptr;
    Bno055 *imu = nullptr;
    E220 *rfModule = nullptr;
    L76 *gpsModule = nullptr;

    ApplicationRunState runState;

    BaroData baroData;
    GpsData gpsData;
    ImuData imuData;

    void getData();

    void rampAltitude();
    void caseCheck();
    int altitudeCircularInWrapper(int);

    void haberlesmeRun();
    void haberlesmePrint();
    void haberlesmeReceiverRun();
    void algoritmaRun();
    void algoritmaReceiverRun();
    void fonksiyonelRun();
    void fonksiyonelPrint();
    void kademeAyirma();
    void kademeAyirmaBilgisayar();
    void ykiRun();
    void sitSutRun();
    void gpsTestRun();

    void resetCnt();

    int rampaDegerTotalCount = 200;
    
    States rocketState = STATE_RAMP;
    float rampAlt = 0.0f;
    
    float altitudeCircular[5] = {0};
    int altitudeCircularIn = 0;
    int rampCnt = 0;
    int safeCnt = 0;
    int deriCnt = 0;
    int accCnt = 0;

    time_t apogeeTime = 0;

    uint8_t sutSelfState = 0;
};

#endif