#include "Application.h"

HardwareSerial usbPort(DEBUG_TX_PIN, DEBUG_RX_PIN);

Application::Application(ApplicationRunState State) : runState(State) {
    DebugSerial = &usbPort;
    DebugSerial->begin(115200);
    
    switch (runState)
    {
    case Haberlesme_Run:
        GPSSerial = new HardwareSerial(GPS_RX_PIN, GPS_TX_PIN);
        RFSerial = new HardwareSerial(RF_RX_PIN, RF_TX_PIN);

        rfModule = new E220(RFSerial, RF_AUX_PIN, RF_M0_PIN, RF_M1_PIN, DebugSerial);
        gpsModule = new L76(GPSSerial, DebugSerial);
        break;
    
    case Normal_Run:
        GPSSerial = new HardwareSerial(GPS_RX_PIN, GPS_TX_PIN);
        I2CImuBus = new I2Class(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
        I2CBaroBus = new I2Class(BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN);
        RFSerial = new HardwareSerial(RF_RX_PIN, RF_TX_PIN);

        altitudeManager = new AltitudeManager(I2CBaroBus);
        imu = new Bno055(I2CImuBus);
        rfModule = new E220(RFSerial, RF_AUX_PIN, RF_M0_PIN, RF_M1_PIN, DebugSerial);
        gpsModule = new L76(GPSSerial, DebugSerial);
        break;

    case Algoritma_Run:
        I2CImuBus = new I2Class(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
        I2CBaroBus = new I2Class(BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN);
        RFSerial = new HardwareSerial(RF_RX_PIN, RF_TX_PIN);

        altitudeManager = new AltitudeManager(I2CBaroBus);
        imu = new Bno055(I2CImuBus);
        rfModule = new E220(RFSerial, RF_AUX_PIN, RF_M0_PIN, RF_M1_PIN, DebugSerial);
        break;

    case HaberlesmeReceiver_Run:
    case AlgoritmaReceiver_Run:
        RFSerial = new HardwareSerial(RF_RX_PIN, RF_TX_PIN);

        rfModule = new E220(RFSerial, RF_AUX_PIN, RF_M0_PIN, RF_M1_PIN, DebugSerial);
        break;
    
    case Fonksiyonel_Run:
        I2CImuBus = new I2Class(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
        I2CBaroBus = new I2Class(BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN);
        
        altitudeManager = new AltitudeManager(I2CBaroBus);
        imu = new Bno055(I2CImuBus);
        break;
    
    case KademeAyirmaBilgisayar_Run:
    case KademeAyirma_Run:
        RFSerial = new HardwareSerial(RF_RX_PIN, RF_TX_PIN);

        rfModule = new E220(RFSerial, RF_AUX_PIN, RF_M0_PIN, RF_M1_PIN, DebugSerial);
        break;

    case YKI_Run:
        GPSSerial = new HardwareSerial(GPS_RX_PIN, GPS_TX_PIN);
        I2CImuBus = new I2Class(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
        I2CBaroBus = new I2Class(BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN);
        
        altitudeManager = new AltitudeManager(I2CBaroBus);
        imu = new Bno055(I2CImuBus);
        gpsModule = new L76(GPSSerial, DebugSerial);
        break;

    case SITSUT_Run:
        RS232Serial = new HardwareSerial(RS232_RX_PIN, RS232_TX_PIN);
        GPSSerial = new HardwareSerial(GPS_RX_PIN, GPS_TX_PIN);
        I2CImuBus = new I2Class(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
        I2CBaroBus = new I2Class(BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN);
        
        altitudeManager = new AltitudeManager(I2CBaroBus);
        imu = new Bno055(I2CImuBus);
        gpsModule = new L76(GPSSerial, DebugSerial);
        break;
    }
}

Application::~Application() {
    // Todo: State'e göre temizleme işlemleri yapılabilir.
    if (RS232Serial != nullptr) {
        RS232Serial->end();
        delete RS232Serial;
        RS232Serial = nullptr;
    }
    if (I2CBaroBus != nullptr) {
        delete I2CBaroBus;
        I2CBaroBus = nullptr;
    }
    if (I2CImuBus != nullptr) {
        delete I2CImuBus;
        I2CImuBus = nullptr;
    }
    if (DebugSerial != nullptr) {
        DebugSerial->end();
        DebugSerial = nullptr;
    }
    if (GPSSerial != nullptr) {
        delete GPSSerial;
        GPSSerial = nullptr;
    }
    if (RFSerial != nullptr) {
        delete RFSerial;
        RFSerial = nullptr;
    }
    if (altitudeManager != nullptr) {
        delete altitudeManager;
        altitudeManager = nullptr;
    }
    if (imu != nullptr) {
        delete imu;
        imu = nullptr;
    }
    if (rfModule != nullptr) {
        delete rfModule;
        rfModule = nullptr;
    }
    if (gpsModule != nullptr) {
        delete gpsModule;
        gpsModule = nullptr;
    }
}

void Application::initialize() {
    altitudeManager->initiliaze(CONFIG_BMP_PRESSURE_OVERSAMPLING, CONFIG_BMP_TEMPERATURE_OVERSAMPLING,
                                CONFIG_BMP_IIR_SAMPLING, CONFIG_BMP_ODR_SAMPLING,
                                CONFIG_LPS_ODR_RATE, CONFIG_LPS_LPFP, CONFIG_LPS_BDU, CONFIG_LPS_LOW_NOISE);
    imu->BNOInit();
    rfModule->RFBegin(CONFIG_RF_HIGH_ADDR, CONFIG_RF_LOW_ADDR, CONFIG_RF_CHANNEL, CONFIG_RF_BAUDRATE,
                      CONFIG_RF_AIR_DATA_RATE, CONFIG_RF_RSSI_BYTE, CONFIG_RF_PACKET_SIZE,
                      CONFIG_RF_TRANSFER_MODE, CONFIG_RF_TRANSFER_POWER, CONFIG_RF_WIRELESS_WAKEUP,
                      CONFIG_RF_UART_PARITY, CONFIG_RF_RSSI_AMBIENT, CONFIG_RF_LBT_ENABLE);
    gpsModule->begin();

    DebugSerial->println(F("\nInitialize Basariyla Baslatildi.\n"));
}

void Application::run() {
    switch (runState)
    {
    case Haberlesme_Run:
        haberlesmeRun();
        break;
    
    case HaberlesmeReceiver_Run:
        haberlesmeReceiverRun();
        break;

    case Algoritma_Run:
        algoritmaRun();
        break;

    case AlgoritmaReceiver_Run:
        algoritmaReceiverRun();
        break;

    case Fonksiyonel_Run:
        fonksiyonelRun();
        break;

    case KademeAyirma_Run:
        kademeAyirma();
        break;

    case KademeAyirmaBilgisayar_Run:
        kademeAyirmaBilgisayar();
        break;

    case YKI_Run:
        ykiRun();
        break;

    case SITSUT_Run:
        sitSutRun();
        break;
    }
}

void Application::getData() {
    altitudeManager->update();
    baroData = altitudeManager->getBaroData();
    
    gpsModule->getData();
    gpsData = gpsModule->getDataStruct();

    imuData.acc = imu->getAccData();
    imuData.gyro = imu->getGyroData();
    imuData.mag = imu->getMagData();
}

void Application::haberlesmeRun() {
    rfModule->RFBegin(CONFIG_RF_HIGH_ADDR, CONFIG_RF_LOW_ADDR, CONFIG_RF_CHANNEL, CONFIG_RF_BAUDRATE,
                      CONFIG_RF_AIR_DATA_RATE, CONFIG_RF_RSSI_BYTE, CONFIG_RF_PACKET_SIZE,
                      CONFIG_RF_TRANSFER_MODE, CONFIG_RF_TRANSFER_POWER, CONFIG_RF_WIRELESS_WAKEUP,
                      CONFIG_RF_UART_PARITY, CONFIG_RF_RSSI_AMBIENT, CONFIG_RF_LBT_ENABLE);

    gpsModule->begin();

    DebugSerial->println(F("Haberlesme Modu Baslatildi."));

    rfModule->viewSettings();

    uint8_t packet[sizeof(GpsData)];

    while(true){
        gpsModule->getData();
        gpsData = gpsModule->getDataStruct();

        haberlesmePrint();

        memset(packet, 0, sizeof(packet));
        memcpy(packet, &gpsData, sizeof(GpsData));

        Status status = rfModule->send(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, packet, sizeof(GpsData));

        DEBUG_PRINTLN(F("Veri Gonderildi"));

        delay(3000);
    }
}

void Application::haberlesmePrint() {
    DEBUG_PRINT(F("\nEnlem :")); DEBUG_PRINT(gpsData.latitude);
    DEBUG_PRINT(F(" Boylam :")); DEBUG_PRINT(gpsData.longitude);
    DEBUG_PRINT(F(" Yukseklik :")); DEBUG_PRINTLN(gpsData.altitude);
    DEBUG_PRINTLN(F("------------------------------"));DEBUG_PRINTLN();
}

void Application::haberlesmeReceiverRun() {
    rfModule->RFBegin(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, CONFIG_RF_BAUDRATE,
                      CONFIG_RF_AIR_DATA_RATE, CONFIG_RF_RSSI_BYTE, CONFIG_RF_PACKET_SIZE,
                      CONFIG_RF_TRANSFER_MODE, CONFIG_RF_TRANSFER_POWER, CONFIG_RF_WIRELESS_WAKEUP,
                      CONFIG_RF_UART_PARITY, CONFIG_RF_RSSI_AMBIENT, CONFIG_RF_LBT_ENABLE);

    DebugSerial->println(F("Haberlesme Modu Baslatildi."));

    rfModule->viewSettings();

    RF_Msg msg;

    while(true){
        msg = rfModule->receive();
        if(E220_Success == msg.status){
            memcpy(&gpsData, msg.buffer, sizeof(GpsData));

            DEBUG_PRINTLN(F("Veri Alindi"));

            haberlesmePrint();
        }
    }
}

void Application::rampAltitude() {
    float totalAlt = 0.0f;
    for(int i = 0; i < rampaDegerTotalCount; i++){
        altitudeManager->update();
        BaroData data = altitudeManager->getBaroData();
        totalAlt += data.altitude;
        delay(50);
    }

    rampAlt = totalAlt / rampaDegerTotalCount;
}

void Application::algoritmaRun() {
    altitudeManager->initiliaze(CONFIG_BMP_PRESSURE_OVERSAMPLING, CONFIG_BMP_TEMPERATURE_OVERSAMPLING,
                                CONFIG_BMP_IIR_SAMPLING, CONFIG_BMP_ODR_SAMPLING,
                                CONFIG_LPS_ODR_RATE, CONFIG_LPS_LPFP, CONFIG_LPS_BDU, CONFIG_LPS_LOW_NOISE);
    imu->BNOInit();
    rfModule->RFBegin(CONFIG_RF_HIGH_ADDR, CONFIG_RF_LOW_ADDR, CONFIG_RF_CHANNEL, CONFIG_RF_BAUDRATE,
                      CONFIG_RF_AIR_DATA_RATE, CONFIG_RF_RSSI_BYTE, CONFIG_RF_PACKET_SIZE,
                      CONFIG_RF_TRANSFER_MODE, CONFIG_RF_TRANSFER_POWER, CONFIG_RF_WIRELESS_WAKEUP,
                      CONFIG_RF_UART_PARITY, CONFIG_RF_RSSI_AMBIENT, CONFIG_RF_LBT_ENABLE);

    DebugSerial->println(F("Algortima Modu Baslatildi."));

    rfModule->viewSettings();
    
    pinMode(BUZZER_PIN, OUTPUT);

    rampAltitude();
    DEBUG_PRINT(F("Rampa Yuksekligi : ")); DEBUG_PRINTLN(rampAlt);

    while(true){
        altitudeManager->update();
        baroData = altitudeManager->getBaroData();

        imuData.acc = imu->getAccData();

        imuData.gyro = imu->getGyroData();
        imuData.mag = imu->getMagData();

        altitudeCircularIn = altitudeCircularIn % 5;
        altitudeCircular[altitudeCircularIn] = baroData.altitude;

        caseCheck();

        delay(50);
    }
}

void Application::caseCheck(){
    switch (rocketState)
    {
    case STATE_RAMP:
        {
            if(!(abs(baroData.altitude - rampAlt) > 20)){
                rampCnt = 0;
                break;
            }

            rampCnt++;

            if(rampCnt > 5){
                rocketState = STATE_UNSAFE_TAKEOFF;
                sutSelfState |= 0b1;
                rfModule->sendStatus(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, "Rampa Durum Değişkeni Değişti");
            }
        }
        break;
    
    case STATE_UNSAFE_TAKEOFF:
        {
            if(!(abs(baroData.altitude - rampAlt) > 100)){
                safeCnt = 0;
                break;
            }

            safeCnt++;

            if(safeCnt > 5){
                rocketState = STATE_SAFE_TAKEOFF;
                sutSelfState |= 0b1 << 2;
                rfModule->sendStatus(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, "Güvenli Sınır Durum Değişkeni Değişti");
            }
        }
        break;

    case STATE_SAFE_TAKEOFF:
        {
            float totalG = sqrtf(powf(imuData.acc.x, 2) + powf(imuData.acc.y, 2) + powf(imuData.acc.z, 2));

            int i0 = altitudeCircularInWrapper(altitudeCircularIn - 1);
            int i1 = altitudeCircularInWrapper(altitudeCircularIn - 2);
            int i2 = altitudeCircularInWrapper(altitudeCircularIn - 3);
            int i3 = altitudeCircularInWrapper(altitudeCircularIn - 4);
            int i4 = altitudeCircularInWrapper(altitudeCircularIn - 4);

            float res = (25*altitudeCircular[i0] - 48*altitudeCircular[i1] + 36*altitudeCircular[i2] - 16*altitudeCircular[i3] + 3*altitudeCircular[i4]) / 12*1.0;

            if(res <= 0){
                deriCnt++;
                sutSelfState |= 0b1 << 4;
            }

            if(totalG < 0.3){
                accCnt++;
            }

            if(deriCnt > 5 && accCnt > 5){
                rocketState = STATE_FIRST_BOMB;
                rfModule->sendStatus(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, "Apogee Algilandi");
                sutSelfState |= 0b1 << 5;

                if(runState != SITSUT_Run){
                    digitalWrite(BUZZER_PIN, HIGH);
                    apogeeTime = millis();
                }
            }
        }
        break;
    
    case STATE_FIRST_BOMB:
        {
            if(runState != SITSUT_Run && millis() - apogeeTime > 500){
                digitalWrite(BUZZER_PIN, LOW);
            }

            if(abs(baroData.altitude - rampAlt) < 100){
                rocketState = STATE_SECOND_BOMB;
                sutSelfState |= 0b1 << 6;
                rfModule->sendStatus(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, "Main Algilandi");
            }
        }
        break;

    case STATE_SECOND_BOMB:
        sutSelfState |= 0b1 << 7;
        break;
    }
}

int Application::altitudeCircularInWrapper(int i) {
    return (i + 5) % 5;
}

void Application::algoritmaReceiverRun() {
    rfModule->RFBegin(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, CONFIG_RF_BAUDRATE,
                      CONFIG_RF_AIR_DATA_RATE, CONFIG_RF_RSSI_BYTE, CONFIG_RF_PACKET_SIZE,
                      CONFIG_RF_TRANSFER_MODE, CONFIG_RF_TRANSFER_POWER, CONFIG_RF_WIRELESS_WAKEUP,
                      CONFIG_RF_UART_PARITY, CONFIG_RF_RSSI_AMBIENT, CONFIG_RF_LBT_ENABLE);

    DebugSerial->println(F("Algoritma Modu Baslatildi."));

    rfModule->viewSettings();

    RF_Msg msg;

    while(true){
        msg = rfModule->receive();
        if(E220_Success == msg.status && msg.isMessage == true){
            DEBUG_PRINTLN(msg.message);
        }
    }
}

void Application::fonksiyonelRun() {
    altitudeManager->initiliaze(CONFIG_BMP_PRESSURE_OVERSAMPLING, CONFIG_BMP_TEMPERATURE_OVERSAMPLING,
                                CONFIG_BMP_IIR_SAMPLING, CONFIG_BMP_ODR_SAMPLING,
                                CONFIG_LPS_ODR_RATE, CONFIG_LPS_LPFP, CONFIG_LPS_BDU, CONFIG_LPS_LOW_NOISE);
    imu->BNOInit();

    ahrs.begin();
    ahrs.setFusionAlgorithm(SensorFusion::EXTENDED_KALMAN);
    ahrs.setDeclination(6.13f);
    ahrs.setDOF(DOF::DOF_9);

    DebugSerial->println(F("Fonksiyonel Modu Baslatildi."));

    SensorData tempData;

    while(true){
        altitudeManager->update();
        baroData = altitudeManager->getBaroData();

        imuData.acc = imu->getAccData();
        tempData.aTimeStamp = millis();
        imuData.gyro = imu->getGyroData();
        tempData.gTimeStamp = millis();
        imuData.mag = imu->getMagData();
        tempData.mTimeStamp = millis();

        tempData.ax = imuData.acc.x;
        tempData.ay = imuData.acc.y;
        tempData.az = imuData.acc.z;
        tempData.gx = imuData.gyro.x;
        tempData.gy = imuData.gyro.y;
        tempData.gz = imuData.gyro.z;
        tempData.mx = imuData.mag.x;
        tempData.my = imuData.mag.y;
        tempData.mz = imuData.mag.z;

        ahrs.setData(tempData);
        ahrs.update();

        fonksiyonelPrint();

        delay(700);
    }
}

void Application::fonksiyonelPrint() {
    DEBUG_PRINT(F("\nBaro Yuksekligi : ")); DEBUG_PRINT(baroData.altitude);
    DEBUG_PRINT(F(" Barometre Basinci : ")); DEBUG_PRINT(baroData.pressure);
    DEBUG_PRINT(F(" Barometre Sicakligi : ")); DEBUG_PRINTLN(baroData.temperature);

    DEBUG_PRINT(F("IMU Acc X : ")); DEBUG_PRINT(imuData.acc.x);
    DEBUG_PRINT(F(" Y : ")); DEBUG_PRINT(imuData.acc.y);
    DEBUG_PRINT(F(" Z : ")); DEBUG_PRINTLN(imuData.acc.z);

    DEBUG_PRINT(F("IMU Gyro X : ")); DEBUG_PRINT(imuData.gyro.x);
    DEBUG_PRINT(F(" Y : ")); DEBUG_PRINT(imuData.gyro.y);
    DEBUG_PRINT(F(" Z : ")); DEBUG_PRINTLN(imuData.gyro.z);

    DEBUG_PRINT(F("IMU Mag X : ")); DEBUG_PRINT(imuData.mag.x);
    DEBUG_PRINT(F(" Y : ")); DEBUG_PRINT(imuData.mag.y);
    DEBUG_PRINT(F(" Z : ")); DEBUG_PRINTLN(imuData.mag.z);

    DEBUG_PRINT(F("Heading : ")); DEBUG_PRINTLN(ahrs.angles.heading);
    DEBUG_PRINT(F("Pitch : ")); DEBUG_PRINTLN(ahrs.angles.pitch);
    DEBUG_PRINT(F("Roll : ")); DEBUG_PRINTLN(ahrs.angles.roll);
    DEBUG_PRINT(F("Yaw : ")); DEBUG_PRINTLN(ahrs.angles.yaw);

    DEBUG_PRINTLN(F("------------------------------"));DEBUG_PRINTLN();
}

void Application::kademeAyirmaBilgisayar(){
    rfModule->RFBegin(CONFIG_RF_HIGH_ADDR, CONFIG_RF_LOW_ADDR, CONFIG_RF_CHANNEL, CONFIG_RF_BAUDRATE,
                      CONFIG_RF_AIR_DATA_RATE, CONFIG_RF_RSSI_BYTE, CONFIG_RF_PACKET_SIZE,
                      CONFIG_RF_TRANSFER_MODE, CONFIG_RF_TRANSFER_POWER, CONFIG_RF_WIRELESS_WAKEUP,
                      CONFIG_RF_UART_PARITY, CONFIG_RF_RSSI_AMBIENT, CONFIG_RF_LBT_ENABLE);

    DebugSerial->println(F("Kademe Ayirma Bilgisayar Modu Baslatildi."));

    rfModule->viewSettings();

    uint8_t packet;

    int state = 0;
    bool isPrinted = false;

    uint8_t apogeeSendPacket[3] = {0x12, 0x83, 0x51};
    uint8_t mainSendPacket[3] = {0x37, 0x11, 0x52};

    while(true){
        if(usbPort.available()){
            packet = usbPort.read();

            if(state == 0 && packet == 'k'){
                state = 1;
                isPrinted = false;
            } else if(state == 1 && packet == 'o'){
                state = 2;
                isPrinted = false;
            } else if(state == 2 && (packet == 'a' || packet == 'z')){
                state = 3;
                isPrinted = false;
                if(packet == 'a'){
                    DEBUG_PRINTLN(F("Apogee Atesleme Sinyali Gonderiliyor..."));
                    rfModule->send(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, apogeeSendPacket, sizeof(apogeeSendPacket));
                }
                else if(packet == 'z'){
                    DEBUG_PRINTLN(F("Main Atesleme Sinyali Gonderiliyor..."));
                    rfModule->send(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, mainSendPacket, sizeof(mainSendPacket));
                }
                DEBUG_PRINT(F("Atesleme Sinyali Gönderildi!")); DEBUG_PRINTLN();
            }
            else {
                DEBUG_PRINT(F("Gecersiz Komut!")); DEBUG_PRINTLN();
                isPrinted = false;
                state = 0;
                DEBUG_PRINTLN(F("Sifirlandi!")); DEBUG_PRINTLN();
            }

            usbPort.flush();
        }

        if(isPrinted == false){
            switch (state)
            {
            case 0:
                DEBUG_PRINTLN(F("Patlamak istiyorsan 'k' tuşuna bas!")); DEBUG_PRINTLN();
                isPrinted = true;
                break;
            
            case 1:
                DEBUG_PRINTLN(F("Çok eminsen 'o' tuşuna bas!")); DEBUG_PRINTLN();
                isPrinted = true;
                break;

            case 2:
                DEBUG_PRINTLN(F("Apogee için 'a' main için 'z' tuşuna bas!")); DEBUG_PRINTLN();
                isPrinted = true;
                break;
            }
        }
    }
}

void Application::kademeAyirma(){
    rfModule->RFBegin(RF_SEND_HIGH_ADDR, RF_SEND_LOW_ADDR, CONFIG_RF_CHANNEL, CONFIG_RF_BAUDRATE,
                      CONFIG_RF_AIR_DATA_RATE, CONFIG_RF_RSSI_BYTE, CONFIG_RF_PACKET_SIZE,
                      CONFIG_RF_TRANSFER_MODE, CONFIG_RF_TRANSFER_POWER, CONFIG_RF_WIRELESS_WAKEUP,
                      CONFIG_RF_UART_PARITY, CONFIG_RF_RSSI_AMBIENT, CONFIG_RF_LBT_ENABLE);

    DebugSerial->println(F("Kademe Ayirma Modu Baslatildi."));

    rfModule->viewSettings();

    RF_Msg msg;

    uint8_t apogeeSendPacket[3] = {0x12, 0x83, 0x51};
    uint8_t mainSendPacket[3] = {0x37, 0x11, 0x52};

    pinMode(PYRO_APOGEE_PIN, OUTPUT);
    pinMode(PYRO_MAIN_PIN, OUTPUT);

    while(true){
        msg = rfModule->receive();
        if(E220_Success == msg.status){
            if(msg.buffer[0] == apogeeSendPacket[0] && msg.buffer[1] == apogeeSendPacket[1] && msg.buffer[2] == apogeeSendPacket[2]){
                DEBUG_PRINTLN(F("Apogee Atesleme Sinyali Alindi!"));
                digitalWrite(PYRO_APOGEE_PIN, HIGH);
                delay(500);
                digitalWrite(PYRO_APOGEE_PIN, LOW);
            }
            else if(msg.buffer[0] == mainSendPacket[0] && msg.buffer[1] == mainSendPacket[1] && msg.buffer[2] == mainSendPacket[2]){
                DEBUG_PRINTLN(F("Main Atesleme Sinyali Alindi!"));
                digitalWrite(PYRO_MAIN_PIN, HIGH);
                delay(500);
                digitalWrite(PYRO_MAIN_PIN, LOW);
            }
        }
    }
}

void Application::ykiRun() {
    altitudeManager->initiliaze(CONFIG_BMP_PRESSURE_OVERSAMPLING, CONFIG_BMP_TEMPERATURE_OVERSAMPLING,
                                CONFIG_BMP_IIR_SAMPLING, CONFIG_BMP_ODR_SAMPLING,
                                CONFIG_LPS_ODR_RATE, CONFIG_LPS_LPFP, CONFIG_LPS_BDU, CONFIG_LPS_LOW_NOISE);
    imu->BNOInit();

    gpsModule->begin();

    DebugSerial->println(F("YKI Modu Baslatildi."));

    uint8_t packet[sizeof(BaroData) + sizeof(ImuData) + sizeof(GpsDataSimplified)];

    GpsDataSimplified gpsDataSimplified;

    while(true){
        altitudeManager->update();
        baroData = altitudeManager->getBaroData();
        
        gpsModule->getData();
        gpsData = gpsModule->getDataStruct();
        gpsDataSimplified.latitude = gpsData.latitude;
        gpsDataSimplified.longitude = gpsData.longitude;
        gpsDataSimplified.altitude = gpsData.altitude;

        imuData.acc = imu->getAccData();
        imuData.gyro = imu->getGyroData();
        imuData.mag = imu->getMagData();

        memset(packet, 0, sizeof(packet));
        memcpy(packet, &baroData, sizeof(BaroData));
        memcpy(packet + sizeof(BaroData), &imuData, sizeof(ImuData));
        memcpy(packet + sizeof(BaroData) + sizeof(ImuData), &gpsDataSimplified, sizeof(GpsDataSimplified));

        usbPort.write(packet, sizeof(packet));

        delay(100);
    }
}

void Application::sitSutRun() {
    altitudeManager->initiliaze(CONFIG_BMP_PRESSURE_OVERSAMPLING, CONFIG_BMP_TEMPERATURE_OVERSAMPLING,
                                CONFIG_BMP_IIR_SAMPLING, CONFIG_BMP_ODR_SAMPLING,
                                CONFIG_LPS_ODR_RATE, CONFIG_LPS_LPFP, CONFIG_LPS_BDU, CONFIG_LPS_LOW_NOISE);
    imu->BNOInit();

    gpsModule->begin();

    RS232Serial->begin(115200, SERIAL_8N1);

    DebugSerial->println(F("SIT-SUT Modu Baslatildi."));

    uint8_t packet[sizeof(BaroData) + sizeof(ImuData) + sizeof(GpsDataSimplified)];

    SutSitStates currentState = SUT_SIT_IDLE;

    uint8_t sitPacket[36] = {0};

    uint8_t sutSendPacket[6] = {0};
    uint8_t sutReceivePacket[36] = {0};

    ahrs.begin();
    ahrs.setFusionAlgorithm(SensorFusion::EXTENDED_KALMAN);
    ahrs.setDeclination(6.13f);
    ahrs.setDOF(DOF::DOF_9);

    SensorData tempData;

    BaroData BaroSendData;
    ImuData ImuSendData;
    imuData_float ImuEulerData;

    bool isSutFirst = false;

    bool commandGet = false;

    uint8_t commandSet[4] = {0};

    while(true){
        if(RS232Serial->available() > 0){
            uint8_t receivedByte = RS232Serial->read();
            if(receivedByte == 0xAB){
                // nothing
            }
            else if(receivedByte == 0xAA){
                commandGet = true;

            }

            if(commandGet == true){
                while(RS232Serial->available() < 4);

                RS232Serial->readBytes(commandSet, sizeof(commandSet));
                if(commandSet[3] == 0x0D && commandSet[2] == 0x0A){
                    commandGet = false;
                }
                else{
                    memset(commandSet, 0, sizeof(commandSet));
                    continue;
                }

                switch (commandSet[0])
                {
                case 0x20:
                    if(commandSet[1] == 0x8C)
                        currentState = STATE_SIT;
                    break;
                
                case 0x22:
                    if(commandSet[1] == 0x8E)
                        currentState = STATE_SUT;
                    break;

                case 0x24:
                    if(commandSet[1] == 0x90)
                        currentState = SUT_SIT_IDLE;
                    break;
                }
            }
        }

        if(currentState == STATE_SIT){
            altitudeManager->update();
            baroData = altitudeManager->getBaroData();
            BaroSendData.altitude = roundf(baroData.altitude * 100.0f) / 100.0f;
            BaroSendData.pressure = roundf(baroData.pressure * 100.0f) / 100.0f;
            
            gpsModule->getData();
            gpsData = gpsModule->getDataStruct();

            imuData.acc = imu->getAccData();
            tempData.aTimeStamp = millis();
            imuData.gyro = imu->getGyroData();
            tempData.gTimeStamp = millis();
            imuData.mag = imu->getMagData();
            tempData.mTimeStamp = millis();

            ImuSendData.acc.x = roundf(imuData.acc.x * 100.0f) / 100.0f;
            ImuSendData.acc.y = roundf(imuData.acc.y * 100.0f) / 100.0f;
            ImuSendData.acc.z = roundf(imuData.acc.z * 100.0f) / 100.0f;

            tempData.ax = imuData.acc.x;
            tempData.ay = imuData.acc.y;
            tempData.az = imuData.acc.z;
            tempData.gx = imuData.gyro.x;
            tempData.gy = imuData.gyro.y;
            tempData.gz = imuData.gyro.z;
            tempData.mx = imuData.mag.x;
            tempData.my = imuData.mag.y;
            tempData.mz = imuData.mag.z;

            ahrs.setData(tempData);
            ahrs.update();

            ImuEulerData.x = roundf(ahrs.angles.roll * 100.0f) / 100.0f;
            ImuEulerData.y = roundf(ahrs.angles.pitch * 100.0f) / 100.0f;
            ImuEulerData.z = roundf(ahrs.angles.yaw * 100.0f) / 100.0f;

            sitPacket[0] = 0xAB; // Header
            memcpy(sitPacket + 1, &BaroSendData.altitude, sizeof(float));
            memcpy(sitPacket + 5, &BaroSendData.pressure, sizeof(float));
            memcpy(sitPacket + 9, &ImuSendData.acc.x, sizeof(float));
            memcpy(sitPacket + 13, &ImuSendData.acc.y, sizeof(float));
            memcpy(sitPacket + 17, &ImuSendData.acc.z, sizeof(float));
            memcpy(sitPacket + 21, &ImuEulerData.x, sizeof(float));
            memcpy(sitPacket + 25, &ImuEulerData.y, sizeof(float));
            memcpy(sitPacket + 29, &ImuEulerData.z, sizeof(float));

            int totalCount = 0;
            for(int i = 1; i < 33; i++) {
                totalCount += sitPacket[i];
            }
            totalCount %= 256;
            sitPacket[33] = totalCount;
            sitPacket[34] = 0x0D;
            sitPacket[35] = 0x0A;

            RS232Serial->write(sitPacket, sizeof(sitPacket));

            memset(sitPacket, 0, sizeof(sitPacket));

            delay(100);
        }
        else if(currentState == STATE_SUT){
            while(RS232Serial->available() < 35);

            RS232Serial->readBytes(sutReceivePacket, sizeof(sutReceivePacket) - 1);

            memcpy(&BaroSendData.altitude, &sutReceivePacket[1], sizeof(float));
            memcpy(&BaroSendData.pressure, &sutReceivePacket[5], sizeof(float));
            memcpy(&ImuSendData.acc.x, &sutReceivePacket[9], sizeof(float));
            memcpy(&ImuSendData.acc.y, &sutReceivePacket[13], sizeof(float));
            memcpy(&ImuSendData.acc.z, &sutReceivePacket[17], sizeof(float));
            memcpy(&ImuEulerData.x, &sutReceivePacket[21], sizeof(float));
            memcpy(&ImuEulerData.y, &sutReceivePacket[25], sizeof(float));
            memcpy(&ImuEulerData.z, &sutReceivePacket[29], sizeof(float));

            int totalCount = 0;
            for(int i = 1; i < 33; i++) {
                totalCount = sutReceivePacket[i];
            }
            totalCount %= 256;
            if(totalCount != sutReceivePacket[33] || sutReceivePacket[34] != 0x0D || sutReceivePacket[35] != 0x0A) {
                memset(&sutReceivePacket, 0, sizeof(sutReceivePacket));
                continue;
            }

            if(isSutFirst == false){
                rampAlt = BaroSendData.altitude;
                isSutFirst = true;
            }

            altitudeCircularIn = altitudeCircularIn % 5;
            altitudeCircular[altitudeCircularIn] = BaroSendData.altitude;

            caseCheck();

            sutSendPacket[0] = 0xAA;
            sutSendPacket[1] = sutSelfState;
            sutSendPacket[2] = 0x00;
            int totalCountFor = 0;
            totalCountFor += sutSendPacket[1] + sutSendPacket[2];
            totalCountFor %= 256;
            sutSendPacket[3] = totalCountFor;
            sutSendPacket[4] = 0x0D;
            sutSendPacket[5] = 0x0A;

            RS232Serial->write(sutSendPacket, sizeof(sutSendPacket));

            delay(100);
        }
        else if(currentState == SUT_SIT_IDLE){
            resetCnt();
            isSutFirst = false;
            sutSelfState = 0;
            commandGet = false;
            memset(&BaroSendData, 0, sizeof(BaroData));
            memset(&ImuSendData, 0, sizeof(ImuData));
            memset(&ImuEulerData, 0, sizeof(imuData_float));
        }
    }
}

void Application::resetCnt() {
    altitudeCircularIn = 0;
    rampCnt = 0;
    safeCnt = 0;
    deriCnt = 0;
    accCnt = 0;
    rampAlt = 0.0f;
    rocketState = STATE_RAMP;
    apogeeTime = 0;
    memset(altitudeCircular, 0, sizeof(altitudeCircular));
}