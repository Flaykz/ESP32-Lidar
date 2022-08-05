#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <TFMPlus.cpp>  // Include TFMini Plus Library v1.4.2
#include <HardwareSerial.h>
#include <TFT_eSPI.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <Sony.cpp>

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

// pin config
#define RX_D1 26
#define TX_D1 27
#define RX_D2 33
#define TX_D2 25
#define FLASH 12
#define SENSOR_NUMBER 2
#define BUTTON_1 35
#define BUTTON_2  0     
#define LASER 13
#define LIDAR_1 1
#define LIDAR_2 2

// params
#define ACTIVE_BLUETOOTH true
#define BAUD_RATE 115200
#define FONT_H tft.fontHeight()
#define FONT_W tft.fontHeight()
#define WDT_TIMEOUT_SECONDS 20 // Timeout in seconds for watchdog
#define ONE_SECOND_US 1000000 // Une seconde en us
#define ONE_SECOND_MS 1000 // Une seconde en ms
#define OBTURATION_TIME_US 20*ONE_SECOND_US // in us
#define FLASH_REACTIVATION_TIME_MS 25*60*ONE_SECOND_MS  // 25min in millisecond
#define MIN_TIME_BETWEEN_FLASH_MS 1*ONE_SECOND_MS // min time in ms between 2 flash

// BLE service principal
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c3319140"
#define CHARACTERISTIC_ISO_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319141"
#define CHARACTERISTIC_TRIGGER_COUNT_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319142"
#define CHARACTERISTIC_LOOPS_COUNT_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319143"
#define CHARACTERISTIC_IS_ON_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319144"

// BLE service calibration
#define SERVICE_CALIBRATION_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319130"
#define CHARACTERISTIC_CALIBRATION_TIME_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319131"
#define CHARACTERISTIC_CALIBRATION_A_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319132"
#define CHARACTERISTIC_CALIBRATION_B_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319133"
#define CHARACTERISTIC_CALIBRATION_RESTART_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319134"

// BLE service notify
#define SERVICE_NOTIFY_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319120"
#define CHARACTERISTIC_CALIBRATION_DELTA_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319121"

TFMPlus lidar1;
TFMPlus lidar2;

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicIso = NULL;
BLECharacteristic* pCharacteristicTriggerCount = NULL;
BLECharacteristic* pCharacteristicLoopsCount = NULL;
BLECharacteristic* pCharacteristicIsOn = NULL;
BLECharacteristic* pCharacteristicCalibrationTime = NULL;
BLECharacteristic* pCharacteristicCalibrationA = NULL;
BLECharacteristic* pCharacteristicCalibrationB = NULL;
BLECharacteristic* pCharacteristicCalibrationRestart = NULL;
BLECharacteristic* pCharacteristicCalibrationDelta = NULL;


long loopCount = 0; // count nb of loop in one second
long currentMicros = 0;
long lastLoopMicros = 0;

long currentMillis = 0;
long lastFlashTime = 0; // Si > 25min, on lance flash pour eviter mise en veille
int cameraId = 0; // 0 = declenchement timer (pas connecté au wifi), > 0 declenchement bulb (connecté à camera 1 ou 2)

String defaultIso = "200"; // default iso
uint16_t defaultLidarCalibrationTimeMs = 10000;
uint8_t defaultLidarCalibrationA = 1; // a in ax + b
uint8_t defaultLidarCalibrationB = 1; // b in ax + b
boolean shouldRecalibrateLidar = false;
boolean handleCameraBulb = false;
boolean isOn = true;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int counter;
volatile int enableTrigger = 0;
volatile int triggerCount = 0;

// Initialize variables
int16_t distance1, lastDistance1, distance2, lastDistance2 = 0;    // Distance to object in centimeters
int16_t signalQuality1, lastSignalQuality1, signalQuality2, lastSignalQuality2 = 0;    // Strength or quality of return signal
int16_t temperature1, temperature2 = 0;    // Internal temperature of Lidar sensor chip
int16_t minDistance1, maxDistance1, diffDistance1, minDistance2, maxDistance2, diffDistance2 = 0; // For lidar calibration

String getCharacteristicStringValue(BLECharacteristic *pCharacteristic) {
    std::string characteristicValue = pCharacteristic->getValue();
      String value = "";
      if (characteristicValue.length() > 0) {
        for (int i = 0; i < characteristicValue.length(); i++) {
          value = value + characteristicValue[i];          
        }
        Serial.println(value);
     }
     return value;
}

void setMinMaxLidarData() {
    minDistance1 = distance1;
    maxDistance1 = distance1;
    minDistance2 = distance2;
    maxDistance2 = distance2;
}

void refreshMinMaxLidarData() {
    minDistance1 = min(distance1, minDistance1);
    minDistance2 = min(distance2, minDistance2);
    maxDistance1 = max(distance1, maxDistance1);
    maxDistance2 = max(distance2, maxDistance2);
}

void refreshLidarData() {
    lastDistance1 = distance1;
    lastDistance2 = distance2;
    lastSignalQuality1 = signalQuality1;
    lastSignalQuality2 = signalQuality2;

    lidar1.getData(distance1, signalQuality1, temperature1);
    lidar2.getData(distance2, signalQuality2, temperature2);
}

String getLidarsDelta(int16_t delta1, int16_t delta2) {
    return String(delta1) + ";" + String(delta2);
}

void calibrateLidar() {
    currentMillis = millis();
    Serial.println("Etalonnage du lidar pendant 10 sec...");
    refreshLidarData();
    setMinMaxLidarData();
    while (millis() - currentMillis <= defaultLidarCalibrationTimeMs) {
        refreshLidarData();
        refreshMinMaxLidarData();
        esp_task_wdt_reset();
    }
    diffDistance1 = maxDistance1 - minDistance1;
    diffDistance2 = maxDistance2 - minDistance2;

    Serial.println("Etalonnage du lidar 1 terminé");
    Serial.print("Distance min/max: ");
    Serial.print(minDistance1);
    Serial.print("/");
    Serial.println(maxDistance1);
    Serial.println("Etalonnage du lidar 2 terminé");
    Serial.print("Distance min/max: ");
    Serial.print(minDistance2);
    Serial.print("/");
    Serial.println(maxDistance2);

    minDistance1 -= (diffDistance1 * defaultLidarCalibrationA) + defaultLidarCalibrationB;
    minDistance2 -= (diffDistance2 * defaultLidarCalibrationA) + defaultLidarCalibrationB;

    Serial.print("Seuil déclenchement lidar 1 distance min: ");
    Serial.println(minDistance1);
    Serial.print("Seuil déclenchement lidar 2 distance min: ");
    Serial.println(minDistance2);
    if (minDistance1 < 0 or minDistance2 < 0) {
        Serial.println("Probleme lors de l'étalonnage, redémarrage de celui-ci");
        calibrateLidar();
    }
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("connect");
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("disconnect");
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
    }
};

class MyIsoCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string characteristicValue = pCharacteristic->getValue();
      
      if (characteristicValue.length() > 0) {
        String value = "";
        for (int i = 0; i < characteristicValue.length(); i++) {
          value = value + characteristicValue[i];          
        }

        if (handleCameraBulb) {
            Serial.print("Set ISO to ");
            Serial.println(value);
            setIso(value, cameraId);
        }
     }
    }
};

class MyStringCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      getCharacteristicStringValue(pCharacteristic);
    }
};

class MyLidarCalibrationTimeCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = getCharacteristicStringValue(pCharacteristic);
      defaultLidarCalibrationTimeMs = value.toInt();
    }
};

class MyLidarCalibrationACallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = getCharacteristicStringValue(pCharacteristic);
      defaultLidarCalibrationA = value.toInt();
    }
};

class MyLidarCalibrationBCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = getCharacteristicStringValue(pCharacteristic);
      defaultLidarCalibrationB = value.toInt();
    }
};

class MyLidarCalibrationRestartCallback: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
        shouldRecalibrateLidar = true;
    }
};

class MyIsOnCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t *characteristicValue = pCharacteristic->getData();
      uint8_t value = characteristicValue[0] - '0';
      Serial.println(*characteristicValue, HEX);
      if (value == 1) {
        isOn = true;
        Serial.println("Activation du système");
      } else {
        isOn = false;
        Serial.println("Désactivation du système");
      }
    }
};

void IRAM_ATTR onTime() {
    enableTrigger++;
    timerAlarmDisable(timer);
    printf( "timer restarted\r\n");
    //tft.drawString("           ", FONT_W*3, FONT_H*4);
}

void showDistance() {  
    Serial.print("Lidar 1 (distance/quality): ");
    Serial.print(distance1);
    Serial.print("/");
    Serial.print(signalQuality1);
    Serial.print(", previous: ");
    Serial.print(lastDistance1);
    Serial.print("/");
    Serial.println(lastSignalQuality1);
    Serial.print("Lidar 2 (distance/quality): ");
    Serial.print(distance2);
    Serial.print("/");
    Serial.print(signalQuality2);
    Serial.print(", previous: ");
    Serial.print(lastDistance2);
    Serial.print("/");
    Serial.println(lastSignalQuality2);
    Serial.println();
} 

void resetBulb() {
    stopBulb(cameraId);
    getEvent(cameraId);
    startBulb(cameraId);
    timerAlarmEnable(timer);
    timerRestart(timer);
}

void triggerCameraFlash() {
    digitalWrite(FLASH, HIGH);
    delayMicroseconds(100);                  
    digitalWrite(FLASH, LOW);

    if (handleCameraBulb) {
        resetBulb();
        enableTrigger = 0;
    }

    triggerCount++;

    if (ACTIVE_BLUETOOTH) {
        pCharacteristicTriggerCount->setValue(String(triggerCount).c_str());
        pCharacteristicTriggerCount->notify();
    }

    printf("Declenchement n° %u\r\n", triggerCount);
	lastFlashTime = millis();
}

void showLcdInit() {
    tft.begin();
    tft.setRotation(3);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
    tft.drawString("ESP32 Lidar Project", tft.width() / 2, tft.height() / 2);
    tft.setTextSize(3);
}

void showLcdDistances() {
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Dist1 ", FONT_W*3, FONT_H*1);
    tft.drawString("Dist2 ", FONT_W*3, FONT_H*2);
    tft.drawString("SRate ", FONT_W*3, FONT_H*3);
}

void initSerialTerminal() {
    Serial.begin(BAUD_RATE);   // Intialize terminal serial port
    delay(20);               // Give port time to initalize
}

void initLidar() {
    lidar1.init(LIDAR_1, true, RX_D1, TX_D1);
    lidar2.init(LIDAR_2, true, RX_D2, TX_D2);

    currentMillis = millis();
    while (millis() - currentMillis <= 2000) { //Permet de stabiliser les valeurs mesurées
        refreshLidarData();
    }

    calibrateLidar();
}

void initFlash() {
    // Initilise le flash sur éteind
    pinMode(FLASH, OUTPUT);
    digitalWrite(FLASH, LOW);

    delay(500);
}

boolean isDistanceChange() {
    return distance1 <= minDistance1 or distance2 <= minDistance2;
}

void initBle() {
    BLEDevice::init("MyESP32Lidar");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLEService *pServiceCalibration = pServer->createService(SERVICE_CALIBRATION_UUID);
    BLEService *pServiceNotify = pServer->createService(SERVICE_NOTIFY_UUID);
    // Create a BLE Characteristic
    pCharacteristicIso = pService->createCharacteristic(
                                            CHARACTERISTIC_ISO_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
                                            );
    pCharacteristicTriggerCount = pService->createCharacteristic(
                                            CHARACTERISTIC_TRIGGER_COUNT_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
                                            );
    pCharacteristicLoopsCount = pService->createCharacteristic(
                                            CHARACTERISTIC_LOOPS_COUNT_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
                                            );
    pCharacteristicIsOn = pService->createCharacteristic(
                                            CHARACTERISTIC_IS_ON_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE
                                            );
    pCharacteristicCalibrationTime = pServiceCalibration->createCharacteristic(
                                            CHARACTERISTIC_CALIBRATION_TIME_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE
                                            );
    pCharacteristicCalibrationA = pServiceCalibration->createCharacteristic(
                                            CHARACTERISTIC_CALIBRATION_A_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE
                                            );
    pCharacteristicCalibrationB = pServiceCalibration->createCharacteristic(
                                            CHARACTERISTIC_CALIBRATION_B_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE
                                            );
    pCharacteristicCalibrationRestart = pServiceCalibration->createCharacteristic(
                                            CHARACTERISTIC_CALIBRATION_RESTART_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE
                                            );
    pCharacteristicCalibrationDelta = pServiceNotify->createCharacteristic(
                                            CHARACTERISTIC_CALIBRATION_DELTA_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_NOTIFY
                                            );
    // https://www.bluetooth.com/specifications/gatt/viewer?
    //   attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    pCharacteristicIso->addDescriptor(new BLE2902());
    pCharacteristicIso->setCallbacks(new MyIsoCallbacks());
    pCharacteristicIso->setValue(defaultIso.c_str());

    pCharacteristicTriggerCount->addDescriptor(new BLE2902());
    pCharacteristicTriggerCount->setNotifyProperty(true);
    pCharacteristicTriggerCount->setCallbacks(new MyStringCallback());
    pCharacteristicTriggerCount->setValue("0");

    pCharacteristicLoopsCount->addDescriptor(new BLE2902());
    pCharacteristicLoopsCount->setNotifyProperty(true);
    pCharacteristicLoopsCount->setCallbacks(new MyStringCallback());
    pCharacteristicLoopsCount->setValue("0");

    pCharacteristicIsOn->addDescriptor(new BLE2902());
    pCharacteristicIsOn->setCallbacks(new MyIsOnCallbacks());
    pCharacteristicIsOn->setValue(isOn ? "1" : "0");

    pCharacteristicCalibrationTime->addDescriptor(new BLE2902());
    pCharacteristicCalibrationTime->setCallbacks(new MyLidarCalibrationTimeCallback());
    pCharacteristicCalibrationTime->setValue(String(defaultLidarCalibrationTimeMs).c_str());

    pCharacteristicCalibrationA->addDescriptor(new BLE2902());
    pCharacteristicCalibrationA->setCallbacks(new MyLidarCalibrationACallback());
    pCharacteristicCalibrationA->setValue(String(defaultLidarCalibrationA).c_str());

    pCharacteristicCalibrationB->addDescriptor(new BLE2902());
    pCharacteristicCalibrationB->setCallbacks(new MyLidarCalibrationBCallback());
    pCharacteristicCalibrationB->setValue(String(defaultLidarCalibrationB).c_str());

    pCharacteristicCalibrationRestart->addDescriptor(new BLE2902());
    pCharacteristicCalibrationRestart->setCallbacks(new MyLidarCalibrationRestartCallback());
    pCharacteristicCalibrationRestart->setValue("0");

    pCharacteristicCalibrationDelta->addDescriptor(new BLE2902());
    pCharacteristicCalibrationDelta->setNotifyProperty(true);
    pCharacteristicCalibrationDelta->setCallbacks(new MyStringCallback());
    pCharacteristicCalibrationDelta->setValue(getLidarsDelta(0, 0).c_str());

    pService->start();
    pServiceCalibration->start();
    pServiceNotify->start();

    // Start advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->addServiceUUID(SERVICE_CALIBRATION_UUID);
    pAdvertising->addServiceUUID(SERVICE_NOTIFY_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");
}

void setup()
{
    initSerialTerminal();

    if (ACTIVE_BLUETOOTH) {
        // Create the BLE Server
        initBle();
    }

    cameraId = connectCameraAndGetId(defaultIso);
    if (cameraId > 0) {
        handleCameraBulb = true;
    }

    initLidar();

    initFlash();


    // Watchdog
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch

    if (handleCameraBulb) {
        timer = timerBegin(0, 80, true); //80MHz / 80 => 1us     
        timerAttachInterrupt(timer, &onTime, true);
        timerAlarmWrite(timer, OBTURATION_TIME_US, true);
    } else {
        // timerAlarmWrite(timer, ONE_SECOND_US, true); //tempo lors de l'allumage
        delay(1000); //tempo lors de l'allumage
        triggerCameraFlash(); // double flash pour signaler le mode sans gestion de la camera
        delay(500);
        triggerCameraFlash();  
    }
	
	lastFlashTime = millis();
    lastLoopMicros = micros();
}

void loop()
{
    loopCount++;
    currentMillis = millis();
    currentMicros = micros();
    esp_task_wdt_reset();  //reset watchdog

    if (shouldRecalibrateLidar) {
        calibrateLidar();
        shouldRecalibrateLidar = false;
        pCharacteristicCalibrationDelta->setValue(getLidarsDelta(diffDistance1, diffDistance2).c_str());
        pCharacteristicCalibrationDelta->notify();
    }

    if (handleCameraBulb and enableTrigger > 0 and isOn) // Temps exposition atteint
    {
        resetBulb();
        enableTrigger = 0;
    }

    if (isOn) {
        refreshLidarData(); // get latest data for lidar 1 and 2
    }

    if (isDistanceChange() and (currentMillis - lastFlashTime >= MIN_TIME_BETWEEN_FLASH_MS) and isOn) {
        triggerCameraFlash();
        showDistance();
    }
	
	if (currentMillis - lastFlashTime > FLASH_REACTIVATION_TIME_MS) {
        triggerCameraFlash();
	}

    if (micros() - lastLoopMicros > ONE_SECOND_US) {
        // Serial.print("number of loop in one second: ");
        // Serial.println(loopCount);
        if (ACTIVE_BLUETOOTH) {
            pCharacteristicLoopsCount->setValue(String(loopCount).c_str());
            pCharacteristicLoopsCount->notify();
        }
        lastLoopMicros = micros();
        loopCount = 0;
    }
}