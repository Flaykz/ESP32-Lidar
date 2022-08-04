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
#define WDT_TIMEOUT_SECONDS 20 // Timeout in seconds
#define ONE_SECOND_US 1000000 // Une seconde en us
#define ONE_SECOND_MS 1000 // Une seconde en ms
#define OBTURATION_TIME_US 20*ONE_SECOND_US // in us
#define FLASH_REACTIVATION_TIME_MS 25*60*ONE_SECOND_MS  // 25min in millisecond
#define MIN_TIME_BETWEEN_FLASH_MS 1*ONE_SECOND_MS // min time in ms between 2 flash

// params BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_ISO_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TRIGGER_COUNT_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHARACTERISTIC_LOOPS_COUNT_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"

TFMPlus lidar1;
TFMPlus lidar2;

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicIso = NULL;
BLECharacteristic* pCharacteristicTriggerCount = NULL;
BLECharacteristic* pCharacteristicLoopsCount = NULL;

long loopCount = 0; // count nb of loop in one second
long currentMicros = 0;
long lastLoopMicros = 0;

long currentMillis = 0;
long lastFlashTime = 0; // Si > 25min, on lance flash pour eviter mise en veille
int cameraId = 0; // 0 = declenchement timer (pas connecté au wifi), > 0 declenchement bulb (connecté à camera 1 ou 2)
String defaultIso = "200"; // default iso
boolean handleCameraBulb = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile int counter;
volatile int enableTrigger = 0;
volatile int triggerCount = 0;

// Initialize variables
int16_t distance1, lastDistance1, distance2, lastDistance2 = 0;    // Distance to object in centimeters
int16_t signalQuality1, lastSignalQuality1, signalQuality2, lastSignalQuality2 = 0;    // Strength or quality of return signal
int16_t temperature1, temperature2 = 0;    // Internal temperature of Lidar sensor chip
int16_t minDistance, maxDistance, minQuality, maxQuality, diffDistance, diffQuality; // For lidar calibration

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

class MyTriggerCountCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string characteristicValue = pCharacteristic->getValue();
      
      if (characteristicValue.length() > 0) {
        String value = "";
        for (int i = 0; i < characteristicValue.length(); i++) {
          value = value + characteristicValue[i];          
        }
        Serial.print("TriggerCount = ");
        Serial.println(value);
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

void setMinMaxLidarData() {
    minDistance = distance1;
    maxDistance = distance1;
    minQuality = signalQuality1;
    maxQuality = signalQuality1;
}

void refreshMinMaxLidarData() {
    minDistance = min(distance1, minDistance);
    minDistance = min(distance2, minDistance);
    minQuality = min(signalQuality1, minQuality);
    minQuality = min(signalQuality2, minQuality);
    maxDistance = max(distance1, maxDistance);
    maxDistance = max(distance2, maxDistance);
    maxQuality = max(signalQuality1, maxQuality);
    maxQuality = max(signalQuality2, maxQuality);
}

void refreshLidarData() {
    lastDistance1 = distance1;
    lastDistance2 = distance2;
    lastSignalQuality1 = signalQuality1;
    lastSignalQuality2 = signalQuality2;

    lidar1.getData(distance1, signalQuality1, temperature1);
    lidar2.getData(distance2, signalQuality2, temperature2);
}

void calibrateLidar() {
    Serial.println("Etalonnage du lidar pendant 10 sec...");
    currentMillis = millis();
    refreshLidarData();
    setMinMaxLidarData();
    while (millis() - currentMillis <= 2000) { //Permet de stabiliser les valeurs mesurées
        refreshLidarData();
    }
    while (millis() - currentMillis <= 10000) {
        refreshLidarData();
        refreshMinMaxLidarData();
    }
    diffDistance = maxDistance - minDistance;
    diffQuality = maxQuality - minQuality;
    Serial.println("Etalonnage du lidar terminé");
    Serial.print("Distance min/max: ");
    Serial.print(minDistance);
    Serial.print("/");
    Serial.println(maxDistance);
    Serial.print("Quality min/max: ");
    Serial.print(minQuality);
    Serial.print("/");
    Serial.println(maxQuality);

    minDistance -= (diffDistance) + 1;
    minQuality -= (diffQuality) + 1;
    maxQuality += (diffQuality) + 1;

    Serial.print("Seuil déclenchement distance min: ");
    Serial.println(minDistance);
    Serial.print("Seuil déclenchement quality min/max: ");
    Serial.print(minQuality);
    Serial.print("/");
    Serial.println(maxQuality);
    if (minDistance < 0) {
        Serial.println("Probleme lors de l'étalonnage, redémarrage de celui-ci");
        calibrateLidar();
    }
}

void initLidar() {
    lidar1.init(LIDAR_1, true, RX_D1, TX_D1);
    lidar2.init(LIDAR_2, true, RX_D2, TX_D2);

    calibrateLidar();
}

void initFlash() {
    // Initilise le flash sur éteind
    pinMode(FLASH, OUTPUT);
    digitalWrite(FLASH, LOW);

    delay(500);
}

boolean isDistanceChange() {
    return distance1 <= minDistance or distance2 <= minDistance;
    //or signalQuality1 >= maxQuality or signalQuality1 <= minQuality or signalQuality2 >= maxQuality or signalQuality2 <= minQuality;
}

void initBle() {
    BLEDevice::init("MyESP32Lidar");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
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
    // https://www.bluetooth.com/specifications/gatt/viewer?
    //   attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    pCharacteristicIso->addDescriptor(new BLE2902());
    pCharacteristicIso->setCallbacks(new MyIsoCallbacks());
    pCharacteristicIso->setValue(defaultIso.c_str());

    pCharacteristicTriggerCount->addDescriptor(new BLE2902());
    pCharacteristicTriggerCount->setCallbacks(new MyTriggerCountCallbacks());
    pCharacteristicTriggerCount->setValue("0");

    pCharacteristicLoopsCount->addDescriptor(new BLE2902());
    pCharacteristicLoopsCount->setCallbacks(new MyTriggerCountCallbacks());
    pCharacteristicLoopsCount->setValue("0");

    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
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

    if (handleCameraBulb and enableTrigger > 0) // Temps exposition atteint
    {
        resetBulb();
        enableTrigger = 0;
    }
    //showDistance();
    refreshLidarData(); // get latest data for lidar 1 and 2

    if (isDistanceChange() and (currentMillis - lastFlashTime >= MIN_TIME_BETWEEN_FLASH_MS)) {
        triggerCameraFlash();
        showDistance();
    }
	
	if (currentMillis - lastFlashTime > FLASH_REACTIVATION_TIME_MS) {
        triggerCameraFlash();
	}

    if (ACTIVE_BLUETOOTH) {
        if (micros() - lastLoopMicros > 1000000) {
            // Serial.print("number of loop in one second: ");
            // Serial.println(loopCount);
            pCharacteristicLoopsCount->setValue(String(loopCount).c_str());
            pCharacteristicLoopsCount->notify();
            lastLoopMicros = micros();
            loopCount = 0;
        }
    }
}