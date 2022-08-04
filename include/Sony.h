#ifndef SONY_H       // Guard to compile only once
#define SONY_H

#include <Arduino.h>    // Always include this. It's important.
#include <WiFi.h>

const String buildJsonConfig(String, String);
void httpPost(const String, int);
boolean connectToWifi(const char*, const char*);
int connectToCameraWifi(void);
int connectCameraAndGetId(String);
int disconnectCamera(void);
String addGuillements(String);
void setStillQuality(String, int);
void setFocusMode(String, int);
void setShutterSpeed(String, int);
void initialConnectionToCamera(int);
void startRecMode(int);
void startLiveView(int);
int stopLiveView(int);
void setIso(String, int);
void takePicture(int);
void startBulb(int);
void stopBulb(int);
void getEvent(int);

#endif