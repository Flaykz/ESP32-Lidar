#include <Arduino.h>
#include <WiFi.h>
#include <Sony.h>
#include <string.h>

#define Camera_A7xx 1
#define Camera_A6xxx 2
#define WIFI_TIMEOUT 8000 // Timeout pour réponse de l'appareil photo en wifi

const char* ssid     = "DIRECT-CeE0:ILCE-7RM2";
const char* ssid2     = "DIRECT-mgE0:ILCE-6300";
const char* password = "9E8EqQDV";     // your WPA2 password. Get it on Sony camera (connect with password procedure)
const char* password2 = "qXb1X35h";

String sonyUrl = "/sony/camera";

const char* host = "192.168.122.1";   // fixed IP of camera
const int httpPort = 8080;

boolean isBulbActive = false;

WiFiClient wifiClient;

const String buildJsonConfig(String paramName, String params) {
  String json = "{\"version\":\"1.0\",\"id\":1,\"method\":\"" + paramName + "\",\"params\":[" + params + "]}";
 return json;
}

void httpPost(const String jString, int cameraId) {
  if (cameraId == 0 || !wifiClient.connect(host, httpPort)) {
    return;
  }

  String request;
  if (cameraId == Camera_A7xx) 
    {request = String("POST " + sonyUrl + " HTTP/1.1\r\nHost: " + host + "\r\n");}
  else
    {request = String("POST " + sonyUrl + " HTTP/1.1\r\n"); ///A6300
  } 


  // Headers
  wifiClient.print(request);
  wifiClient.println("Content-Type: application/json");
  wifiClient.print("Content-Length: ");
  wifiClient.println(jString.length());
  // End of headers
  wifiClient.println();
  // Request body
  wifiClient.println(jString);
  // End of body

  unsigned long lastmillis;
  lastmillis = millis();

  // if (jString.indexOf("startBulbShooting") <= 0) {
  // }
  while (!wifiClient.available() && millis() - lastmillis < WIFI_TIMEOUT) {} // wait 8s max for answer
 
  // // Read all the lines of the reply from server and print them to Serial
  while (wifiClient.available()) {
    String line = wifiClient.readStringUntil('\r');
    // Serial.println(line);
  }

  wifiClient.stop();
}

boolean connectToWifi(const char* ssid, const char* password) {
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  int n = 0;
  while (WiFi.status() != WL_CONNECTED and n <= 3) {   // wait for WiFi connection
    delay(1000);
    n = n + 1;
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

int connectToCameraWifi() {
  Serial.println("Tentative de connexion sur Camera_A7xx");
  if (connectToWifi(ssid, password)) {
    return Camera_A7xx;
  }
  Serial.println("Tentative de connexion sur Camera_A6xxx");
  if (connectToWifi(ssid2, password2)) {
    return Camera_A6xxx;
  }

  Serial.println("No connection to camera");
  return 0;
}

int connectCameraAndGetId(String defaultIso)
{
  int cameraId = connectToCameraWifi();
  if (cameraId == 0) {
    return 0;
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("1");
  initialConnectionToCamera(cameraId);
  Serial.println("2");
  startRecMode(cameraId);
  Serial.println("3");
  startLiveView(cameraId);
  Serial.println("4");
  setIso(defaultIso, cameraId);
  Serial.println("5");
  setShutterSpeed("BULB", cameraId); // or "1/160"
  Serial.println("6");
  setFocusMode("MF", cameraId); //set MF
  setStillQuality("{\"stillQuality\": \"RAW+JPEG\"}", cameraId); //set setStillQuality
 
  Serial.print("CamId : ");
  Serial.println(cameraId);
  return cameraId;
}

int disconnectCamera()
{
  Serial.println(WiFi.status());
  WiFi.disconnect();
  return 1;
}

String addGuillements(String value) {
  String str = "\"" + value;
  return str + "\"";
}

void setStillQuality(String stillQuality, int cameraId) {
  httpPost(buildJsonConfig("setStillQuality", stillQuality), cameraId);
}

void setFocusMode(String focusMode, int cameraId) {
  httpPost(buildJsonConfig("setFocusMode", addGuillements(focusMode)), cameraId);
}

void setShutterSpeed(String shutterSpeed, int cameraId) {
  httpPost(buildJsonConfig("setShutterSpeed", addGuillements(shutterSpeed)), cameraId);
}

void initialConnectionToCamera(int cameraId) {
  httpPost(buildJsonConfig("getVersions", ""), cameraId);
}

void startRecMode(int cameraId) {
  httpPost(buildJsonConfig("startRecMode", ""), cameraId);
}

void startLiveView(int cameraId)
{
  httpPost(buildJsonConfig("startLiveview", ""), cameraId);
}

int stopLiveView(int cameraId)
{
  httpPost(buildJsonConfig("stopLiveview", ""), cameraId);
  return 1;
}

void setIso(String iso, int cameraId) {
  httpPost(buildJsonConfig("setIsoSpeedRate", addGuillements(iso)), cameraId);
}

void takePicture(int cameraId)
{
  httpPost(buildJsonConfig("actTakePicture", ""), cameraId);
}

void startBulb(int cameraId)
{
  if (!isBulbActive) {
    Serial.println("startBulb");
    httpPost(buildJsonConfig("startBulbShooting", ""), cameraId);
    Serial.println("startBulb ended");
  }
  isBulbActive = true;
}

void stopBulb(int cameraId)
{
  if (isBulbActive) {
    Serial.println("stopBulb");
    httpPost(buildJsonConfig("stopBulbShooting", ""), cameraId);
  }
  isBulbActive = false;
}

void getEvent(int cameraId)
{
  Serial.println("getEvent");
  httpPost(buildJsonConfig("getEvent", "true"), cameraId);
}
