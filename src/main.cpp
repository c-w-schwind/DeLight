#include <Arduino.h>
#include <BLEDevice.h>
#include <WiFi.h> 
#include <HTTPClient.h>
#include <movingAvg.h>
#include "mySecrets.h"
#include <ArduinoJson.h>
#include <time.h>

//WIFI
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;

//API
const char* serverPathLatLon = "https://api.openweathermap.org/data/2.5/weather?lat=50.9375&lon=6.9603&appid=5a246f369afff4ce3d85a772dfe8e031";
const char* serverPathZipCode = "https://api.openweathermap.org/data/2.5/weather?zip=51103,de&appid=5a246f369afff4ce3d85a772dfe8e031";
unsigned long lastAPICall;

//BLE
static BLEUUID serviceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59"); 
static BLEUUID charUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");   
static String Partner_BLE_Address = "b4:e6:2d:eb:17:73";                       
static BLERemoteCharacteristic *pRemoteCharacteristic;
BLEClient *pClient;
BLEScan *pBLEScan;
BLEScanResults foundDevices;
String Scanned_BLE_Address;

//RSSI
int rssi;
movingAvg rssiAvg(10);

//WEATHER
StaticJsonDocument<1024> doc;
int cloudiness;
long sunrise, sunset;
long timeToSunrise = 86400, timeToSunset = 86400;

//LAMP
bool isLampOn = false;

//TIME
const char* ntpServer = "pool.ntp.org";


unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 60000U)) {    // trying for max 60 seconds
    Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

bool connectToServer(BLEAddress pAddress) {
  pClient->connect(pAddress);
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService != nullptr) {
    Serial.println("Connected to server");
    return true;
  } else {
    Serial.println("Connection to server failed");
    return false;
  }
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Scan Result: %s \n", advertisedDevice.toString().c_str());
  }
};

void initBLEScan() {
  BLEDevice::init("");
  Serial.println("ESP32 BLE Server program. Scanning for Bluetooth devices...");
  pBLEScan = BLEDevice::getScan();                                            // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());  // Call the class that is defined above
  pBLEScan->setActiveScan(true);                                              // active scan uses more power, but get results faster
  //pBLEScan->setInterval(scanInterval);                                      // in msec
  //pBLEScan->setWindow(scanWindow);                                          // in msec
  //pBLEScan->start(scanTime);                                                // in sec
}

bool connectWifi() {
  int counter = 0;
  WiFi.begin(ssid, password);
  Serial.print("Attempting to connect to WiFi with following SSID: ");
  Serial.println(ssid);
  while (counter < 40) {  //attempting connection ends after 20 seconds (40 * 500ms)
    if (WiFi.status() != WL_CONNECTED) {
      delay (500);
      Serial.print(".");
    } else {
      Serial.print("Connected to WiFi network with IP Address: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    ++counter;
  }
  Serial.print("\nCould not connect to WiFi network: ");
  Serial.println(ssid);
  return false;
}

String getRequestWeatherAPI() {
  HTTPClient http;
  http.begin(serverPathZipCode);
  int httpResponseCode = http.GET();
  String payload = "{}";
  if (httpResponseCode > 0) {
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    payload = http.getString();
  } else {
    Serial.println("Error code: " + String(httpResponseCode));
  }
  http.end();
  return payload;
}

void parseWeatherData() {
  DeserializationError error = deserializeJson(doc, getRequestWeatherAPI());
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  cloudiness = doc["clouds"]["all"];
  sunrise = doc["sys"]["sunrise"];    // Sunrise in UTC, add timezone for local time
  sunset = doc["sys"]["sunset"];      // TODO: nur einmal täglich abrufen...
}

void turnLampOff() {
  digitalWrite(14, LOW);
  digitalWrite(26, LOW);
  digitalWrite(33, LOW);
  isLampOn = false;
  Serial.println("\nLAMP OFF\n");
}

void updateValues() {
  long time = getTime();              // failure of getTime() returns 0
  if (time != 0) {                    // only change when time was obtained
    timeToSunset = sunset - time;
    timeToSunrise = sunrise - time;
  }
}

void turnLampOn() {                   // TODO: default lamp settings. only when settings not updated for a while
  digitalWrite(14, HIGH);   
  digitalWrite(26, HIGH);
  digitalWrite(33, HIGH);
  isLampOn = true;
  Serial.println("\nLAMP ON\n");
}

void updateLamp() {                   // TODO: input variables
  updateValues();
  if (timeToSunset != 86400 && timeToSunset >= -3600) {
    
  }
  if (timeToSunrise != 86400 && timeToSunrise >= -3600) {
    
  }
  Serial.print("\nRight now in ");
  Serial.printf(doc["name"]);
  Serial.print("   Cloudiness: ");
  Serial.println(cloudiness);
  Serial.print("   Sunrise: ");
  Serial.println(sunrise);
  Serial.print("   Sunset: ");
  Serial.println(sunset);
  Serial.print("   Time to sunset: ");
  Serial.println(timeToSunset);
  Serial.print("   Time to sunrise: ");
  Serial.println(timeToSunrise);
}

void updateRssiWithDelay() {
  rssi = -10;//pClient->getRssi();
  Serial.printf("RSSI: %d       ", rssi);
  Serial.printf("Moving Average: %d       ", rssiAvg.reading(rssi));
  Serial.printf("Average input count: %d\n", rssiAvg.getCount());
  delay(500);
}

void setup() {
  Serial.begin(115200);
  initBLEScan();
  pClient = BLEDevice::createClient();
  rssiAvg.begin();
  pinMode(14, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(33, OUTPUT);
  configTime(0, 0, ntpServer);
}


void loop() {
  /*
  while (!pClient->isConnected()) {
    foundDevices = pBLEScan->start(3); // Scan for 3 seconds
    Serial.printf("\nScan done.\nNumber of BLE devices found: %d\n\n\n)", foundDevices.getCount());

    for (int i = 0; i < foundDevices.getCount(); i++) {
      Scanned_BLE_Address = foundDevices.getDevice(i).getAddress().toString().c_str();
      if (Scanned_BLE_Address == Partner_BLE_Address) {
        Serial.println("Found partner device. Connecting to server as client");
        connectToServer(foundDevices.getDevice(i).getAddress()) ? Serial.println("Paired with partner device") : Serial.println("Pairing failed");
      }
    }
  }

  while (pClient->isConnected()) {
  */
  while(true) {
    updateRssiWithDelay();
    if (rssiAvg.getAvg() < -80) {
      if (isLampOn) turnLampOff();
    } else {
      if (!isLampOn) {
        turnLampOn();         // TODO: only default if for a while no updated settings yet? 
        lastAPICall = millis() - /*10 **/ 60000UL;
        Serial.println(millis());
        Serial.println(millis() - /*10 **/ 60000UL);
      }
      if (millis() - lastAPICall >= /*10 **/ 60000UL) {       // every 10 Minutes? 20?     TODO: remove -> /**/
        if(WiFi.status() != WL_CONNECTED) connectWifi();      // TODO: disconnect? reasonable with > 10 minutes
        parseWeatherData();
        lastAPICall = millis();
        updateLamp();
      }
    }
  }

  Serial.println("\nDisconnected\n");
  WiFi.disconnect();
  rssiAvg.reset();
}