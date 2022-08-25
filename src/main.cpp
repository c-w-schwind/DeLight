#include <Arduino.h>
#include <BLEDevice.h>
#include <WiFi.h> 
#include <HTTPClient.h>
#include <movingAvg.h>
#include "mySecrets.h"
#include <ArduinoJson.h>
#include <time.h>
#include <Adafruit_NeoPixel.h>

//NEOPIXEL
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define NUMPIXELS 12
Adafruit_NeoPixel pixels(NUMPIXELS, 16, NEO_GRB + NEO_KHZ800);

//WIFI
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;

//API
const char* serverPathLatLon = "https://api.openweathermap.org/data/2.5/weather?lat=50.9375&lon=6.9603&appid=5a246f369afff4ce3d85a772dfe8e031";
const char* serverPathZipCode = "https://api.openweathermap.org/data/2.5/weather?zip=51103,de&appid=5a246f369afff4ce3d85a772dfe8e031";
const char* localhost = "http://192.168.1.116:8000/json";
unsigned long lastAPICall;
unsigned long apiCallFrequency = 20000;

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
long sunrise, sunset;
long timeToSunset, timeToSunrise;
double cloudiness;
double rainVolume;

//LAMP
bool isLampOn = false;
int lampBrightness;
int sunFactor;
int rainFactor;
int cloudFactor;

//TIME
const char* ntpServer = "pool.ntp.org";
unsigned long currentTime; 
unsigned long timestamp;
int timeTracker = 0, totalSecondsToday;


void getTimeFromNTP() {
  time_t now;
  struct tm timeinfo;
  Serial.println("\nObtaining current time in unix format...");
  while (!getLocalTime(&timeinfo, 30000U)) {    // trying for 30 seconds. necessary. won't continue until time is got
    Serial.println("Failed to obtain time within 30 seconds. Trying again.");
  }
  timestamp = millis();
  time(&now);
  Serial.println("Successfully obtained current time.\n");
  currentTime = now;
}

void updateCurrentTime() {
  unsigned long timeVar = millis();
  currentTime += (timeVar - timestamp) / 1000;
  timeVar -= (timeVar - timestamp) % 1000;
  timestamp = timeVar;
}

void synchTimeOncePerDay() {
  totalSecondsToday = currentTime % 86400;
  if (totalSecondsToday < timeTracker)
    getTimeFromNTP();
  timeTracker = totalSecondsToday;
}

bool connectToServer(BLEAddress pAddress) {
  pClient->connect(pAddress);
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  return pRemoteService != nullptr ? true : false;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Scan Result: %s \n", advertisedDevice.toString().c_str());
  }
};

void initBLEScan() {
  BLEDevice::init("");
  Serial.println("ESP32 BLE DeLight program. Scanning for partner device...");
  pBLEScan = BLEDevice::getScan();                                            // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());  // Call the class that is defined above
  pBLEScan->setActiveScan(true);                                              // active scan uses more power, but get results faster
  //pBLEScan->setInterval(scanInterval);                                      // in msec
  //pBLEScan->setWindow(scanWindow);                                          // in msec
  //pBLEScan->start(scanTime);                                                // in sec
}

bool connectWifi() {
  if(WiFi.status() != WL_CONNECTED) {
    int counter = 0;
    WiFi.begin(ssid, password);
    Serial.printf("\nAttempting to connect to WiFi with SSID \"%s\"\n", ssid);
    while (counter < 40) {  //attempting connection ends after 20 seconds (40 * 500ms)
      if (WiFi.status() != WL_CONNECTED) {
        delay (500);
        Serial.print(".");
      } else {
        Serial.print("\nConnected to WiFi network with IP Address: ");
        Serial.println(WiFi.localIP());
        return true;
      }
      ++counter;
    }
    Serial.print("\nCould not connect to WiFi network: ");
    Serial.println(ssid);
    return false;
  }
  return true;
}

void disconnectWifi() {
  WiFi.disconnect();
  Serial.println("Disconnected WiFi.\n");
}

String getRequestWeatherAPI() {
  Serial.println("\nRequesting weather data from API...");
  HTTPClient http;
  http.begin(serverPathZipCode);
  int httpResponseCode = http.GET();
  String payload = "{}";
  if (httpResponseCode > 0) {
    Serial.println("Weather API HTTP response code: " + String(httpResponseCode));
    payload = http.getString();
  } else {
    Serial.println("API get request error code: " + String(httpResponseCode));
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
  sunrise = doc["sys"]["sunrise"];
  sunset = doc["sys"]["sunset"];
  rainVolume = doc["rain"]["1h"];
  cloudiness = doc["clouds"]["all"];
}

void setSunFactor(long timeToSunset, long timeToSunrise) {
  if (timeToSunrise < 1800 && timeToSunrise >= -3600) {
    sunFactor = 255 - (1800 - timeToSunrise) / 21; // range of 5400 seconds mapped onto 255 brightness steps
  } else if (timeToSunset < 1800 && timeToSunset >= -3600) {
    sunFactor = (1800 - timeToSunset) / 21;
  } else
   sunFactor = 0;     // TODO: day 0, night 255
}

void setCloudFactor() {
  cloudFactor = int(cloudiness * 0.5);
}

void setRainFactor() {
  rainFactor = int(rainVolume * 30.0);
}

void updateValues() {
  updateCurrentTime();
  timeToSunset = sunset - currentTime;
  timeToSunrise = sunrise - currentTime;
  setSunFactor(timeToSunset, timeToSunrise);      // TODO: je höher der Wert, desto weniger blau
  setCloudFactor();
  setRainFactor();
}

void calculateLampBrightness() {
  lampBrightness = 100 + sunFactor + cloudFactor + rainFactor;
  // TODO: interface values + default values default nur bei tag?

}

void turnLampOn() {                   // TODO: default lamp settings. siehe calculateLampBrightness: default. only when settings not updated for a while
  pixels.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<NUMPIXELS; i++) {
    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));
  }
  //for (int i = 0; i < 255; ++i) {
  //  pixels.setBrightness(i);

    pixels.show();
  //  delay(10);
  //}
  digitalWrite(27,HIGH);
  isLampOn = true;
  Serial.println("\nLAMP ON\n");
  // TODO: check if in sunset or rise
  // TODO: mit settings von interface überschreiben (wenn gesetzt)
}

void turnLampOff() {
  //for (int i = 255; i > 0; --i) {
  //  pixels.setBrightness(i);
    pixels.clear();
    pixels.show();
  //  delay(10);
  //}
  digitalWrite(27,LOW);
  isLampOn = false;
  Serial.println("\nLAMP OFF\n");
  
}

void updateLamp() {                   // TODO: input variables (?)
  updateValues();
  calculateLampBrightness();
  //analogWrite(14, lampBrightness); // lampBrightness + interface Werte
                          // TODO: smoothing function with delay for jumping values
}

void updateRssiWithDelay() {
  rssi = pClient->getRssi();
  Serial.printf("RSSI: %d       ", rssi);
  Serial.printf("Moving Average: %d       ", rssiAvg.reading(rssi));
  Serial.printf("Average calculated from %d inputs.\n", rssiAvg.getCount());
  delay(500);
}

void printInfo() {
  Serial.print("\nRight now in ");
  Serial.printf(doc["name"]);
  Serial.printf("\n\tTime to sunset: %ld\n", timeToSunset);
  Serial.printf("\tTime to sunrise: %ld\n", timeToSunrise);
  Serial.printf("\tSun factor is %d.\n", sunFactor);
  Serial.printf("\tCloud factor is %d with a cloudiness percentage of %f.\n", cloudFactor, cloudiness);
  Serial.printf("\tRain factor is %d with a rain volume of %f within the last hour.\n\n", rainFactor, rainVolume);
}


void setup() {
  Serial.begin(115200);
  initBLEScan();
  pClient = BLEDevice::createClient();
  rssiAvg.begin();
  pinMode(27, OUTPUT);
  configTime(0, 0, ntpServer);
  while (!connectWifi()); // necessary. won't continue until connected.
  getTimeFromNTP();
  disconnectWifi();
  pixels.begin();
}

void loop() {
  /*
  while (!pClient->isConnected()) {
    Serial.println("Starting BLE scan...");
    foundDevices = pBLEScan->start(3); // Scan for 3 seconds
    Serial.printf("\nScan done.\nNumber of BLE devices found: %d\n\n\n", foundDevices.getCount());
    for (int i = 0; i < foundDevices.getCount(); i++) {
      Scanned_BLE_Address = foundDevices.getDevice(i).getAddress().toString().c_str();
      if (Scanned_BLE_Address == Partner_BLE_Address) {
        Serial.println("Found partner device. Connecting to server as client.");
        connectToServer(foundDevices.getDevice(i).getAddress()) ? Serial.println("Connected to partner device.") : Serial.println("Connection to partner device failed.");
      }
    }
  }

  while (pClient->isConnected()) {
    pBLEScan->stop();// TODO: check
    // */
  while(true) {
    updateRssiWithDelay();
    if (rssiAvg.getAvg() < -80) {
      if (isLampOn) turnLampOff();
    } else {
      if (!isLampOn) {
        turnLampOn();         // TODO: only default if for a while no updated settings yet? 
        lastAPICall = millis() - apiCallFrequency;  
      }
      if (millis() - lastAPICall >= apiCallFrequency) {       // TODO: every 10 Minutes? 20?
        if (connectWifi()) {                                  // TODO: disconnect? seems reasonable with > 10 minutes
          parseWeatherData();
          printInfo();
          synchTimeOncePerDay();
          lastAPICall = millis();
          //disconnectWifi();
        }      
      }
      updateLamp(); // TODO: here? independend of api call, because of sun value. 
    }
  }

  Serial.println("\nDisconnected from partner device.");
  disconnectWifi();
  rssiAvg.reset();
}

// connect wifi
// api call
// parse api
// set values
//
//

//
// Bool zum ausschalten der Automatik
// Zusätzliche konstante Werte zur Automatik (+/- Helligkeit)
// Default Werte einstellen
//
//