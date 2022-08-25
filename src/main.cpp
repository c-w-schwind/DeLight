#include <Arduino.h>
#include <BLEDevice.h>
#include <WiFi.h> 
#include <HTTPClient.h>
#include <movingAvg.h>
#include "mySecrets.h"
#include <ArduinoJson.h>
#include <time.h>
#include <Adafruit_NeoPixel.h>

// NEOPIXEL VARIABLES
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define NUMPIXELS 12
Adafruit_NeoPixel pixels(NUMPIXELS, 16, NEO_GRB + NEO_KHZ800);

// WIFI VARIABLES
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;

// API VARIABLES
const char* serverPathLatLon = "https://api.openweathermap.org/data/2.5/weather?lat=50.9375&lon=6.9603&appid=5a246f369afff4ce3d85a772dfe8e031";
const char* serverPathZipCode = "https://api.openweathermap.org/data/2.5/weather?zip=51103,de&appid=5a246f369afff4ce3d85a772dfe8e031";
const char* localhost = "http://192.168.1.116:8000/json"; // TODO: iteration function for finding fourth number?
unsigned long lastAPICall;
unsigned long apiCallFrequency = 10000;       // TODO: every 10 Minutes? 20?

// BLE VARIABLES
static BLEUUID serviceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59"); 
static BLEUUID charUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");   
static String Partner_BLE_Address = "b4:e6:2d:eb:17:73";                       
static BLERemoteCharacteristic *pRemoteCharacteristic;
BLEClient *pClient;
BLEScan *pBLEScan;
BLEScanResults foundDevices;
String Scanned_BLE_Address;

// RSSI VARIABLES
int rssi;
movingAvg rssiAvg(10);

// WEATHER VARIABLES
StaticJsonDocument<1024> doc;
long sunrise, sunset;
long timeToSunset, timeToSunrise;
double cloudiness;
double rainVolume;

// LIGHT VARIABLES
int additionalLamp = 27;
bool isLampOn = false;
bool isNightFiltered = true;
int lampSaturation;
int lampBrightness;
unsigned int lampHue;
unsigned int hueValue = 5000;
int nightFilter;
int sunFactor;
int rainFactor;
int cloudFactor;


// TIME VARIABLES
const char* ntpServer = "pool.ntp.org";
unsigned long currentTime; 
unsigned long timestamp;
int timeTracker = 0, totalSecondsToday;


// TIME

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

bool isNight() {
  return timeToSunrise > 0 || timeToSunset < 0 ? true : false;
}


// BLE CONNECTION

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

void updateRssiWithDelay() {
  rssi = pClient->getRssi();
  Serial.printf("RSSI: %d       ", rssi);
  Serial.printf("Moving Average: %d       ", rssiAvg.reading(rssi));
  Serial.printf("Average calculated from %d inputs.\n", rssiAvg.getCount());
  delay(500);
}


// WIFI

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


// WEATHER

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

void parseWeatherData(String weatherData) {
  DeserializationError error = deserializeJson(doc, weatherData);
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

void printWeatherInfo() {
  Serial.print("\nRight now in ");
  Serial.printf(doc["name"]);
  Serial.printf("\n\tTime to sunset: %ld\n", timeToSunset);
  Serial.printf("\tTime to sunrise: %ld\n", timeToSunrise);
  Serial.printf("\tSun factor is %d.\n", sunFactor);
  Serial.printf("\tCloud factor is %d with a cloudiness percentage of %f.\n", cloudFactor, cloudiness);
  Serial.printf("\tRain factor is %d with a rain volume of %f within the last hour.\n\n", rainFactor, rainVolume);
}

void setNightFilter() {
  if (isNightFiltered) {
    if (timeToSunrise < 900 && timeToSunrise >= -900) {       // decreasing blue light filter over 30 minutes. starting 15 minutes before sunrise.
      nightFilter = 100 - (900 - timeToSunrise) / 7;          // range of 1800 seconds mapped onto 255 saturation steps.
    } else if (timeToSunset < 900 && timeToSunset >= -900) {  // increasing blue light filter over 30 minutes. starting 15 minutes before sunset.
      nightFilter = (900 - timeToSunset) / 7;                 // range of 1800 seconds mapped onto 255 saturation steps.
    } else nightFilter = isNight() ? 255 : 0;                 // complete filter active over the course of the night.
  }
}

void setSunFactor() {
  if (timeToSunrise < 1800 && timeToSunrise >= -1800) {       // decreasing brightness over 90 minutes. starting 30 minutes before sunrise.
    sunFactor = 255 - (1800 - timeToSunrise) / 14;            // range of 3600 seconds mapped onto 255 brightness steps.
  } else if (timeToSunset < 1800 && timeToSunset >= -1800) {  // increasing brightness over 90 minutes. starting 30 minutes before sunset.
    sunFactor = (1800 - timeToSunset) / 14;                   // range of 3600 seconds mapped onto 255 brightness steps.
  } else sunFactor = isNight() ? 255 : 0;                     // full brightness setting active over the course of the night.
}

void setCloudFactor() {
  cloudFactor = int(cloudiness * 0.5);
}

void setRainFactor() {
  rainFactor = int(rainVolume * 30.0);
}

void updateWeatherAndTimeFactorsLocally() {
  updateCurrentTime();
  timeToSunset = sunset - currentTime;
  timeToSunrise = sunrise - currentTime;
  setNightFilter();
  setSunFactor();
  setCloudFactor();
  setRainFactor();
}

void updateWeatherAndTimeFactorsByAPICall () {
  if (connectWifi()) {
    parseWeatherData(getRequestWeatherAPI());
    lastAPICall = millis();
    synchTimeOncePerDay();
    updateWeatherAndTimeFactorsLocally();
    printWeatherInfo();
  } else 
    Serial.println("Connection to WiFi was not possible. Weather and time factors have not been updated.");
}


// LIGHT SETTINGS

void calculateLampHSV() { // TODO: default lamp settings. siehe calculateLampHSV: default. only when settings not updated for a while
  if (lastAPICall == 0 || millis() - lastAPICall >= apiCallFrequency * 3 /*|| isActiveUserDefault*/) { // call default values if: too long since last update
    lampHue = hueValue;                                                       // TODO: hier drüber: isActiveUserDefault bool & logic
    lampSaturation = 300;       // TODO: ersetzen mit defaults, die von user gesetzt werden können
    lampBrightness = 255;
    lastAPICall = millis() - apiCallFrequency;
  } else {
    lampBrightness = sunFactor + cloudFactor + rainFactor;  // + interface + default (<-- separately? or interface changes default?)
    lampSaturation = nightFilter;                           // + interface  
    lampHue = hueValue;   // TODO: duplicated line...
    // TODO: interface values + default values default nur bei tag?
  }
  lampBrightness = lampBrightness > 255 ? 255 : lampBrightness;
  lampSaturation = lampSaturation > 255 ? 255 : lampSaturation;
}

void calculateAllValuesForLampUpdate() {
  millis() - lastAPICall >= apiCallFrequency ? updateWeatherAndTimeFactorsByAPICall() : updateWeatherAndTimeFactorsLocally();
  calculateLampHSV();
}

void turnLampOff() {
  for (int i = lampBrightness; i >= 0; --i) {
    analogWrite(additionalLamp,i);
    pixels.fill(pixels.ColorHSV(lampHue, lampSaturation, i));
    pixels.show();
    delay(15);
  }
  isLampOn = false;
  Serial.println("\nLAMP OFF\n");
}

void turnLampOn() {
  for (int i = 0; i < lampBrightness; ++i) {
    analogWrite(additionalLamp,i);
    pixels.fill(pixels.ColorHSV(lampHue, lampSaturation, i));
    pixels.show();
    delay(5);
  }
  isLampOn = true;                    // TODO: check if in sunset or rise
  Serial.println("\nLAMP ON\n");      // TODO: mit settings von interface überschreiben (wenn gesetzt)
}

void updateLamp() {
  // TODO: smoothing function with delay for jumping values: while (oldLampHue != lampHue && oldLampSaturation != lampSaturation && ) ++
  analogWrite(additionalLamp,lampBrightness);
  pixels.fill(pixels.ColorHSV(lampHue, lampSaturation, lampBrightness));
  pixels.show();
}


void setup() {
  Serial.begin(115200);
  delay(2000); //sometimes
  configTime(0, 0, ntpServer);
  while (!connectWifi()); // necessary. won't continue until connected.
  getTimeFromNTP();
  updateWeatherAndTimeFactorsByAPICall();
  disconnectWifi();

  initBLEScan();
  pClient = BLEDevice::createClient();
  rssiAvg.begin();
  
  pinMode(additionalLamp, OUTPUT);
  pixels.begin();
  pixels.clear();
  pixels.show();
  lastAPICall = 0;
}

void loop() {
  ///*
  while (!pClient->isConnected()) {
    Serial.println("Starting new BLE scan...");
    foundDevices = pBLEScan->start(3); // Scanning for 3 seconds
    Serial.printf("\nScan done.\nNumber of BLE devices found: %d\n\n", foundDevices.getCount());
    for (int i = 0; i < foundDevices.getCount(); i++) {
      Scanned_BLE_Address = foundDevices.getDevice(i).getAddress().toString().c_str();
      if (Scanned_BLE_Address == Partner_BLE_Address) {
        Serial.println("Found partner device. Connecting to server as client.");
        connectToServer(foundDevices.getDevice(i).getAddress()) ? Serial.println("Connected to partner device.") : Serial.println("Connection to partner device failed.");
      }
    }
  }

  while (pClient->isConnected()) {
    // */
  //while(true) {
    updateRssiWithDelay();
    if (rssiAvg.getAvg() < -80) {
      if (isLampOn) turnLampOff();
    } else {
      calculateAllValuesForLampUpdate();
      if (!isLampOn) {
        turnLampOn();
      }
      updateLamp();
    }
  }

  Serial.println("\nDisconnected from partner device.");
  disconnectWifi();
  rssiAvg.reset();
}

//
// Bool zum ausschalten der Automatik
// Zusätzliche konstante Werte zur Automatik (+/- Helligkeit)
// Default Werte einstellen
//
//