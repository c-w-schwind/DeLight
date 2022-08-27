#include <Arduino.h>
#include <BLEDevice.h>
#include <WiFi.h> 
#include <HTTPClient.h>
#include <movingAvg.h>
#include "mySecrets.h"
#include <ArduinoJson.h>
#include <time.h>
#include <Adafruit_NeoPixel.h>

// DEFAULTS
#define DEFAULT_HUE 5000

// NEOPIXEL VARIABLES
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define NUMPIXELS 12
Adafruit_NeoPixel pixels(NUMPIXELS, 16, NEO_GRB + NEO_KHZ800);

// WIFI VARIABLES
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;

// USER INTERFACE VARIABLES
unsigned long lastUserInterfaceCall;
unsigned int userHue;
float userSaturation;
int userBrightnessFactor;
unsigned long callFrequencyUserInterface = 5000;    // check for updates every 5 seconds

// API VARIABLES
const char* serverPathLatLon = "https://api.openweathermap.org/data/2.5/weather?lat=50.9375&lon=6.9603&appid=5a246f369afff4ce3d85a772dfe8e031";
const char* serverPathZipCode = "https://api.openweathermap.org/data/2.5/weather?zip=51103,de&appid=5a246f369afff4ce3d85a772dfe8e031";
const char* serverPathLocalhost = "http://192.168.178.39:8080/json"; // Future update: iteration function for finding 4th number?
unsigned long lastAPICall;
unsigned long callFrequencyAPI = 60000 * 5;       // every 5 Minutes

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
bool isLampOn = false;
bool isManualMode = true;

int nightFilter;
int sunFactor;
int rainFactor;
int cloudFactor;

int lampSaturation = 255;
int lampSaturationOld;
int lampBrightness;
int lampBrightnessOld;
unsigned int lampHue = 5000;
unsigned int lampHueOld;

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
  pBLEScan->setActiveScan(true);                                              // active scan uses more power, but get results faster TODO
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


// USER INTERFACE

String getRequestUserInterface() {
  Serial.println("\nRequesting user data from interface...");
  HTTPClient http;
  http.begin(serverPathLocalhost);
  int httpResponseCode = http.GET();
  String payload = "{}";
  if (httpResponseCode > 0) {
    Serial.println("User interface HTTP response code: " + String(httpResponseCode) + "\n");
    payload = http.getString();
  } else {
    Serial.println("User interface get request error code: " + String(httpResponseCode) + "\n");
  }
  http.end();
  return payload;
}

void parseUserInterfaceData(String userInterfaceData) {
  DeserializationError error = deserializeJson(doc, userInterfaceData);
  if (error) {
    Serial.print("deserializeJson() of user interface data failed: ");
    Serial.println(error.c_str());
    return;
  }
  userHue = doc["color"][0];
  userSaturation = doc["color"][1];
  userBrightnessFactor = doc["brightness"];
  isManualMode = doc["automationOff"];
}

void updateUserInterfaceFactors () {
  if (connectWifi()) {
    parseUserInterfaceData(getRequestUserInterface());
    lastUserInterfaceCall = millis();
    userHue *= 182;         // mapped from 360 values to 65.536
    userSaturation *= 2.55; // mapped from 100 values to 255
    if (!isManualMode) {  // in automation mode user should be able to completely overpower brightness values set by other factors
      userBrightnessFactor = (userBrightnessFactor - 128) * 2;
    }
  } else 
    Serial.println("Connection to WiFi was not possible. User interface factors have not been updated.");
}


// WEATHER

String getRequestWeatherAPI() {
  Serial.println("\nRequesting weather data from API...");
  HTTPClient http;
  http.begin(serverPathZipCode);
  int httpResponseCode = http.GET();
  String payload = "{}";
  if (httpResponseCode > 0) {
    Serial.println("Weather API HTTP response code: " + String(httpResponseCode) + "\n");
    payload = http.getString();
  } else {
    Serial.println("API get request error code: " + String(httpResponseCode) + "\n");
  }
  http.end();
  return payload;
}

void parseWeatherData(String weatherData) {
  DeserializationError error = deserializeJson(doc, weatherData);
  if (error) {
    Serial.print("deserializeJson() of weather data failed: ");
    Serial.println(error.c_str());
    return;
  }
  sunrise = doc["sys"]["sunrise"];
  sunset = doc["sys"]["sunset"];
  rainVolume = doc["rain"]["1h"];
  cloudiness = doc["clouds"]["all"];
}

void printWeatherInfo() {
  Serial.print("Right now in ");
  Serial.printf(doc["name"]);
  Serial.printf("\n\tTime to sunset: %ld\n", timeToSunset);
  Serial.printf("\tTime to sunrise: %ld\n", timeToSunrise);
  Serial.printf("\tCloud factor is %d with a cloudiness percentage of %f.\n", cloudFactor, cloudiness);
  Serial.printf("\tRain factor is %d with a rain volume of %f within the last hour.\n\n", rainFactor, rainVolume);
}

void setNightFilter() {
  if (timeToSunrise < 900 && timeToSunrise >= -900) {         // decreasing blue light filter over 30 minutes. starting 15 minutes before sunrise.
    nightFilter = 100 - (900 - timeToSunrise) / 7;            // range of 1800 seconds mapped onto 255 saturation steps.
  } else if (timeToSunset < 900 && timeToSunset >= -900) {    // increasing blue light filter over 30 minutes. starting 15 minutes before sunset.
    nightFilter = (900 - timeToSunset) / 7;                   // range of 1800 seconds mapped onto 255 saturation steps.
  } else nightFilter = isNight() ? 255 : 0;                   // complete filter active over the course of the night.
}

void setSunFactor() {
  if (timeToSunrise < 900 && timeToSunrise >= -1800) {        // decreasing brightness over 45 minutes. starting 15 minutes before sunrise.
    sunFactor = 255 - (900 - timeToSunrise) / 10.6;           // range of 2700 seconds mapped onto 255 brightness steps.
  } else if (timeToSunset < 900 && timeToSunset >= -1800) {   // increasing brightness over 45 minutes. starting 15 minutes before sunset.
    sunFactor = (900 - timeToSunset) / 10.6;                  // range of 2700 seconds mapped onto 255 brightness steps.
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

void calculateLampHSV() {
  lampHueOld = lampHue;
  lampSaturationOld = lampSaturation;
  lampBrightnessOld = lampBrightness;
  
  if (!isManualMode) {
    lampBrightness = sunFactor + cloudFactor + rainFactor;
    if (lampBrightness > 255) lampBrightness = 255;
    lampBrightness += userBrightnessFactor;   // in automation mode userBrightnessFactor ranges from -256
    lampSaturation = nightFilter;             // to 255 for ability to completely overpower auto values
    lampHue = DEFAULT_HUE;
  } else {
    lampHue = userHue;
    lampSaturation = userSaturation;
    lampBrightness = userBrightnessFactor;
  }

  lampBrightness = lampBrightness > 255 ? 255 : lampBrightness < 0 ? 0 : lampBrightness;
  if (lampSaturation > 255) lampSaturation = 255;
}

void calculateAllValuesForLampUpdate() {
  millis() - lastAPICall >= callFrequencyAPI ? updateWeatherAndTimeFactorsByAPICall() : updateWeatherAndTimeFactorsLocally();
  if (millis() - lastUserInterfaceCall >= callFrequencyUserInterface) 
    updateUserInterfaceFactors();
  calculateLampHSV();
}


void turnLampOff() {
  for (int i = lampBrightness; i >= 0; --i) {
    pixels.fill(pixels.ColorHSV(lampHue, lampSaturation, i));
    pixels.show();
    delay(15);
  }
  isLampOn = false;
  Serial.println("\nLAMP OFF\n");
}

void turnLampOn() {
  if (isManualMode) {
    lampBrightness = userBrightnessFactor;
  }
  for (int i = 0; i < lampBrightness; ++i) {
    pixels.fill(pixels.ColorHSV(lampHue, lampSaturation, i));
    pixels.show();
    delay(5);
  }
  isLampOn = true;
  Serial.println("\nLAMP ON\n");
}

void updateLamp() {
  for (int i = lampBrightnessOld; i != lampBrightness; lampBrightnessOld < lampBrightness ? ++i : --i) {
    pixels.fill(pixels.ColorHSV(lampHueOld, lampSaturationOld, i));
    pixels.show();
    delay(5);
  }
  if (lampBrightness == 0 && isLampOn) {
    turnLampOff();
  } else {
    if (lampHue != lampHueOld) {                                      // if change in hue value:
      for (int i = lampSaturationOld; i > 0; --i) {                   // - take saturation away
        pixels.fill(pixels.ColorHSV(lampHueOld, i, lampBrightness));
        pixels.show();
        delay(4);
      }
      pixels.fill(pixels.ColorHSV(lampHue, 0, lampBrightness));       // - set correct hue
      for (int i = 0; i < lampSaturation; ++i) {                      // - then bring saturation back
        pixels.fill(pixels.ColorHSV(lampHue, i, lampBrightness));
        pixels.show();
        delay(2);
      }
    } else {
      for (int i = lampSaturationOld; i != lampSaturation; lampSaturationOld < lampSaturation ? ++i : --i) {
        pixels.fill(pixels.ColorHSV(lampHue, i, lampBrightness));
        pixels.show();
        delay(3);
      }
    }
  }
}



void setup() {
  Serial.begin(115200);
  delay(500);                   // sometimes printed lines got cut off...
  configTime(0, 0, ntpServer);
  while (!connectWifi());       // necessary. won't continue until connected.
  getTimeFromNTP();
  updateWeatherAndTimeFactorsByAPICall();
  updateUserInterfaceFactors();
  calculateLampHSV();
  disconnectWifi();
  Serial.println("Sun factor is: ");
  Serial.println(vari);
  initBLEScan();
  pClient = BLEDevice::createClient();
  rssiAvg.begin();
  
  pixels.begin();
  pixels.clear();
  pixels.show();
}

void loop() {
  /*
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
  while(true) {
    updateRssiWithDelay();
    if (rssiAvg.getAvg() < -80) {
      if (isLampOn) turnLampOff();
    } else {
      calculateAllValuesForLampUpdate();
      if (!isLampOn && lampBrightness != 0) {
        turnLampOn();
      }
      updateLamp();
    }
  }

  Serial.println("\nDisconnected from partner device.");
  disconnectWifi();
  rssiAvg.reset();
}


// rename: isManual --> also in ui.
// explain ui in ui.