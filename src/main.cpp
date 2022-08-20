#include <Arduino.h>
#include <BLEDevice.h>
#include <WiFi.h> 
#include <HTTPClient.h>
#include <movingAvg.h>
#include "mySecrets.h"
//#include "esp_coexist_internal.h"
//#define CONFIG_SW_COEXIST_ENABLE 1

//BLE
static BLEUUID serviceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59"); 
static BLEUUID charUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");   
static BLERemoteCharacteristic *pRemoteCharacteristic;
String Partner_BLE_Address = "b4:e6:2d:eb:17:73";                       
BLEScan *pBLEScan;
BLEScanResults foundDevices;
BLEClient *pClient;
static BLEAddress *Server_BLE_Address;
String Scanned_BLE_Address;
int rssi;
movingAvg rssiAvg(10);

//WIFI
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
std::string serverPath = "https://api.openweathermap.org/data/2.5/weather?lat=50.9375&lon=6.9603&appid=5a246f369afff4ce3d85a772dfe8e031";

bool one = true;
bool two = true;
unsigned long lastAPICall;

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

void getWeather() {
  HTTPClient http;
  http.begin(serverPath.c_str());
  int httpResponseCode = http.GET();    // Send HTTP GET request
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);     // TODO: Response code error handling
  String payload = http.getString();    // TODO: parse payload and extract
  Serial.println(payload);
  http.end();                           // Free resources
}

void lampOff() {
  digitalWrite(14, LOW);
  digitalWrite(26, LOW);
  digitalWrite(33, LOW);
  Serial.println("\nLED OFF\n");
}

void lampOn() {                         // TODO: default lamp settings
  digitalWrite(14, HIGH);   
  digitalWrite(26, HIGH);
  digitalWrite(33, HIGH);
  Serial.println("\nLED ON\n");
}

void lampAdjust(int brightness) {       // TODO: input variables

}

void setup() {
  Serial.begin(115200);
  initBLEScan();
  pClient = BLEDevice::createClient();
  rssiAvg.begin();
  pinMode(14, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(33, OUTPUT);
}


void loop() {
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
    rssi = pClient->getRssi();
    Serial.printf("RSSI: %d       ", rssi);
    Serial.printf("Moving Average: %d       ", rssiAvg.reading(rssi));
    Serial.printf("Average input count: %d\n", rssiAvg.getCount());
    delay(500);
    if (rssiAvg.getAvg() < -80) {
      if (one) {
        lampOff();
        one = false;
        two = true;
      }
    } else {
      if (two) {
        lampOn();
        lastAPICall = millis() - 60000UL;
        two = false;
        one = true;
      }
      if (millis() - lastAPICall >= 60000UL){
        if(WiFi.status() != WL_CONNECTED) 
          connectWifi();
        lastAPICall = millis();
        getWeather();
        lampAdjust();
      }
       
    }
  }

  Serial.println("\nDisconnected\n");
  WiFi.disconnect();
  rssiAvg.reset();
}