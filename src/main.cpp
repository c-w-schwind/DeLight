#include <Arduino.h>
#include <BLEDevice.h>
//#include <ArduinoHttpClient.h>
#include <WiFi.h> 
#include <HTTPClient.h>
#include "mySecrets.h"
//#include "esp_coexist_internal.h"
//#define CONFIG_SW_COEXIST_ENABLE 1

//BLE
static BLEUUID serviceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59"); 
static BLEUUID charUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");   
static BLERemoteCharacteristic *pRemoteCharacteristic;
String My_BLE_Address = "08:3a:f2:b9:0b:fa";                       
BLEScan *pBLEScan;
BLEScanResults foundDevices;
BLEClient *pClient;
static BLEAddress *Server_BLE_Address;
String Scanned_BLE_Address;
int rssi;
boolean paired = false; // boolean variable to togge light

//WIFI
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
std::string serverName = "https://api.openweathermap.org/data/2.5/weather?lat=50.9375&lon=6.9603&appid=5a246f369afff4ce3d85a772dfe8e031";


bool connectToServer(BLEAddress pAddress) {
  pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  // Connect to the BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to lamp");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService != nullptr) {
    Serial.println(" - Found our service");
    return true;
  } else {
    return false;
  }
  /*  // unreachable code. delete?
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  
  if (pRemoteCharacteristic != nullptr)
    Serial.println(" - Found our characteristic");
  return true;
  */
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Scan Result: %s \n", advertisedDevice.toString().c_str());
    Server_BLE_Address = new BLEAddress(advertisedDevice.getAddress());
  }
};

void scanForDevices() {
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
  while (counter < 40) {  //if connection cannot be established, attempting ends after 20 seconds (40 * 500ms)
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
  std::string serverPath = serverName;  // + "?temperature=24.37";
  http.begin(serverPath.c_str());
    
  int httpResponseCode = http.GET();    // Send HTTP GET request
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  String payload = http.getString();
  Serial.println(payload);
  //TODO: parse payload and extract
  http.end();                           // Free resources
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  scanForDevices();
  pinMode(14, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(33, OUTPUT);
}


void loop() {
  // TODO: First search for device, then call weather. (if too slow: occasionally call weather api (e.g. every 30 min) for quick settings roughly correct)
  
  while (paired == false) {
    foundDevices = pBLEScan->start(3); // Scan for 3 seconds to find the Fitness band
    Serial.printf("Scan done.\n");
    Serial.printf("Number of BLE devices found: %d\n", foundDevices.getCount());

    for (int i = 0; i < foundDevices.getCount(); i++) {
      Scanned_BLE_Address = foundDevices.getDevice(i).getAddress().toString().c_str();
      if (Scanned_BLE_Address == My_BLE_Address && paired == false) {
        Serial.println("Found Device. Connecting to Server as client");
        if (connectToServer(foundDevices.getDevice(i).getAddress())) {
          paired = true;
          break;
        } else {
          Serial.println("Pairing failed");
          break;
        }
      }
    }
  }
  while (paired == true) {
    if (pClient->isConnected()) {
      rssi = pClient->getRssi();
      Serial.printf("RSSI: %d\n", rssi);
      delay(1000);                  //??????????
      if (rssi < -80) {
        Serial.println("LED OFF");
        digitalWrite(14, LOW);
        digitalWrite(26, LOW);
        digitalWrite(33, LOW);
        break;
      } else {
        Serial.println("LED ON");
        // switch light on with low brightness (default; also settable via interface)
        if(connectWifi()){
          getWeather();
          //WiFi.disconnect();    // TODO:  which one?
          //WiFi.mode(WIFI_OFF);  //        which one?
          delay(1000);            // TODO:  how long?
        }
        digitalWrite(14, HIGH);
        digitalWrite(26, HIGH);
        digitalWrite(33, HIGH);
        break;
      }
    } else {
      Serial.println("Disconnected");
      paired = false;
    }
  }
  
  
  //pBLEScan->stop();
  //btStop();
  Serial.print(5000);
  delay(6000);
  //*/
}