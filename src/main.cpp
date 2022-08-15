#include <Arduino.h>
#include <BLEDevice.h> //Header file for BLE
//#include <ArduinoHttpClient.h>
#include <WiFi.h> 
#include <HTTPClient.h>
#include "mySecrets.h"

static BLEUUID serviceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59"); // Service UUID of fitnessband obtained through nRF connect application
static BLEUUID charUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");    // Characteristic  UUID of fitnessband obtained through nRF connect application
String My_BLE_Address = "b4:e6:2d:eb:0b:df";                        // Hardware Bluetooth MAC of my fitnessband, will vary for every band obtained through nRF connect application
static BLERemoteCharacteristic *pRemoteCharacteristic;

BLEScan *pBLEScan; // Name the scanning device as pBLEScan
BLEScanResults foundDevices;

BLEClient *pClient;
int rssi;

const char* ssid = SECRET_SSID;
const char* password =  SECRET_PASS;

std::string serverName = "https://api.openweathermap.org/data/2.5/weather?lat=50.9375&lon=6.9603&appid=5a246f369afff4ce3d85a772dfe8e031";

//unsigned long lastTime = 0;
//unsigned long timerDelay = 5000;


static BLEAddress *Server_BLE_Address;
String Scanned_BLE_Address;

boolean paired = false; // boolean variable to togge light

bool connectToServer(BLEAddress pAddress)
{

  pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  // Connect to the BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to lamp");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService != nullptr)
  {
    Serial.println(" - Found our service");
    return true;
  }
  else
    return false;
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic != nullptr)
    Serial.println(" - Found our characteristic");
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.printf("Scan Result: %s \n", advertisedDevice.toString().c_str());
    Server_BLE_Address = new BLEAddress(advertisedDevice.getAddress());
  }
};

void setup()
{
  Serial.begin(115200);
  delay(2000);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  
  Serial.println("ESP32 BLE Server program");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();                                           // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // Call the class that is defined above
  pBLEScan->setActiveScan(true);                                             // active scan uses more power, but get results faster
  
  pinMode(14, OUTPUT); // Declare the in-built LED pin as output
}

void loop()
{

  //Check WiFi connection status
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    std::string serverPath = serverName;// + "?temperature=24.37";
    
    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());
    
    // Send HTTP GET request
    int httpResponseCode = http.GET();
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
    Serial.println(payload);

    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  delay(60000);

  /*
  while (paired == false)
  {
    foundDevices = pBLEScan->start(3); // Scan for 3 seconds to find the Fitness band
    Serial.printf("Scan done.\n");
    Serial.printf("Number of BLE devices found: %d\n", foundDevices.getCount());

    for (int i = 0; i < foundDevices.getCount(); i++)
    {
      Scanned_BLE_Address = foundDevices.getDevice(i).getAddress().toString().c_str();
      if (Scanned_BLE_Address == My_BLE_Address && paired == false)
      {
        Serial.println("Found Device. Connecting to Server as client");
        if (connectToServer(foundDevices.getDevice(i).getAddress()))
        {
          paired = true;
          break;
        }
        else
        {
          Serial.println("Pairing failed");
          break;
        }
      }
    }
  }
  while (paired == true)
  {
    if (pClient->isConnected())
    {
      rssi = pClient->getRssi();
      Serial.printf("RSSI: %d\n", rssi);
      delay(1000);
      if (rssi < -80)
      {
        Serial.println("LED OFF");
        digitalWrite(14, LOW);
        break;
      }
      else
      {
        Serial.println("LED ON");
        digitalWrite(14, HIGH);
        break;
      }
    }
    else
    {
      Serial.println("Disconnected");
      paired = false;
      delay(5000);
      break;
    }
  }*/
}