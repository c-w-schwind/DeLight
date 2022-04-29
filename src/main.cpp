/*
 * Program to operate ESP32 in client mode and use fitness band as proximity switch
 * Program by: Aswinth Raj B
 * Dated: 31-10-2018
 * Website: www.circuitdigest.com&nbsp;
 * Reference: https://github.com/nkolban/esp32-snippets&nbsp;
 * //NOTE: The My_BLE_Address, serviceUUID and charUUID should be changed based on the BLe server you are using 
 */

#include <Arduino.h>
#include <BLEDevice.h> //Header file for BLE 

static BLEUUID serviceUUID("0000fee7-0000-1000-8000-00805f9b34fb"); //Service UUID of fitnessband obtained through nRF connect application 
static BLEUUID    charUUID("0000fee7-0000-1000-8000-00805f9b34fb"); //Characteristic  UUID of fitnessband obtained through nRF connect application 
String My_BLE_Address = "xxxx"; //Hardware Bluetooth MAC of my fitnessband, will vary for every band obtained through nRF connect application 
static BLERemoteCharacteristic* pRemoteCharacteristic;

BLEScan* pBLEScan; //Name the scanning device as pBLEScan
BLEScanResults foundDevices;

static BLEAddress *Server_BLE_Address;
String Scaned_BLE_Address;

boolean paired = false; //boolean variable to togge light

 

bool connectToServer (BLEAddress pAddress){
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to fitnessband");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService != nullptr){
      Serial.println(" - Found our service");
      return true;
    } else
      return false;
    // ????
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic != nullptr)
      Serial.println(" - Found our characteristic");

      return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks 
{
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Scan Result: %s \n", advertisedDevice.toString().c_str());
      Server_BLE_Address = new BLEAddress(advertisedDevice.getAddress());
      
      Scaned_BLE_Address = Server_BLE_Address->toString().c_str();
      
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 BLE Server program");

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); //Call the class that is defined above 
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster

    pinMode (14,OUTPUT); //Declare the in-built LED pin as output 
}

void loop() {

  foundDevices = pBLEScan->start(3); //Scan for 3 seconds to find the Fitness band 

  while (foundDevices.getCount() >= 1)
  {
    if (Scaned_BLE_Address == My_BLE_Address && paired == false)
    {
      Serial.println("Found Device. Connecting to Server as client");
      if (connectToServer(*Server_BLE_Address)) {
        paired = true;
        Serial.println("********************LED turned ON************************");
        digitalWrite (14,HIGH);
        break;
      } else {
        Serial.println("Pairing failed");
        break;
      }
    }
    
    if (Scaned_BLE_Address == My_BLE_Address && paired == true)
    {
      Serial.println("Our device went out of range");
      paired = false;
      Serial.println("********************LED OOOFFFFF************************");
      digitalWrite (14,LOW);
      ESP.restart();
      break;
    }
    else
    {
    Serial.println("We have some other BLE device in range");
    break;
    }
  } 
}

/*
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setup() {
  Serial.begin(115200);

  BLEDevice::init("BLE EXPERT GROUP");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World says Kristof");
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void loop() {
  delay(2000);
}
*/