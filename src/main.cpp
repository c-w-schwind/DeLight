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

static BLEUUID serviceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59"); // Service UUID of fitnessband obtained through nRF connect application
static BLEUUID charUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");    // Characteristic  UUID of fitnessband obtained through nRF connect application
String My_BLE_Address = "b4:e6:2d:eb:0b:df";                        // Hardware Bluetooth MAC of my fitnessband, will vary for every band obtained through nRF connect application
static BLERemoteCharacteristic *pRemoteCharacteristic;

BLEScan *pBLEScan; // Name the scanning device as pBLEScan
BLEScanResults foundDevices;

BLEClient *pClient;
int rssi;

static BLEAddress *Server_BLE_Address;
String Scanned_BLE_Address;

boolean paired = false; // boolean variable to togge light

bool connectToServer(BLEAddress pAddress) {

  *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  // Connect to the BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to lamp");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService != nullptr) {
    Serial.println(" - Found our service");
    return true;
  } else
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
  Serial.println("ESP32 BLE Server program");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();                                           // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // Call the class that is defined above
  pBLEScan->setActiveScan(true);                                             // active scan uses more power, but get results faster

  pinMode(14, OUTPUT); // Declare the in-built LED pin as output
}

void loop()
{
  while (paired == false) {
    foundDevices = pBLEScan->start(10); // Scan for 3 seconds to find the Fitness band
    Serial.printf("Scan done.\n");
    Serial.printf("Number of BLE devices found: %d\n", foundDevices.getCount());
    
    for (int i = 0; i < foundDevices.getCount(); i++) {
      Scanned_BLE_Address = foundDevices.getDevice(i).getAddress().toString().c_str();
      if (Scanned_BLE_Address == My_BLE_Address && paired == false) {
        Serial.println("Found Device. Connecting to Server as client");
        if (connectToServer(foundDevices.getDevice(i).getAddress())) {
          paired = true;
          Serial.println("********************LED turned ON************************");
          digitalWrite(14, HIGH);
          break;
        } else {
          Serial.println("Pairing failed");
          break;
        }
      }
    }
  }
  while (paired) {
    rssi = pClient->getRssi();
    Serial.println("RSSI: " + rssi);

    /*
    if (rssi < 10) {
      digitalWrite(14, LOW);
      paired = false;
    }
    */
  }
}