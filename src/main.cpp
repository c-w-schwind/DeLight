#include <Arduino.h>
#include <BLEDevice.h> //Header file for BLE

static BLEUUID serviceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59"); 
static BLEUUID charUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");   
String My_BLE_Address = "08:3a:f2:b9:0b:fa";                       
static BLERemoteCharacteristic *pRemoteCharacteristic;

BLEScan *pBLEScan; // Name the scanning device as pBLEScan
BLEScanResults foundDevices;

BLEClient *pClient;
int rssi;

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
  Serial.println("ESP32 BLE Server program");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();                                           // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // Call the class that is defined above
  pBLEScan->setActiveScan(true);                                             // active scan uses more power, but get results faster

  pinMode(14, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(33, OUTPUT);
}

void loop()
{
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
        digitalWrite(26, LOW);
        digitalWrite(33, LOW);
        break;
      }
      else
      {
        Serial.println("LED ON");
        digitalWrite(14, HIGH);
        digitalWrite(26, HIGH);
        digitalWrite(33, HIGH);
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
  }
}