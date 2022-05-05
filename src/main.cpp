/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>



//BLE server name
#define bleServerName "BME280_ESP32"

BLEServer *pServer;
bool deviceConnected = false;
bool once = false;
bool once2 = false;

#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"


//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};


void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
}

void loop() {
  if (deviceConnected) {
    if (!once) {
      Serial.println("Device connected.");
      once = true;
      once2 = false;
    }
  } else {
    if (!once2) {
      Serial.println("Waiting a client connection to notify...");
      pServer->getAdvertising()->start();
      once2 = true;
      once = false;
    }
  }
}
