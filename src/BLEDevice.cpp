// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "Arduino.h"

#include "BLEDevice.h"

#define UNIT_1_25_MS 1250
#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

#define DEFAULT_ADVERTISING_INTERVAL 100
#define DEFAULT_CONNECTABLE          true

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS) // 0.5s
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS) // 1.0s

BLEDevice::BLEDevice() :
  _advertisingInterval(DEFAULT_ADVERTISING_INTERVAL),
  _minimumConnectionInterval(0),//MIN_CONN_INTERVAL),
  _maximumConnectionInterval(0),//MAX_CONN_INTERVAL),
  _connectable(DEFAULT_CONNECTABLE),
  _bondStore(NULL),
  _eventListener(NULL)
{
}

BLEDevice::~BLEDevice() {
}

void BLEDevice::setEventListener(BLEDeviceEventListener* eventListener) {
  this->_eventListener = eventListener;
}

void BLEDevice::setAdvertisingInterval(unsigned short advertisingInterval) {
  this->_advertisingInterval = advertisingInterval;
}

void BLEDevice::setConnectionInterval(unsigned short minimumConnectionInterval, unsigned short maximumConnectionInterval) {
  if (maximumConnectionInterval < minimumConnectionInterval) {
    maximumConnectionInterval = minimumConnectionInterval;
  }

  this->_minimumConnectionInterval = minimumConnectionInterval;
  this->_maximumConnectionInterval = maximumConnectionInterval;
}

void BLEDevice::setConnectable(bool connectable) {
  this->_connectable = connectable;
}

void BLEDevice::setBondStore(BLEBondStore& bondStore) {
  this->_bondStore = &bondStore;
}
