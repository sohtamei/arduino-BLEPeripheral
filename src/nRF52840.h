// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef _NRF_52840_H_
#define _NRF_52840_H_

#if defined(NRF52_S140) // || defined(NRF51_S130)
#include <ble_gatts.h>
#include <ble_gattc.h>
#include <nrf_soc.h>

#include "BLEDevice.h"

class nRF52840 : public BLEDevice
{
  friend class BLEPeripheral;

  protected:
    struct localCharacteristicInfo {
      BLECharacteristic* characteristic;
      BLEService* service;

      ble_gatts_char_handles_t handles;
      bool notifySubscribed;
      bool indicateSubscribed;
    };

    struct remoteServiceInfo {
      BLERemoteService* service;

      ble_uuid_t uuid;
      ble_gattc_handle_range_t handlesRange;
    };

    struct remoteCharacteristicInfo {
      BLERemoteCharacteristic* characteristic;
      BLERemoteService* service;

      ble_uuid_t uuid;
      ble_gatt_char_props_t properties;
      uint16_t valueHandle;
    };

    nRF52840();

    virtual ~nRF52840();

    virtual void begin(unsigned char advertisementDataSize,
                BLEEirData *advertisementData,
                unsigned char scanDataSize,
                BLEEirData *scanData,
                BLELocalAttribute** localAttributes,
                unsigned char numLocalAttributes,
                BLERemoteAttribute** remoteAttributes,
                unsigned char numRemoteAttributes);

    virtual void poll();

    virtual void end();

    virtual bool setTxPower(int8_t txPower);
#if defined(NRF52_S140)      
    virtual boolean setConnectedTxPower(int8_t txPower);
    virtual boolean setAdvertisingTxPower(int8_t txPower);
#endif    
    virtual void startAdvertising();
    virtual void disconnect();

    virtual bool updateCharacteristicValue(BLECharacteristic& characteristic);
    virtual bool broadcastCharacteristic(BLECharacteristic& characteristic);
    virtual bool canNotifyCharacteristic(BLECharacteristic& characteristic);
    virtual bool canIndicateCharacteristic(BLECharacteristic& characteristic);

    virtual bool canReadRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool readRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool canWriteRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool writeRemoteCharacteristic(BLERemoteCharacteristic& characteristic, const unsigned char value[], unsigned char length);
    virtual bool canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool subscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    virtual bool unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);

    virtual void requestAddress();
    virtual void requestTemperature();
    virtual void requestBatteryLevel();

  private:

    unsigned char                     _advData[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    unsigned char                     _scanRsp[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    unsigned char                     _advHandle;
    ble_gap_adv_params_t              _advParams;
    unsigned char                     _advDataLen;
    unsigned char                     _scanRspLen;
    BLECharacteristic*                _broadcastCharacteristic;

    uint16_t                          _connectionHandle;
    uint8_t                           _bondData[((sizeof(ble_gap_enc_key_t) + 3) / 4) * 4]  __attribute__ ((__aligned__(4)));
    ble_gap_enc_key_t*                _encKey;
    unsigned char                     _txBufferCount;

    unsigned char                     _numLocalCharacteristics;
    struct localCharacteristicInfo*   _localCharacteristicInfo;

    unsigned char                     _numRemoteServices;
    struct remoteServiceInfo*         _remoteServiceInfo;
    unsigned char                     _remoteServiceDiscoveryIndex;
    unsigned char                     _numRemoteCharacteristics;
    struct remoteCharacteristicInfo*  _remoteCharacteristicInfo;
    bool                              _remoteRequestInProgress;
    int8_t                            _txPower;
    static void faultHandler(uint32_t id, uint32_t pc, uint32_t info);
};

#endif
#endif
