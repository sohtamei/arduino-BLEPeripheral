// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#if defined(NRF52840)
#if defined(NRF5) || defined(NRF52_S140)
#include <ble.h>
#include <ble_hci.h>
#include <nrf_sdm.h>

#if defined(NRF52) || defined(NRF52_S140)
uint32_t sd_ble_gatts_value_set(uint16_t handle, uint16_t offset, uint16_t* const p_len, uint8_t const * const p_value) {
  ble_gatts_value_t val;

  val.len = *p_len;
  val.offset = offset;
  val.p_value = (uint8_t*)p_value;
  return sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, handle, &val);
}
#endif

#include "Arduino.h"

#include "BLEAttribute.h"
#include "BLEService.h"
#include "BLECharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEUtil.h"
#include "BLEUuid.h"

#include "nRF52840.h"

// #define NRF_52840_DEBUG

#if defined(NRF_52840_DEBUG)
#define PRINT_ERROR(RET_CODE)                   \
  do {                                          \
    const uint32_t print_ret_code = (RET_CODE); \
    if (print_ret_code != NRF_SUCCESS)          \
    {                                           \
      Serial.print("RetCode: ");                \
      Serial.println(print_ret_code);           \
    }                                           \
  } while (0)
#else
#define APP_ERROR_CHECK(ERR_CODE)   null
#endif

#define APP_BLE_CONN_CFG_TAG             1

#define BLE_STACK_EVT_MSG_BUF_SIZE       (sizeof(ble_evt_t) + (BLE_GATT_ATT_MTU_DEFAULT))

#ifndef NRF_SDH_BLE_TOTAL_LINK_COUNT
#define NRF_SDH_BLE_TOTAL_LINK_COUNT     1
#endif

#ifndef NRF_SDH_BLE_GAP_EVENT_LENGTH
#define NRF_SDH_BLE_GAP_EVENT_LENGTH     6
#endif

#define RAM_START                        0x20000000

nRF52840::nRF52840() :
  BLEDevice(),

  _advDataLen(0),
  _broadcastCharacteristic(NULL),

  _connectionHandle(BLE_CONN_HANDLE_INVALID),

  _txBufferCount(0),

  _numLocalCharacteristics(0),
  _localCharacteristicInfo(NULL),

  _numRemoteServices(0),
  _remoteServiceInfo(NULL),
  _remoteServiceDiscoveryIndex(0),
  _numRemoteCharacteristics(0),
  _remoteCharacteristicInfo(NULL),
  _remoteRequestInProgress(false)
{
#if defined(NRF5) || defined(NRF52_S140)
  this->_encKey = (ble_gap_enc_key_t*)&this->_bondData;
  memset(&this->_bondData, 0, sizeof(this->_bondData));
#else
  this->_authStatus = (ble_gap_evt_auth_status_t*)&this->_authStatusBuffer;
  memset(&this->_authStatusBuffer, 0, sizeof(this->_authStatusBuffer));
#endif
}

nRF52840::~nRF52840() {
  this->end();
}

void nRF52840::begin(unsigned char advertisementDataSize,
                      BLEEirData *advertisementData,
                      unsigned char scanDataSize,
                      BLEEirData *scanData,
                      BLELocalAttribute** localAttributes,
                      unsigned char numLocalAttributes,
                      BLERemoteAttribute** remoteAttributes,
                      unsigned char numRemoteAttributes)
{
  uint32_t ret;
#if defined(NRF5) && defined(S140)
  #if defined(USE_LFRC)
    nrf_clock_lf_cfg_t cfg = {
      .source        = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv       = 8, //16
      .rc_temp_ctiv  = 2,
      .accuracy = NRF_CLOCK_LF_ACCURACY_250_PPM
    };
  #elif defined(USE_LFSYNT)
    nrf_clock_lf_cfg_t cfg = {
      .source        = NRF_CLOCK_LF_SRC_SYNTH,
      .rc_ctiv       = 0,
      .rc_temp_ctiv  = 0,
      .accuracy = NRF_CLOCK_LF_ACCURACY_250_PPM
    };
  #else
    //default USE_LFXO
    nrf_clock_lf_cfg_t cfg = {
      .source        = NRF_CLOCK_LF_SRC_XTAL,
      .rc_ctiv       = 0,
      .rc_temp_ctiv  = 0,
      .accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM
    };
  #endif

   ret = sd_softdevice_enable(&cfg, this->faultHandler);
   PRINT_ERROR(ret);
#endif

#if defined(NRF5) && defined(S140)

  extern uint32_t __data_start__;
  uint32_t app_ram_base = (uint32_t) &__data_start__;

  ble_cfg_t ble_cfg;

  // Configure Custom UUIDS
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count      = 10;
  ret = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, app_ram_base);
  PRINT_ERROR(ret);

  // GAP Role Count
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
  ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
  ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
  ret = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, app_ram_base);
  PRINT_ERROR(ret);

  // GATTS TAB Size
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = 2000; // BLE_GATTS_ATTR_TAB_SIZE_MIN; // 248, _DEFAULT is 1408
  ret = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, app_ram_base);
  PRINT_ERROR(ret);

  // GAP Conn
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag                     = APP_BLE_CONN_CFG_TAG;
  ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = NRF_SDH_BLE_TOTAL_LINK_COUNT;
  ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = NRF_SDH_BLE_GAP_EVENT_LENGTH;
  ret = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, app_ram_base);
  PRINT_ERROR(ret);

  // GATT MTU
  memset(&ble_cfg, 0x00, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag                     = APP_BLE_CONN_CFG_TAG;
  ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu     = BLE_GATT_ATT_MTU_DEFAULT;
  ret = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, app_ram_base);
  PRINT_ERROR(ret);

  // GATTS Service changed
  memset(&ble_cfg, 0x00, sizeof(ble_cfg));
  ble_cfg.gatts_cfg.service_changed.service_changed = 0;
  ret = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &ble_cfg, app_ram_base);
  PRINT_ERROR(ret);

  uint32_t check_mem = app_ram_base;
  ret = sd_ble_enable(&app_ram_base);
  PRINT_ERROR(ret);

  if (app_ram_base > check_mem) {
    Serial.println("Insufficient RAM allocated for the SoftDevice");
    Serial.print("Change the RAM start (linker) from 0x");
    Serial.print(check_mem, HEX);
    Serial.print(" to 0x");
    Serial.println(app_ram_base, HEX);
    Serial.print("Maximum RAM size for application is 0x");
    Serial.println((RAM_START + (NRF_FICR->INFO.RAM * 1024)) - (app_ram_base), HEX);
  } else {
    if (app_ram_base != check_mem) {
      Serial.print("RAM start (linker) can be adjusted to 0x");
      Serial.println(app_ram_base, HEX);
      Serial.print("RAM size for application can be adjusted to 0x");
      Serial.println((RAM_START + (NRF_FICR->INFO.RAM * 1024)) - (app_ram_base), HEX);
    }
  }

#endif

#ifdef NRF_52840_DEBUG
  ble_version_t version;

  sd_ble_version_get(&version);
  Serial.print(F("Version: 0x"));
  Serial.print(version.version_number, HEX);
  Serial.print(F("-"));
  Serial.print(version.company_id, HEX);
  Serial.print(F("-"));
  Serial.print(version.subversion_number,HEX);
  Serial.println();
#endif

  ble_gap_conn_params_t gap_conn_params;
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));
  gap_conn_params.min_conn_interval = 40;  // in 1.25ms units
  gap_conn_params.max_conn_interval = 80;  // in 1.25ms unit
  gap_conn_params.slave_latency     = 0;
  gap_conn_params.conn_sup_timeout  = 4000 / 10; // in 10ms unit

  ret = sd_ble_gap_ppcp_set(&gap_conn_params);
  PRINT_ERROR(ret);

  _advHandle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; 

  memset(&_advParams, 0x00, sizeof(_advParams));

  _advParams.properties.type           = this->_connectable ? BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED : BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
  _advParams.p_peer_addr    = NULL;
  _advParams.filter_policy  = BLE_GAP_ADV_FP_ANY;
  _advParams.interval       = (this->_advertisingInterval * 16) / 10; // advertising interval (in units of 0.625 ms)
  _advParams.duration       = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED; // BLE_GAP_ADV_TIMEOUT_LIMITED_MAX;  // 
  _advParams.primary_phy    = BLE_GAP_PHY_1MBPS; // BLE_GAP_PHY_AUTO;
  _advParams.secondary_phy  = BLE_GAP_PHY_1MBPS;
  _advParams.scan_req_notification = 0;

  this->_advDataLen = 0;
  this->_scanRspLen = 0;

  // Advertise Flags 3
  this->_advData[this->_advDataLen + 0] = 2;          // Len 2
  this->_advData[this->_advDataLen + 1] = BLE_GAP_AD_TYPE_FLAGS;  
  this->_advData[this->_advDataLen + 2] = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  this->_advDataLen += 3;

  if (advertisementDataSize && advertisementData) {
    for (int i = 0; i < advertisementDataSize; i++) {
      this->_advData[this->_advDataLen + 0] = advertisementData[i].length + 1;
      this->_advData[this->_advDataLen + 1] = advertisementData[i].type;
      this->_advDataLen += 2;

      memcpy(&this->_advData[this->_advDataLen], advertisementData[i].data, advertisementData[i].length);
      this->_advDataLen += advertisementData[i].length;
    }
  }

  if (scanDataSize && scanData) {
    for (int i = 0; i < scanDataSize; i++) {
      this->_scanRsp[this->_scanRspLen + 0] = scanData[i].length + 1;
      this->_scanRsp[this->_scanRspLen + 1] = scanData[i].type;
      this->_scanRspLen += 2;

      memcpy(&this->_scanRsp[this->_scanRspLen], scanData[i].data, scanData[i].length);
      this->_scanRspLen += scanData[i].length;
    }
  }

  static ble_gap_adv_data_t m_adv_data =
  {
      .adv_data =
      {
          .p_data = _advData,
          .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
      },
      .scan_rsp_data =
      {
          .p_data = _scanRsp,
          .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

      }
  };

  ret = sd_ble_gap_adv_set_configure(&_advHandle, &m_adv_data, &_advParams);
  PRINT_ERROR(ret);

  ret = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
  PRINT_ERROR(ret);

  for (int i = 0; i < numLocalAttributes; i++) {
    BLELocalAttribute *localAttribute = localAttributes[i];

    if (localAttribute->type() == BLETypeCharacteristic) {
      this->_numLocalCharacteristics++;
    }
  }

  this->_numLocalCharacteristics -= 3; // 0x2a00, 0x2a01, 0x2a05

  this->_localCharacteristicInfo = (struct localCharacteristicInfo*)malloc(sizeof(struct localCharacteristicInfo) * this->_numLocalCharacteristics);

  unsigned char localCharacteristicIndex = 0;

  uint16_t handle = 0;
  BLEService *lastService = NULL;

  for (int i = 0; i < numLocalAttributes; i++) {
    BLELocalAttribute *localAttribute = localAttributes[i];
    BLEUuid uuid = BLEUuid(localAttribute->uuid());
    const unsigned char* uuidData = uuid.data();
    unsigned char value[255];

    ble_uuid_t nordicUUID;

    if (uuid.length() == 2) {
      nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
      nordicUUID.type = BLE_UUID_TYPE_BLE;
    } else {
      unsigned char uuidDataTemp[16];

      memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

      nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

      uuidDataTemp[13] = 0;
      uuidDataTemp[12] = 0;

      ret = sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
      PRINT_ERROR(ret);

    }

    if (localAttribute->type() == BLETypeService) {
      BLEService *service = (BLEService *)localAttribute;

      if (strcmp(service->uuid(), "1800") == 0 || strcmp(service->uuid(), "1801") == 0) {
        continue; // skip
      }

      ret = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &nordicUUID, &handle);
      PRINT_ERROR(ret);

      lastService = service;
    } else if (localAttribute->type() == BLETypeCharacteristic) {
      BLECharacteristic *characteristic = (BLECharacteristic *)localAttribute;

      if (strcmp(characteristic->uuid(), "2a00") == 0) {
        ble_gap_conn_sec_mode_t secMode;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode); // no security is needed

        ret = sd_ble_gap_device_name_set(&secMode, characteristic->value(), characteristic->valueLength());
        PRINT_ERROR(ret);

      } else if (strcmp(characteristic->uuid(), "2a01") == 0) {
        const uint16_t *appearance = (const uint16_t*)characteristic->value();

        ret = sd_ble_gap_appearance_set(*appearance);
        PRINT_ERROR(ret);

      } else if (strcmp(characteristic->uuid(), "2a05") == 0) {
        // do nothing
      } else {
        uint8_t properties = characteristic->properties() & 0xfe;
        uint16_t valueLength = characteristic->valueLength();

        this->_localCharacteristicInfo[localCharacteristicIndex].characteristic = characteristic;
        this->_localCharacteristicInfo[localCharacteristicIndex].notifySubscribed = false;
        this->_localCharacteristicInfo[localCharacteristicIndex].indicateSubscribed = false;
        this->_localCharacteristicInfo[localCharacteristicIndex].service = lastService;

        ble_gatts_char_md_t characteristicMetaData;
        ble_gatts_attr_md_t clientCharacteristicConfigurationMetaData;
        ble_gatts_attr_t    characteristicValueAttribute;
        ble_gatts_attr_md_t characteristicValueAttributeMetaData;

        memset(&characteristicMetaData, 0, sizeof(characteristicMetaData));

        memcpy(&characteristicMetaData.char_props, &properties, 1);

        characteristicMetaData.p_char_user_desc  = NULL;
        characteristicMetaData.p_char_pf         = NULL;
        characteristicMetaData.p_user_desc_md    = NULL;
        characteristicMetaData.p_cccd_md         = NULL;
        characteristicMetaData.p_sccd_md         = NULL;

        if (properties & (BLENotify | BLEIndicate)) {
          memset(&clientCharacteristicConfigurationMetaData, 0, sizeof(clientCharacteristicConfigurationMetaData));

          BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.read_perm);
          BLE_GAP_CONN_SEC_MODE_SET_OPEN(&clientCharacteristicConfigurationMetaData.write_perm);

          clientCharacteristicConfigurationMetaData.vloc = BLE_GATTS_VLOC_STACK;

          characteristicMetaData.p_cccd_md = &clientCharacteristicConfigurationMetaData;
        }

        memset(&characteristicValueAttributeMetaData, 0, sizeof(characteristicValueAttributeMetaData));

        if (properties & (BLERead | BLENotify | BLEIndicate)) {
          if (this->_bondStore) {
            BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&characteristicValueAttributeMetaData.read_perm);
          } else {
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.read_perm);
          }
        }

        if (properties & (BLEWriteWithoutResponse | BLEWrite)) {
          if (this->_bondStore) {
            BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&characteristicValueAttributeMetaData.write_perm);
          } else {
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&characteristicValueAttributeMetaData.write_perm);
          }
        }

        characteristicValueAttributeMetaData.vloc       = BLE_GATTS_VLOC_STACK;
        characteristicValueAttributeMetaData.rd_auth    = 0;
        characteristicValueAttributeMetaData.wr_auth    = 0;
        characteristicValueAttributeMetaData.vlen       = !characteristic->fixedLength();

        for (int j = (i + 1); j < numLocalAttributes; j++) {
          localAttribute = localAttributes[j];

          if (localAttribute->type() != BLETypeDescriptor) {
            break;
          }

          BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

          if (strcmp(descriptor->uuid(), "2901") == 0) {
            characteristicMetaData.p_char_user_desc        = (uint8_t*)descriptor->value();
            characteristicMetaData.char_user_desc_max_size = descriptor->valueLength();
            characteristicMetaData.char_user_desc_size     = descriptor->valueLength();
          } else if (strcmp(descriptor->uuid(), "2904") == 0) {
            characteristicMetaData.p_char_pf = (ble_gatts_char_pf_t *)descriptor->value();
          }
        }

        memset(&characteristicValueAttribute, 0, sizeof(characteristicValueAttribute));

        characteristicValueAttribute.p_uuid       = &nordicUUID;
        characteristicValueAttribute.p_attr_md    = &characteristicValueAttributeMetaData;
        characteristicValueAttribute.init_len     = valueLength;
        characteristicValueAttribute.init_offs    = 0;
        characteristicValueAttribute.max_len      = characteristic->valueSize();
        characteristicValueAttribute.p_value      = NULL;

        ret = sd_ble_gatts_characteristic_add(BLE_GATT_HANDLE_INVALID, &characteristicMetaData, &characteristicValueAttribute, &this->_localCharacteristicInfo[localCharacteristicIndex].handles);
        PRINT_ERROR(ret);

        if (valueLength) {
          for (int j = 0; j < valueLength; j++) {
            value[j] = (*characteristic)[j];
          }

          ret = sd_ble_gatts_value_set(this->_localCharacteristicInfo[localCharacteristicIndex].handles.value_handle, 0, &valueLength, value);
          PRINT_ERROR(ret);

        }

        localCharacteristicIndex++;
      }
    } else if (localAttribute->type() == BLETypeDescriptor) {
      BLEDescriptor *descriptor = (BLEDescriptor *)localAttribute;

      if (strcmp(descriptor->uuid(), "2901") == 0 ||
          strcmp(descriptor->uuid(), "2902") == 0 ||
          strcmp(descriptor->uuid(), "2903") == 0 ||
          strcmp(descriptor->uuid(), "2904") == 0) {
        continue; // skip
      }

      uint16_t valueLength = descriptor->valueLength();

      ble_gatts_attr_t descriptorAttribute;
      ble_gatts_attr_md_t descriptorMetaData;

      memset(&descriptorAttribute, 0, sizeof(descriptorAttribute));
      memset(&descriptorMetaData, 0, sizeof(descriptorMetaData));

      descriptorMetaData.vloc = BLE_GATTS_VLOC_STACK;
      descriptorMetaData.vlen = (valueLength == descriptor->valueLength()) ? 0 : 1;

      if (this->_bondStore) {
        BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&descriptorMetaData.read_perm);
      } else {
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&descriptorMetaData.read_perm);
      }

      descriptorAttribute.p_uuid    = &nordicUUID;
      descriptorAttribute.p_attr_md = &descriptorMetaData;
      descriptorAttribute.init_len  = valueLength;
      descriptorAttribute.max_len   = descriptor->valueLength();
      descriptorAttribute.p_value   = NULL;

      ret = sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &descriptorAttribute, &handle);
      PRINT_ERROR(ret);

      if (valueLength) {
        for (int j = 0; j < valueLength; j++) {
          value[j] = (*descriptor)[j];
        }

        ret = sd_ble_gatts_value_set(handle, 0, &valueLength, value);
        PRINT_ERROR(ret);

      }
    }
  }

  if ( numRemoteAttributes > 0) {
    numRemoteAttributes -= 2; // 0x1801, 0x2a05
  }

  for (int i = 0; i < numRemoteAttributes; i++) {
    BLERemoteAttribute *remoteAttribute = remoteAttributes[i];

    if (remoteAttribute->type() == BLETypeService) {
      this->_numRemoteServices++;
    } else if (remoteAttribute->type() == BLETypeCharacteristic) {
      this->_numRemoteCharacteristics++;
    }
  }

  this->_remoteServiceInfo = (struct remoteServiceInfo*)malloc(sizeof(struct remoteServiceInfo) * this->_numRemoteServices);
  this->_remoteCharacteristicInfo = (struct remoteCharacteristicInfo*)malloc(sizeof(struct remoteCharacteristicInfo) * this->_numRemoteCharacteristics);

  BLERemoteService *lastRemoteService = NULL;
  unsigned char remoteServiceIndex = 0;
  unsigned char remoteCharacteristicIndex = 0;

  for (int i = 0; i < numRemoteAttributes; i++) {
    BLERemoteAttribute *remoteAttribute = remoteAttributes[i];
    BLEUuid uuid = BLEUuid(remoteAttribute->uuid());
    const unsigned char* uuidData = uuid.data();

    ble_uuid_t nordicUUID;

    if (uuid.length() == 2) {
      nordicUUID.uuid = (uuidData[1] << 8) | uuidData[0];
      nordicUUID.type = BLE_UUID_TYPE_BLE;
    } else {
      unsigned char uuidDataTemp[16];

      memcpy(&uuidDataTemp, uuidData, sizeof(uuidDataTemp));

      nordicUUID.uuid = (uuidData[13] << 8) | uuidData[12];

      uuidDataTemp[13] = 0;
      uuidDataTemp[12] = 0;

      ret = sd_ble_uuid_vs_add((ble_uuid128_t*)&uuidDataTemp, &nordicUUID.type);
      PRINT_ERROR(ret);

    }

    if (remoteAttribute->type() == BLETypeService) {
      this->_remoteServiceInfo[remoteServiceIndex].service = lastRemoteService = (BLERemoteService *)remoteAttribute;
      this->_remoteServiceInfo[remoteServiceIndex].uuid = nordicUUID;

      memset(&this->_remoteServiceInfo[remoteServiceIndex].handlesRange, 0, sizeof(this->_remoteServiceInfo[remoteServiceIndex].handlesRange));

      remoteServiceIndex++;
    } else if (remoteAttribute->type() == BLETypeCharacteristic) {
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].characteristic = (BLERemoteCharacteristic *)remoteAttribute;
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].service = lastRemoteService;
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].uuid = nordicUUID;

      memset(&this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties, 0, sizeof(this->_remoteCharacteristicInfo[remoteCharacteristicIndex].properties));
      this->_remoteCharacteristicInfo[remoteCharacteristicIndex].valueHandle = 0;

      remoteCharacteristicIndex++;
    }
  }

  if (this->_bondStore && this->_bondStore->hasData()) {
#ifdef NRF_52840_DEBUG
    Serial.println(F("Restoring bond data"));
#endif
    this->_bondStore->getData(this->_bondData, 0, sizeof(this->_bondData));
  }

  this->startAdvertising();

}

void nRF52840::poll() {
  uint32_t   evtBuf[BLE_STACK_EVT_MSG_BUF_SIZE] __attribute__ ((__aligned__(BLE_EVT_PTR_ALIGNMENT)));
  uint16_t   evtLen = sizeof(evtBuf);
  ble_evt_t* bleEvt = (ble_evt_t*)evtBuf;

  if (sd_ble_evt_get((uint8_t*)evtBuf, &evtLen) == NRF_SUCCESS) {
    switch (bleEvt->header.evt_id) {
      case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt TX complete "));
        Serial.println(bleEvt->evt.gattc_evt.params.write_cmd_tx_complete.count);
#endif
        this->_txBufferCount += bleEvt->evt.gattc_evt.params.write_cmd_tx_complete.count;
        break;

      case BLE_GAP_EVT_CONNECTED:
#ifdef NRF_52840_DEBUG
        char address[18];

        BLEUtil::addressToString(bleEvt->evt.gap_evt.params.connected.peer_addr.addr, address);

        Serial.print(F("Evt Connected "));
        Serial.println(address);
#endif

        this->_connectionHandle = bleEvt->evt.gap_evt.conn_handle;
        this->_txBufferCount = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;

        if (this->_eventListener) {
          this->_eventListener->BLEDeviceConnected(*this, bleEvt->evt.gap_evt.params.connected.peer_addr.addr);
        }

        if (this->_minimumConnectionInterval >= BLE_GAP_CP_MIN_CONN_INTVL_MIN &&
            this->_maximumConnectionInterval <= BLE_GAP_CP_MAX_CONN_INTVL_MAX) {
          ble_gap_conn_params_t gap_conn_params;

          gap_conn_params.min_conn_interval = this->_minimumConnectionInterval;  // in 1.25ms units
          gap_conn_params.max_conn_interval = this->_maximumConnectionInterval;  // in 1.25ms unit
          gap_conn_params.slave_latency     = 0;
          gap_conn_params.conn_sup_timeout  = 4000 / 10; // in 10ms unit

          sd_ble_gap_conn_param_update(this->_connectionHandle, &gap_conn_params);
        }

        if (this->_numRemoteServices > 0) {
          sd_ble_gattc_primary_services_discover(this->_connectionHandle, 1, NULL);
        }
        break;

      case BLE_GAP_EVT_DISCONNECTED:
#ifdef NRF_52840_DEBUG
        Serial.println(F("Evt Disconnected"));
#endif
        this->_connectionHandle = BLE_CONN_HANDLE_INVALID;
        this->_txBufferCount = 0;

        for (int i = 0; i < this->_numLocalCharacteristics; i++) {
          struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

          localCharacteristicInfo->notifySubscribed = false;
          localCharacteristicInfo->indicateSubscribed = false;

          if (localCharacteristicInfo->characteristic->subscribed()) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceCharacteristicSubscribedChanged(*this, *localCharacteristicInfo->characteristic, false);
            }
          }
        }

        if (this->_eventListener) {
          this->_eventListener->BLEDeviceDisconnected(*this);
        }

        // clear remote handle info
        for (int i = 0; i < this->_numRemoteServices; i++) {
          memset(&this->_remoteServiceInfo[i].handlesRange, 0, sizeof(this->_remoteServiceInfo[i].handlesRange));
        }

        for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
          memset(&this->_remoteCharacteristicInfo[i].properties, 0, sizeof(this->_remoteCharacteristicInfo[i].properties));
          this->_remoteCharacteristicInfo[i].valueHandle = 0;
        }

        this->_remoteRequestInProgress = false;

        this->startAdvertising();
        break;

      case BLE_GAP_EVT_CONN_PARAM_UPDATE:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Conn Param Update 0x"));
        Serial.print(bleEvt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval, HEX);
        Serial.print(F(" 0x"));
        Serial.print(bleEvt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval, HEX);
        Serial.print(F(" 0x"));
        Serial.print(bleEvt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency, HEX);
        Serial.print(F(" 0x"));
        Serial.print(bleEvt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout, HEX);
        Serial.println();
#endif
        break;

      case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Sec Params Request "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.bond);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.mitm);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.lesc);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.keypress);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.io_caps);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.oob);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.min_key_size);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_params_request.peer_params.max_key_size);
        Serial.println();
#endif

        if (this->_bondStore && !this->_bondStore->hasData()) {
          // only allow bonding if bond store exists and there is no data

          ble_gap_sec_params_t gapSecParams;

          memset(&gapSecParams, 0x00, sizeof(ble_gap_sec_params_t));

          gapSecParams.kdist_own.enc = 1;

          gapSecParams.bond             = true;
          gapSecParams.mitm             = false;
          gapSecParams.io_caps          = BLE_GAP_IO_CAPS_NONE;
          gapSecParams.oob              = false;
          gapSecParams.min_key_size     = 7;
          gapSecParams.max_key_size     = 16;

          ble_gap_sec_keyset_t keyset;

          keyset.keys_peer.p_enc_key  = NULL;
          keyset.keys_peer.p_id_key   = NULL;
          keyset.keys_peer.p_sign_key = NULL;
          keyset.keys_own.p_enc_key   = this->_encKey;
          keyset.keys_own.p_id_key    = NULL;
          keyset.keys_own.p_sign_key  = NULL;

          sd_ble_gap_sec_params_reply(this->_connectionHandle, BLE_GAP_SEC_STATUS_SUCCESS, &gapSecParams, &keyset);
        } else {
          sd_ble_gap_sec_params_reply(this->_connectionHandle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        }
        break;

      case BLE_GAP_EVT_SEC_INFO_REQUEST:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Sec Info Request "));
        // Serial.print(bleEvt->evt.gap_evt.params.sec_info_request.peer_addr);
        // Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_info_request.master_id.ediv);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_info_request.enc_info);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_info_request.id_info);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.sec_info_request.sign_info);
        Serial.println();
#endif
        if (this->_encKey->master_id.ediv == bleEvt->evt.gap_evt.params.sec_info_request.master_id.ediv) {
          sd_ble_gap_sec_info_reply(this->_connectionHandle, &this->_encKey->enc_info, NULL, NULL);
        } else {
          sd_ble_gap_sec_info_reply(this->_connectionHandle, NULL, NULL, NULL);
        }
        break;

      case BLE_GAP_EVT_AUTH_STATUS:
#ifdef NRF_52840_DEBUG
        Serial.println(F("Evt Auth Status"));
        Serial.println(bleEvt->evt.gap_evt.params.auth_status.auth_status);
#endif
        if (BLE_GAP_SEC_STATUS_SUCCESS == bleEvt->evt.gap_evt.params.auth_status.auth_status) {

          if (this->_bondStore) {
#ifdef NRF_52840_DEBUG
            Serial.println(F("Storing bond data"));
#endif
            this->_bondStore->putData(this->_bondData, 0, sizeof(this->_bondData));
          }

          if (this->_eventListener) {
            this->_eventListener->BLEDeviceBonded(*this);
          }
        }
        break;

      case BLE_GAP_EVT_CONN_SEC_UPDATE:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Conn Sec Update "));
        Serial.print(bleEvt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.sm);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv);
        Serial.print(F(" "));
        Serial.print(bleEvt->evt.gap_evt.params.conn_sec_update.conn_sec.encr_key_size);
        Serial.println();
#endif
        break;

      case BLE_GAP_EVT_ADV_SET_TERMINATED:
#ifdef NRF_52840_DEBUG
        Serial.println(F("Evt Adv Terminated"));
#endif
      case BLE_GATTS_EVT_HVN_TX_COMPLETE:
#ifdef NRF_52840_DEBUG
        Serial.println(F("Evt Hvn Tx Complete"));
#endif
        break;

      case BLE_GATTS_EVT_WRITE: {
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Write, handle = "));
        Serial.println(bleEvt->evt.gatts_evt.params.write.handle, DEC);
        BLEUtil::printBuffer(bleEvt->evt.gatts_evt.params.write.data, bleEvt->evt.gatts_evt.params.write.len);
#endif

        uint16_t handle = bleEvt->evt.gatts_evt.params.write.handle;

        for (int i = 0; i < this->_numLocalCharacteristics; i++) {
          struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

          if (localCharacteristicInfo->handles.value_handle == handle) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceCharacteristicValueChanged(*this, *localCharacteristicInfo->characteristic, bleEvt->evt.gatts_evt.params.write.data, bleEvt->evt.gatts_evt.params.write.len);
            }
            break;
          } else if (localCharacteristicInfo->handles.cccd_handle == handle) {
            uint8_t* data  = &bleEvt->evt.gatts_evt.params.write.data[0];
            uint16_t value = data[0] | (data[1] << 8);

            localCharacteristicInfo->notifySubscribed = (value & 0x0001);
            localCharacteristicInfo->indicateSubscribed = (value & 0x0002);

            bool subscribed = (localCharacteristicInfo->notifySubscribed || localCharacteristicInfo->indicateSubscribed);

            if (subscribed != localCharacteristicInfo->characteristic->subscribed()) {
              if (this->_eventListener) {
                this->_eventListener->BLEDeviceCharacteristicSubscribedChanged(*this, *localCharacteristicInfo->characteristic, subscribed);
              }
              break;
            }
          }
        }
        break;
      }

      case BLE_GATTS_EVT_SYS_ATTR_MISSING:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Sys Attr Missing "));
        Serial.println(bleEvt->evt.gatts_evt.params.sys_attr_missing.hint);
#endif
        sd_ble_gatts_sys_attr_set(this->_connectionHandle, NULL, 0, 0);
        break;

      case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Prim Srvc Disc Rsp 0x"));
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
#endif
        if (bleEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS) {
          uint16_t count = bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.count;
          for (int i = 0; i < count; i++) {
            for (int j = 0; j < this->_numRemoteServices; j++) {
              if ((bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[i].uuid.type == this->_remoteServiceInfo[j].uuid.type) &&
                  (bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[i].uuid.uuid == this->_remoteServiceInfo[j].uuid.uuid)) {
                this->_remoteServiceInfo[j].handlesRange = bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[i].handle_range;
                break;
              }
            }
          }

          uint16_t startHandle = bleEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.services[count - 1].handle_range.end_handle + 1;

          sd_ble_gattc_primary_services_discover(this->_connectionHandle, startHandle, NULL);
        } else {
          // done discovering services
          for (int i = 0; i < this->_numRemoteServices; i++) {
            if (this->_remoteServiceInfo[i].handlesRange.start_handle != 0 && this->_remoteServiceInfo[i].handlesRange.end_handle != 0) {
              this->_remoteServiceDiscoveryIndex = i;

              sd_ble_gattc_characteristics_discover(this->_connectionHandle, &this->_remoteServiceInfo[i].handlesRange);
              break;
            }
          }
        }
        break;

      case BLE_GATTC_EVT_CHAR_DISC_RSP:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Char Disc Rsp 0x"));
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
#endif
        if (bleEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS) {
          ble_gattc_handle_range_t serviceHandlesRange = this->_remoteServiceInfo[this->_remoteServiceDiscoveryIndex].handlesRange;

          uint16_t count = bleEvt->evt.gattc_evt.params.char_disc_rsp.count;

          for (int i = 0; i < count; i++) {
            for (int j = 0; j < this->_numRemoteCharacteristics; j++) {
              if ((this->_remoteServiceInfo[this->_remoteServiceDiscoveryIndex].service == this->_remoteCharacteristicInfo[j].service) &&
                  (bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].uuid.type == this->_remoteCharacteristicInfo[j].uuid.type) &&
                  (bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].uuid.uuid == this->_remoteCharacteristicInfo[j].uuid.uuid)) {
                this->_remoteCharacteristicInfo[j].properties = bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].char_props;
                this->_remoteCharacteristicInfo[j].valueHandle = bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].handle_value;
              }
            }

            serviceHandlesRange.start_handle = bleEvt->evt.gattc_evt.params.char_disc_rsp.chars[i].handle_value;
          }

          sd_ble_gattc_characteristics_discover(this->_connectionHandle, &serviceHandlesRange);
        } else {
          bool discoverCharacteristics = false;

          for (int i = this->_remoteServiceDiscoveryIndex + 1; i < this->_numRemoteServices; i++) {
            if (this->_remoteServiceInfo[i].handlesRange.start_handle != 0 && this->_remoteServiceInfo[i].handlesRange.end_handle != 0) {
              this->_remoteServiceDiscoveryIndex = i;

              sd_ble_gattc_characteristics_discover(this->_connectionHandle, &this->_remoteServiceInfo[i].handlesRange);
              discoverCharacteristics = true;
              break;
            }
          }

          if (!discoverCharacteristics) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceRemoteServicesDiscovered(*this);
            }
          }
        }
        break;

      case BLE_GATTC_EVT_READ_RSP: {
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Read Rsp 0x"));
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
        Serial.println(bleEvt->evt.gattc_evt.params.read_rsp.handle, DEC);
        BLEUtil::printBuffer(bleEvt->evt.gattc_evt.params.read_rsp.data, bleEvt->evt.gattc_evt.params.read_rsp.len);
#endif
        this->_remoteRequestInProgress = false;

        if (bleEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION &&
            this->_bondStore) {
          ble_gap_sec_params_t gapSecParams;

          memset(&gapSecParams, 0x00, sizeof(ble_gap_sec_params_t));

          gapSecParams.kdist_own.enc = 1;

          gapSecParams.bond             = true;
          gapSecParams.mitm             = false;
          gapSecParams.io_caps          = BLE_GAP_IO_CAPS_NONE;
          gapSecParams.oob              = false;
          gapSecParams.min_key_size     = 7;
          gapSecParams.max_key_size     = 16;

          sd_ble_gap_authenticate(this->_connectionHandle, &gapSecParams);
        } else {
          uint16_t handle = bleEvt->evt.gattc_evt.params.read_rsp.handle;

          for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
            if (this->_remoteCharacteristicInfo[i].valueHandle == handle) {
              if (this->_eventListener) {
                this->_eventListener->BLEDeviceRemoteCharacteristicValueChanged(*this, *this->_remoteCharacteristicInfo[i].characteristic, bleEvt->evt.gattc_evt.params.read_rsp.data, bleEvt->evt.gattc_evt.params.read_rsp. len);
              }
              break;
            }
          }
        }
        break;
      }

      case BLE_GATTC_EVT_WRITE_RSP:
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Write Rsp 0x"));
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
        Serial.println(bleEvt->evt.gattc_evt.params.write_rsp.handle, DEC);
#endif
        this->_remoteRequestInProgress = false;

        if (bleEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION &&
            this->_bondStore) {
          ble_gap_sec_params_t gapSecParams;

          memset(&gapSecParams, 0x00, sizeof(ble_gap_sec_params_t));

          gapSecParams.kdist_own.enc = 1;

          gapSecParams.bond             = true;
          gapSecParams.mitm             = false;
          gapSecParams.io_caps          = BLE_GAP_IO_CAPS_NONE;
          gapSecParams.oob              = false;
          gapSecParams.min_key_size     = 7;
          gapSecParams.max_key_size     = 16;

          sd_ble_gap_authenticate(this->_connectionHandle, &gapSecParams);
        }
        break;

      case BLE_GATTC_EVT_HVX: {
#ifdef NRF_52840_DEBUG
        Serial.print(F("Evt Hvx 0x"));
        Serial.println(bleEvt->evt.gattc_evt.gatt_status, HEX);
        Serial.println(bleEvt->evt.gattc_evt.params.hvx.handle, DEC);
#endif
        uint16_t handle = bleEvt->evt.gattc_evt.params.hvx.handle;

        if (bleEvt->evt.gattc_evt.params.hvx.type == BLE_GATT_HVX_INDICATION) {
          sd_ble_gattc_hv_confirm(this->_connectionHandle, handle);
        }

        for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
          if (this->_remoteCharacteristicInfo[i].valueHandle == handle) {
            if (this->_eventListener) {
              this->_eventListener->BLEDeviceRemoteCharacteristicValueChanged(*this, *this->_remoteCharacteristicInfo[i].characteristic, bleEvt->evt.gattc_evt.params.read_rsp.data, bleEvt->evt.gattc_evt.params.read_rsp. len);
            }
            break;
          }
        }
        break;
      }

      default:
#ifdef NRF_52840_DEBUG
        Serial.print(F("bleEvt->header.evt_id = 0x"));
        Serial.print(bleEvt->header.evt_id, HEX);
        Serial.print(F(" "));
        Serial.println(bleEvt->header.evt_len);
#endif
        break;
    }
  }

  // sd_app_evt_wait();
}

void nRF52840::end() {
  sd_softdevice_disable();

  if (this->_remoteCharacteristicInfo) {
    free(this->_remoteCharacteristicInfo);
  }

  if (this->_remoteServiceInfo) {
    free(this->_remoteServiceInfo);
  }

  if (this->_localCharacteristicInfo) {
    free(this->_localCharacteristicInfo);
  }

  this->_numLocalCharacteristics = 0;
  this->_numRemoteServices = 0;
  this->_numRemoteCharacteristics = 0;
}

bool nRF52840::updateCharacteristicValue(BLECharacteristic& characteristic) {
  bool success = true;

  for (int i = 0; i < this->_numLocalCharacteristics; i++) {
    struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

    if (localCharacteristicInfo->characteristic == &characteristic) {
      if (&characteristic == this->_broadcastCharacteristic) {
        this->broadcastCharacteristic(characteristic);
      }

      uint16_t valueLength = characteristic.valueLength();

      sd_ble_gatts_value_set(localCharacteristicInfo->handles.value_handle, 0, &valueLength, characteristic.value());

      ble_gatts_hvx_params_t hvxParams;

      memset(&hvxParams, 0, sizeof(hvxParams));

      hvxParams.handle = localCharacteristicInfo->handles.value_handle;
      hvxParams.offset = 0;
      hvxParams.p_data = NULL;
      hvxParams.p_len  = &valueLength;

      if (localCharacteristicInfo->notifySubscribed) {
        if (this->_txBufferCount > 0) {
          this->_txBufferCount--;

          hvxParams.type = BLE_GATT_HVX_NOTIFICATION;

          sd_ble_gatts_hvx(this->_connectionHandle, &hvxParams);
        } else {
          success = false;
        }
      }

      if (localCharacteristicInfo->indicateSubscribed) {
        if (this->_txBufferCount > 0) {
          this->_txBufferCount--;

          hvxParams.type = BLE_GATT_HVX_INDICATION;

          sd_ble_gatts_hvx(this->_connectionHandle, &hvxParams);
        } else {
          success = false;
        }
      }
    }
  }

  return success;
}

bool nRF52840::broadcastCharacteristic(BLECharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numLocalCharacteristics; i++) {
    struct localCharacteristicInfo* localCharacteristicInfo = &this->_localCharacteristicInfo[i];

    if (localCharacteristicInfo->characteristic == &characteristic) {
      if (characteristic.properties() & BLEBroadcast && localCharacteristicInfo->service) {

        unsigned char advData[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
        unsigned char advDataLen = this->_advDataLen;

        // copy the existing advertisement data
        memcpy(advData, this->_advData, advDataLen);

        advDataLen += (4 + characteristic.valueLength());

        if (advDataLen <= BLE_GAP_ADV_SET_DATA_SIZE_MAX) {
          BLEUuid uuid = BLEUuid(localCharacteristicInfo->service->uuid());

          advData[this->_advDataLen + 0] = 3 + characteristic.valueLength();
          advData[this->_advDataLen + 1] = BLE_GAP_AD_TYPE_SERVICE_DATA;

          memcpy(&advData[this->_advDataLen + 2], uuid.data(), 2);
          memcpy(&advData[this->_advDataLen + 4], characteristic.value(), characteristic.valueLength());

          static ble_gap_adv_data_t m_adv_data =
          {
              .adv_data =
              {
                  .p_data = advData,
                  .len    = advDataLen
              },
              .scan_rsp_data =
              {
                  .p_data = NULL,
                  .len    = 0
              }
          };

          sd_ble_gap_adv_set_configure(&_advHandle, &m_adv_data, &_advParams);
          success = true;

          this->_broadcastCharacteristic = &characteristic;
        }
      }
      break;
    }
  }

  return success;
}

bool nRF52840::canNotifyCharacteristic(BLECharacteristic& /*characteristic*/) {
  return (this->_txBufferCount > 0);
}

bool nRF52840::canIndicateCharacteristic(BLECharacteristic& /*characteristic*/) {
  return (this->_txBufferCount > 0);
}

bool nRF52840::canReadRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      success = (this->_remoteCharacteristicInfo[i].valueHandle &&
                  this->_remoteCharacteristicInfo[i].properties.read &&
                  !this->_remoteRequestInProgress);
      break;
    }
  }

  return success;
}

bool nRF52840::readRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle && this->_remoteCharacteristicInfo[i].properties.read) {
        this->_remoteRequestInProgress = true;
        success = (sd_ble_gattc_read(this->_connectionHandle, this->_remoteCharacteristicInfo[i].valueHandle, 0) == NRF_SUCCESS);
      }
      break;
    }
  }

  return success;
}

bool nRF52840::canWriteRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle) {
        if (this->_remoteCharacteristicInfo[i].properties.write) {
          success = !this->_remoteRequestInProgress;
        } else if (this->_remoteCharacteristicInfo[i].properties.write_wo_resp) {
          success = (this->_txBufferCount > 0);
        }
      }
      break;
    }
  }

  return success;
}

bool nRF52840::writeRemoteCharacteristic(BLERemoteCharacteristic& characteristic, const unsigned char value[], unsigned char length) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle &&
                  (this->_remoteCharacteristicInfo[i].properties.write_wo_resp || this->_remoteCharacteristicInfo[i].properties.write) &&
                  (this->_txBufferCount > 0)) {

        ble_gattc_write_params_t writeParams;

        writeParams.write_op = (this->_remoteCharacteristicInfo[i].properties.write) ? BLE_GATT_OP_WRITE_REQ : BLE_GATT_OP_WRITE_CMD;
        writeParams.flags = 0;
        writeParams.handle = this->_remoteCharacteristicInfo[i].valueHandle;
        writeParams.offset = 0;
        writeParams.len = length;
        writeParams.p_value = (uint8_t*)value;

        this->_remoteRequestInProgress = true;

        this->_txBufferCount--;

        success = (sd_ble_gattc_write(this->_connectionHandle, &writeParams) == NRF_SUCCESS);
      }
      break;
    }
  }

  return success;
}

bool nRF52840::canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      success = (this->_remoteCharacteristicInfo[i].valueHandle &&
                (this->_remoteCharacteristicInfo[i].properties.notify || this->_remoteCharacteristicInfo[i].properties.indicate));
      break;
    }
  }

  return success;
}

bool nRF52840::subscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle &&
                  (this->_remoteCharacteristicInfo[i].properties.notify || this->_remoteCharacteristicInfo[i].properties.indicate)) {

        ble_gattc_write_params_t writeParams;

        uint16_t value = (this->_remoteCharacteristicInfo[i].properties.notify ? 0x0001 : 0x002);

        writeParams.write_op = BLE_GATT_OP_WRITE_REQ;
        writeParams.flags = 0;

        writeParams.handle = (this->_remoteCharacteristicInfo[i].valueHandle + 1); // don't discover descriptors for now
        writeParams.offset = 0;
        writeParams.len = sizeof(value);
        writeParams.p_value = (uint8_t*)&value;

        this->_remoteRequestInProgress = true;

        success = (sd_ble_gattc_write(this->_connectionHandle, &writeParams) == NRF_SUCCESS);
      }
      break;
    }
  }

  return success;
}

bool nRF52840::canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->canSubscribeRemoteCharacteristic(characteristic);
}

bool nRF52840::unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  bool success = false;

  for (int i = 0; i < this->_numRemoteCharacteristics; i++) {
    if (this->_remoteCharacteristicInfo[i].characteristic == &characteristic) {
      if (this->_remoteCharacteristicInfo[i].valueHandle &&
                  (this->_remoteCharacteristicInfo[i].properties.notify || this->_remoteCharacteristicInfo[i].properties.indicate)) {

        ble_gattc_write_params_t writeParams;

        uint16_t value = 0x0000;

        writeParams.write_op = BLE_GATT_OP_WRITE_REQ;
        writeParams.flags = 0;

        writeParams.handle = (this->_remoteCharacteristicInfo[i].valueHandle + 1); // don't discover descriptors for now
        writeParams.offset = 0;
        writeParams.len = sizeof(value);
        writeParams.p_value = (uint8_t*)&value;

        this->_remoteRequestInProgress = true;

        success = (sd_ble_gattc_write(this->_connectionHandle, &writeParams) == NRF_SUCCESS);
      }
      break;
    }
  }

  return success;
}

bool nRF52840::setTxPower(int txPower) {
  if (txPower <= -40) {
    txPower = -40;
  } else if (txPower <= -30) {
    txPower = -30;
  } else if (txPower <= -20) {
    txPower = -20;
  } else if (txPower <= -16) {
    txPower = -16;
  } else if (txPower <= -12) {
    txPower = -12;
  } else if (txPower <= -8) {
    txPower = -8;
  } else if (txPower <= -4) {
    txPower = -4;
  } else if (txPower <= 0) {
    txPower = 0;
  } else {
    txPower = 4;
  }

//  return (sd_ble_gap_tx_power_set(txPower) == NRF_SUCCESS);
  return (sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, _advHandle, 0) == NRF_SUCCESS);
}

void nRF52840::faultHandler(uint32_t id, uint32_t pc, uint32_t info) {
    Serial.println("*** nRF52840 SD faultHandler");
    PRINT_ERROR(id);
    NVIC_SystemReset();
}

void nRF52840::startAdvertising() {
#ifdef NRF_52840_DEBUG
  Serial.println(F("Start advertisement"));
#endif
  uint32_t ret;

  ret = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, _advHandle, 0);
  PRINT_ERROR(ret);
  ret = sd_ble_gap_adv_start(_advHandle, APP_BLE_CONN_CFG_TAG);
  PRINT_ERROR(ret);

}

void nRF52840::disconnect() {
  sd_ble_gap_disconnect(this->_connectionHandle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

void nRF52840::requestAddress() {
  ble_gap_addr_t gapAddress;

  sd_ble_gap_addr_get(&gapAddress);

  if (this->_eventListener) {
    this->_eventListener->BLEDeviceAddressReceived(*this, gapAddress.addr);
  }
}

void nRF52840::requestTemperature() {
  int32_t rawTemperature = 0;

  sd_temp_get(&rawTemperature);

  float temperature = rawTemperature / 4.0;

  if (this->_eventListener) {
    this->_eventListener->BLEDeviceTemperatureReceived(*this, temperature);
  }
}

void nRF52840::requestBatteryLevel() {
}

#endif

#endif
