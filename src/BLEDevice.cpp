/*
 * BLE.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: kolban
 */
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_bt.h>            // ESP32 BLE
#include <esp_bt_device.h>     // ESP32 BLE
#include <esp_bt_main.h>       // ESP32 BLE
#include <esp_gap_ble_api.h>   // ESP32 BLE
#include <esp_gatts_api.h>     // ESP32 BLE
#include <esp_gattc_api.h>     // ESP32 BLE
#include <esp_gatt_common_api.h>// ESP32 BLE
#include <esp_err.h>           // ESP32 ESP-IDF
#include <esp_log.h>           // ESP32 ESP-IDF
#include <map>                 // Part of C++ Standard library
#include <sstream>             // Part of C++ Standard library
#include <iomanip>             // Part of C++ Standard library

#include "BLEDevice.h"
#include "GeneralUtils.h"
#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif

static const char* LOG_TAG = "BLEDevice";

/**
 * Singletons for the BLEDevice.
 */
BLEServer* BLEDevice::m_pServer = nullptr;
bool       initialized          = false;   // Have we been initialized?
esp_ble_sec_act_t 	BLEDevice::m_securityLevel = (esp_ble_sec_act_t)0;
uint16_t   BLEDevice::m_localMTU = 23;


/**
 * @brief Create a new instance of a server.
 * @return A new instance of the server.
 */
/* STATIC */ BLEServer* BLEDevice::createServer() {
	ESP_LOGD(LOG_TAG, ">> createServer");
#ifndef CONFIG_GATTS_ENABLE  // Check that BLE GATTS is enabled in make menuconfig
	ESP_LOGE(LOG_TAG, "BLE GATTS is not enabled - CONFIG_GATTS_ENABLE not defined");
	abort();
#endif // CONFIG_GATTS_ENABLE
	m_pServer = new BLEServer();
	m_pServer->createApp(0);
	ESP_LOGD(LOG_TAG, "<< createServer");
	return m_pServer;
} // createServer


/**
 * @brief Handle GATT server events.
 *
 * @param [in] event The event that has been newly received.
 * @param [in] gatts_if The connection to the GATT interface.
 * @param [in] param Parameters for the event.
 */
/* STATIC */ void BLEDevice::gattServerEventHandler(
   esp_gatts_cb_event_t      event,
   esp_gatt_if_t             gatts_if,
   esp_ble_gatts_cb_param_t* param
) {

	switch(event) {
		case ESP_GATTS_CONNECT_EVT: {
			BLEDevice::m_localMTU = 23;
#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
			if(BLEDevice::m_securityLevel){
				esp_ble_set_encryption(param->connect.remote_bda, BLEDevice::m_securityLevel);
			}
#endif	// CONFIG_BLE_SMP_ENABLE
			break;
		} // ESP_GATTS_CONNECT_EVT

		case ESP_GATTS_MTU_EVT: {
			BLEDevice::m_localMTU = param->mtu.mtu;
	        ESP_LOGI(LOG_TAG, "ESP_GATTS_MTU_EVT, MTU %d", BLEDevice::m_localMTU);
	        break;
		}
		default: {
			break;
		}
	} // switch


	if (BLEDevice::m_pServer != nullptr) {
		BLEDevice::m_pServer->handleGATTServerEvent(event, gatts_if, param);
	}
} // gattServerEventHandler


/**
 * @brief Handle GAP events.
 */
/* STATIC */ void BLEDevice::gapEventHandler(
	esp_gap_ble_cb_event_t event,
	esp_ble_gap_cb_param_t *param) {

	switch(event) {

		 case ESP_GAP_BLE_OOB_REQ_EVT:                                /* OOB request event */
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
			 break;
		 case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
			 break;
		 case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
			 break;
		 case ESP_GAP_BLE_NC_REQ_EVT:
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_NC_REQ_EVT");
#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
			if(BLEDevice::m_securityCallbacks!=nullptr){
			 	 esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, BLEDevice::m_securityCallbacks->onConfirmPIN(param->ble_security.key_notif.passkey));
			}
#endif	// CONFIG_BLE_SMP_ENABLE
			 break;
		 case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
			ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT: ");
			// esp_log_buffer_hex(LOG_TAG, m_remote_bda, sizeof(m_remote_bda));
#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
			if(BLEDevice::m_securityCallbacks!=nullptr){
				esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, BLEDevice::m_securityCallbacks->onPassKeyRequest());
			}
#endif	// CONFIG_BLE_SMP_ENABLE
			break;
			/*
			 * TODO should we add white/black list comparison?
			 */
		 case ESP_GAP_BLE_SEC_REQ_EVT:
			 /* send the positive(true) security response to the peer device to accept the security request.
			 If not accept the security request, should sent the security response with negative(false) accept value*/
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
			if(BLEDevice::m_securityCallbacks!=nullptr){
				esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, BLEDevice::m_securityCallbacks->onSecurityRequest());
			}
			else{
				esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
			}
#endif	// CONFIG_BLE_SMP_ENABLE
			break;
			 /*
			  *
			  */
		 case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
			 ///show the passkey number to the user to input it in the peer deivce.
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_PASSKEY_NOTIF_EVT");
#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
			if(BLEDevice::m_securityCallbacks!=nullptr){
				ESP_LOGI(LOG_TAG, "passKey = %d", param->ble_security.key_notif.passkey);
				BLEDevice::m_securityCallbacks->onPassKeyNotify(param->ble_security.key_notif.passkey);
			}
#endif	// CONFIG_BLE_SMP_ENABLE
			 break;
		 case ESP_GAP_BLE_KEY_EVT:
			 //shows the ble key type info share with peer device to the user.
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_KEY_EVT");
#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
			 ESP_LOGI(LOG_TAG, "key type = %s", BLESecurity::esp_key_type_to_str(param->ble_security.ble_key.key_type));
#endif	// CONFIG_BLE_SMP_ENABLE
			 break;
		 case ESP_GAP_BLE_AUTH_CMPL_EVT:
			 ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT");
#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
			 if(BLEDevice::m_securityCallbacks!=nullptr){
				 BLEDevice::m_securityCallbacks->onAuthenticationComplete(param->ble_security.auth_cmpl);
			 }
#endif	// CONFIG_BLE_SMP_ENABLE
			 break;
		default: {
			break;
		}
	} // switch

	if (BLEDevice::m_pServer != nullptr) {
		BLEDevice::m_pServer->handleGAPEvent(event, param);
	}

	/*
	 * Security events:
	 */


} // gapEventHandler



/**
 * @brief Initialize the %BLE environment.
 * @param deviceName The device name of the device.
 */
/* STATIC */ void BLEDevice::init(std::string deviceName) {
	if(!initialized){
		initialized = true;   // Set the initialization flag to ensure we are only initialized once.

		esp_err_t errRc = ::nvs_flash_init();
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "nvs_flash_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}

	  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	  errRc = esp_bt_controller_init(&bt_cfg);
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_bt_controller_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}

#ifndef CLASSIC_BT_ENABLED
	//	esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);  //FIXME waiting for response from esp-idf issue
		errRc = esp_bt_controller_enable(ESP_BT_MODE_BLE);
		//errRc = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_bt_controller_enable: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}
#else
		errRc = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_bt_controller_enable: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}
#endif

		errRc = esp_bluedroid_init();
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_bluedroid_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}

		errRc = esp_bluedroid_enable();
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_bluedroid_enable: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}

		errRc = esp_ble_gap_register_callback(BLEDevice::gapEventHandler);
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_ble_gap_register_callback: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}

#ifdef CONFIG_GATTC_ENABLE   // Check that BLE client is configured in make menuconfig
		errRc = esp_ble_gattc_register_callback(BLEDevice::gattClientEventHandler);
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_ble_gattc_register_callback: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}
#endif   // CONFIG_GATTC_ENABLE

#ifdef CONFIG_GATTS_ENABLE  // Check that BLE server is configured in make menuconfig
		errRc = esp_ble_gatts_register_callback(BLEDevice::gattServerEventHandler);
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_ble_gatts_register_callback: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		}
#endif   // CONFIG_GATTS_ENABLE

		errRc = ::esp_ble_gap_set_device_name(deviceName.c_str());
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_ble_gap_set_device_name: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		};

#ifdef CONFIG_BLE_SMP_ENABLE   // Check that BLE SMP (security) is configured in make menuconfig
		esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
		errRc = ::esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
		if (errRc != ESP_OK) {
			ESP_LOGE(LOG_TAG, "esp_ble_gap_set_security_param: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			return;
		};
#endif // CONFIG_BLE_SMP_ENABLE
	}
	vTaskDelay(200/portTICK_PERIOD_MS); // Delay for 200 msecs as a workaround to an apparent Arduino environment issue.
} // init



/*
 * @brief Get local MTU value set during mtu request or default value
 */
uint16_t BLEDevice::getMTU() {
	return m_localMTU;
}
#endif // CONFIG_BT_ENABLED
