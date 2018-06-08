/*
 * BLEDevice.h
 *
 *  Created on: Mar 16, 2017
 *      Author: kolban
 */

#ifndef MAIN_BLEDevice_H_
#define MAIN_BLEDevice_H_
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)
#include <esp_gap_ble_api.h> // ESP32 BLE
#include <esp_gattc_api.h>   // ESP32 BLE
#include <map>               // Part of C++ STL
#include <string>
#include <esp_bt.h>

#include "BLEServer.h"

/**
 * @brief %BLE functions.
 */
class BLEDevice {
public:

	static BLEServer*  createServer();    // Cretae a new BLE server.
	static void        init(std::string deviceName);   // Initialize the local BLE environment.
	static uint16_t	   getMTU();

private:
	static BLEServer *m_pServer;
	static esp_ble_sec_act_t 	m_securityLevel;
	static uint16_t		m_localMTU;

	static esp_gatt_if_t getGattcIF();

	static void gattServerEventHandler(
	   esp_gatts_cb_event_t      event,
	   esp_gatt_if_t             gatts_if,
	   esp_ble_gatts_cb_param_t* param);

	static void gapEventHandler(
		esp_gap_ble_cb_event_t  event,
		esp_ble_gap_cb_param_t* param);

}; // class BLE

#endif // CONFIG_BT_ENABLED
#endif /* MAIN_BLEDevice_H_ */
