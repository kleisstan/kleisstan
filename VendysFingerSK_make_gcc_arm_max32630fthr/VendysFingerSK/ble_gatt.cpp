/*
 * ble_comm.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: Mahir.Ozturk
 */
/******************************************************************************/
#include <mbed.h>
#include "ble_gatt.h"

Mutex ble_mutex;

ble_error_t bleGattAttrWrite(GattAttribute::Handle_t handle, const uint8_t *value, uint16_t size)

{
	BLE &ble = BLE::Instance();
	ble_error_t ret;

	ble_mutex.lock();

	ret = ble.gattServer().write(handle, value, size);

	ble_mutex.unlock();

	return ret;
}
