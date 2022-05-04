/*
 * max30205_app.h
 *
 *  Created on: Jun 20, 2018
 *      Author: Mahir.Ozturk
 */

#ifndef MAX30205_FINGER_H_
#define MAX30205_FINGER_H_

#include "ble_gatt.h"

struct max30205_reader_task_args {
	I2C &i2cBus;
	GattAttribute::Handle_t gatt1;
	GattAttribute::Handle_t gatt2;
	int notify_period_sec;
};

void max30205_reader_task(struct max30205_reader_task_args *args);

#endif /* MAX30205_FINGER_H_ */
