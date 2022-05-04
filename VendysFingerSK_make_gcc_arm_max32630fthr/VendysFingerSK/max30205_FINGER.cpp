/*
 * max30205_app.c
 *
 *  Created on: Jun 20, 2018
 *      Author: Mahir.Ozturk
 */
#include <mbed.h>
#include "max30205_FINGER.h"
#include "MAX30205.h"
#include "USBSerial.h"

// Virtual serial port over USB
Serial uart(P2_1, P2_0);
USBSerial microUSB; 
//Serial daplink(USBTX, USBRX, 115200);

bool max30205_config(MAX30205 &temp_sensor){

	int rc = 0;

	MAX30205::Configuration_u temp_cfg;
	temp_cfg.all = 0;
	temp_cfg.bits.shutdown = 1;     // Shutdown mode
	temp_cfg.bits.comp_int = 1;     // Interrupt mode
	temp_cfg.bits.os_polarity = 0;  // Active low OS
	temp_cfg.bits.fault_queue = 1;  // Two faults for OS condition
	temp_cfg.bits.data_format = 0;  // Normal data format
	temp_cfg.bits.timeout = 0;      // I2C timeout reset enabled
	temp_cfg.bits.one_shot = 0;     // Start with one-shot = 0

	rc = temp_sensor.writeConfiguration(temp_cfg);  // Write config to MAX30205

	return rc;
}


void max30205_reader_task(struct max30205_reader_task_args *args)
{
	MAX30205 max30205_temp_sensor1(args->i2cBus, 0x48);	/* New MAX30205 on i2cBus */
	MAX30205 max30205_temp_sensor2(args->i2cBus, 0x4A);	/* New MAX30205 on i2cBus */
	
	int rc1 = max30205_config(max30205_temp_sensor1);   // Configure sensor, return 0 on success
	int rc2 = max30205_config(max30205_temp_sensor2);   // Configure sensor, return 0 on success

	MAX30205::Configuration_u temp1_cfg;
	uint16_t rawTemperatureRead1;
	float temperature1;

	temp1_cfg.all = 0;

	printf("Starting MAX30205 Temperature Demo Application...\r\n");
//	int rc2 = max30205_config(max30205_temp_sensor2);   // Configure sensor, return 0 on success

	MAX30205::Configuration_u temp2_cfg;
	uint16_t rawTemperatureRead2;

//	temp2_cfg.all = 0;

	printf("Starting MAX30205 Temperature Demo Application...\r\n");

	while (1) {
		if (rc1 == 0) {
			/* Send one-shot cmd to begin conversion */
			temp1_cfg.bits.one_shot = 1;
			rc1 = max30205_temp_sensor1.writeConfiguration(temp1_cfg);

			Thread::wait(50);

			/* Read the temperature data */
			rc1 = max30205_temp_sensor1.readTemperature(rawTemperatureRead1);
			/* Convert temp data to Celsius */
			temperature1 = max30205_temp_sensor1.toCelsius(rawTemperatureRead1);
			bleGattAttrWrite(args->gatt1, (uint8_t *)&rawTemperatureRead1, sizeof(rawTemperatureRead1));

			/* Send one-shot cmd to begin conversion */
			temp2_cfg.bits.one_shot = 1;
			rc2 = max30205_temp_sensor2.writeConfiguration(temp2_cfg);

			Thread::wait(50);

			/* Read the temperature data */
			rc2 = max30205_temp_sensor2.readTemperature(rawTemperatureRead2);
			/* Convert temp data to Celsius */
			float temperature2 = max30205_temp_sensor2.toCelsius(rawTemperatureRead2);
			bleGattAttrWrite(args->gatt2, (uint8_t *)&rawTemperatureRead2, sizeof(rawTemperatureRead2));
			printf("rawdata = %.2f \n",temperature2);
			Thread::wait(1000);

		} else {
			printf("Something went wrong, check the I2C bus and power connections...\r\n");

			return;
		}

		
	}
}

		

