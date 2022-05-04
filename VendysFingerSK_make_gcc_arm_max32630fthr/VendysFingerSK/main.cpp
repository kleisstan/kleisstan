/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include <mbed.h>
#include <events/mbed_events.h>
#include <rtos.h>
#include "config.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "max32630fthr.h"
#include "ble_gatt.h"
#include "hal/gpio_api.h"
#if defined(LIB_MAX30205)
#	include "max30205_FINGER.h"
#endif


/******************************************************************************/

MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);

InterruptIn button(P2_3);

SPI spim2(SPI2_MOSI, SPI2_MISO, SPI2_SCK);

I2C i2c1(I2C1_SDA, I2C1_SCL);		/* I2C bus, P3_4 = SDA, P3_5 = SCL */

/* LEDs */
DigitalOut rLED(LED1, LED_OFF);
DigitalOut gLED(LED2, LED_OFF);
DigitalOut bLED(LED3, LED_OFF);

DigitalOut BTen(BT_RST, 1);


/* Hardware serial port over DAPLink */
Serial daplink(USBTX, USBRX, 115200);

// Virtual serial port over USB

int alive_led_event_id;

/******************************************************************************/
const static char     DEVICE_NAME[] = MAXIM_PLATFORM_NAME;
//static const uint16_t uuid16_list[] = {0xFFFF}; //Custom UUID, FFFF is reserved for development

/* Set Up custom Characteristics */
UUID iot_service_uuid("4D6963736F636869702D524E34383730");

static ble_desc_gatt_cpf_t cpf_float32 = {.format = BLE_DESC_GATT_CPF_FORMAT_FLOAT32};
static ble_desc_gatt_cpf_t cpf_uint16 = {.format = BLE_DESC_GATT_CPF_FORMAT_UINT16};
static ble_desc_gatt_cpf_t cpf_uint8 = {.format = BLE_DESC_GATT_CPF_FORMAT_UINT8};

GattAttribute gatt_attr_cpf_format_float32(BLE_UUID_DESCRIPTOR_CHAR_PRESENTATION_FORMAT, (uint8_t *)&cpf_float32, sizeof(ble_desc_gatt_cpf_t));
GattAttribute gatt_attr_cpf_format_uint16(BLE_UUID_DESCRIPTOR_CHAR_PRESENTATION_FORMAT, (uint8_t *)&cpf_uint16, sizeof(ble_desc_gatt_cpf_t));
GattAttribute gatt_attr_cpf_format_uint8(BLE_UUID_DESCRIPTOR_CHAR_PRESENTATION_FORMAT, (uint8_t *)&cpf_uint8, sizeof(ble_desc_gatt_cpf_t));

UUID gatt_char_uuid_pushbutton("00001522-1d66-11e8-b467-0ed5f89f718b");
static uint8_t pushbutton_press_count = 0;
GattAttribute pushbutton_user_desc_descriptor(BLE_UUID_DESCRIPTOR_CHAR_USER_DESC, (uint8_t *)"Push Button", sizeof("Push Button"));
GattAttribute *pushbutton_descriptors[] = {&pushbutton_user_desc_descriptor, &gatt_attr_cpf_format_uint8};
GattCharacteristic gatt_char_pushbutton(gatt_char_uuid_pushbutton, &pushbutton_press_count, 1, 1,
										GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
										pushbutton_descriptors,
										sizeof(pushbutton_descriptors) / sizeof(GattAttribute*));

UUID gatt_char_uuid_led("00001523-1d66-11e8-b467-0ed5f89f718b");
static uint8_t led_init_value[] = {LED_OFF, LED_OFF, LED_OFF};
GattAttribute led_user_desc_descriptor(BLE_UUID_DESCRIPTOR_CHAR_USER_DESC, (uint8_t *)"LED", sizeof("LED"));
GattAttribute *led_descriptors[] = {&led_user_desc_descriptor};
ReadWriteArrayGattCharacteristic<uint8_t, sizeof(led_init_value)> gatt_char_led(gatt_char_uuid_led, led_init_value,
																				GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NONE,
																				led_descriptors,
																				sizeof(led_descriptors) / sizeof(GattAttribute*));


#if defined(LIB_MAX30205)
UUID gatt_char_uuid_FingerTemp("BF3FBD81063F11E59E690002A5D5C501");
static float FingerTemp_init_value;
GattAttribute FingerTemp_user_desc_descriptor(BLE_UUID_DESCRIPTOR_CHAR_USER_DESC, (uint8_t *)"PatchTemp", sizeof("PatchTemp"));
GattAttribute *FingerTemp_descriptors[] = {&FingerTemp_user_desc_descriptor, &gatt_attr_cpf_format_float32};
ReadOnlyGattCharacteristic<float> gatt_char_FingerTemp(gatt_char_uuid_FingerTemp,
												 &FingerTemp_init_value,
												 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
												 FingerTemp_descriptors,
												 sizeof(FingerTemp_descriptors) / sizeof(GattAttribute*));
UUID gatt_char_uuid_RoomTemp("BF3FBD81063F11E59E690002A5D5C502");
static float RoomTemp_init_value;
GattAttribute RoomTemp_user_desc_descriptor(BLE_UUID_DESCRIPTOR_CHAR_USER_DESC, (uint8_t *)"RoomTemp", sizeof("RoomTemp"));
GattAttribute *RoomTemp_descriptors[] = {&RoomTemp_user_desc_descriptor, &gatt_attr_cpf_format_float32};
ReadOnlyGattCharacteristic<float> gatt_char_RoomTemp(gatt_char_uuid_RoomTemp,
												 &RoomTemp_init_value,
												 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
												 RoomTemp_descriptors,
												 sizeof(RoomTemp_descriptors) / sizeof(GattAttribute*));
#endif

/* Set up custom service */
GattCharacteristic *characteristics[] = {//&gatt_char_led, &gatt_char_pushbutton,
#if defined(LIB_MAX30205)
										 &gatt_char_FingerTemp,&gatt_char_RoomTemp,
#endif
};

GattService iot_gatt_service(iot_service_uuid, characteristics, sizeof(characteristics) / sizeof(GattCharacteristic *));

/******************************************************************************/

static EventQueue eventQueue(/* event count */ 10 * /* event size */ 32);

void updateButtonState(uint8_t newState) {
	printf("Button pressed...\r\n");
	bleGattAttrWrite(gatt_char_pushbutton.getValueHandle(), (uint8_t *)&newState, sizeof(uint8_t));
}

void buttonPressedCallback(void)
{
	eventQueue.call(Callback<void(uint8_t)>(&updateButtonState), ++pushbutton_press_count);
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
	printf("disc\r\n");
	BLE::Instance().gap().startAdvertising(); // restart advertising
}

/* Connection */
void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{
	printf("succ\r\n");
}

void blinkCallback(void)
{
	gLED = !gLED;
}

void onBleInitError(BLE &ble, ble_error_t error)
{
	/* Initialization error handling should go here */
}

/**
 * This callback allows the LEDService to receive updates to the ledState Characteristic.
 *
 * @param[in] params
 *     Information about the characteristic being updated.
 */
void onDataWrittenCallback(const GattWriteCallbackParams *params)
{
	if ((params->handle == gatt_char_led.getValueHandle()) && (params->len >= 3)) {
		rLED = (params->data[0] != 0) ? LED_OFF : LED_ON;
		gLED = (params->data[1] != 0) ? LED_OFF : LED_ON;
		bLED = (params->data[2] != 0) ? LED_OFF : LED_ON;
	}
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
	BLE&        ble   = params->ble;
	ble_error_t error = params->error;

	if (error != BLE_ERROR_NONE) {
		/* In case of error, forward the error handling to onBleInitError */
		onBleInitError(ble, error);
		return;
	}

	/* Ensure that it is the default instance of BLE */
	if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
		return;
	}

	ble.gap().onDisconnection(disconnectionCallback);
	ble.gap().onConnection(connectionCallback);

	ble.gattServer().onDataWritten(onDataWrittenCallback);

	ble.gattServer().addService(iot_gatt_service);

	/* setup advertising */
	ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
	//ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
	ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
	ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
	ble.gap().setAdvertisingInterval(1000); /* 1000ms. */
	ble.gap().startAdvertising();

	button.fall(buttonPressedCallback);
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
	BLE &ble = BLE::Instance();
	eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

int main()
{
	osStatus status;
	rLED = LED_OFF; gLED = LED_OFF; bLED = LED_OFF;
	BTen = 1;

	alive_led_event_id = eventQueue.call_every(1000, blinkCallback);

	printf("Initializing BLE service...\r\n");

	BLE &ble = BLE::Instance();
	ble.onEventsToProcess(scheduleBleEventsProcessing);
	ble.init(bleInitComplete);

#if defined(LIB_MAX30205)
	Thread thread_max30205_reader;
	struct max30205_reader_task_args args_max30205 = {
			i2c1,
			gatt_char_FingerTemp.getValueHandle(),
			gatt_char_RoomTemp.getValueHandle(),
			MAX30205_BLE_NOTIFY_PERIOD_SEC};
	status = thread_max30205_reader.start(callback(max30205_reader_task, &args_max30205));
	if (status != osOK) {
		printf("Starting thread_max30205_reader thread failed(%ld)!\r\n", status);
	}
#endif


	eventQueue.dispatch_forever();

	return 0;
}

