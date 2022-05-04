/*
 * ble_comm.h
 *
 *  Created on: Jun 21, 2018
 *      Author: Mahir.Ozturk
 */

#ifndef BLE_COMM_H_
#define BLE_COMM_H_

#include <ble/BLE.h>

#define BLE_DESC_GATT_CPF_FORMAT_RFU		0x00 /**< Reserved For Future Use. */
#define BLE_DESC_GATT_CPF_FORMAT_BOOL		0x01 /**< Boolean. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT2		0x02 /**< Unsigned 2-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT4		0x03 /**< Unsigned 4-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT8		0x04 /**< Unsigned 8-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT12		0x05 /**< Unsigned 12-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT16		0x06 /**< Unsigned 16-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT24		0x07 /**< Unsigned 24-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT32		0x08 /**< Unsigned 32-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT48		0x09 /**< Unsigned 48-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT64		0x0A /**< Unsigned 64-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_UINT128	0x0B /**< Unsigned 128-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT8		0x0C /**< Signed 2-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT12		0x0D /**< Signed 12-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT16		0x0E /**< Signed 16-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT24		0x0F /**< Signed 24-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT32		0x10 /**< Signed 32-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT48		0x11 /**< Signed 48-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT64		0x12 /**< Signed 64-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_SINT128	0x13 /**< Signed 128-bit integer. */
#define BLE_DESC_GATT_CPF_FORMAT_FLOAT32	0x14 /**< IEEE-754 32-bit floating point. */
#define BLE_DESC_GATT_CPF_FORMAT_FLOAT64	0x15 /**< IEEE-754 64-bit floating point. */
#define BLE_DESC_GATT_CPF_FORMAT_SFLOAT		0x16 /**< IEEE-11073 16-bit SFLOAT. */
#define BLE_DESC_GATT_CPF_FORMAT_FLOAT		0x17 /**< IEEE-11073 32-bit FLOAT. */
#define BLE_DESC_GATT_CPF_FORMAT_DUINT16	0x18 /**< IEEE-20601 format. */
#define BLE_DESC_GATT_CPF_FORMAT_UTF8S		0x19 /**< UTF-8 string. */
#define BLE_DESC_GATT_CPF_FORMAT_UTF16S		0x1A /**< UTF-16 string. */
#define BLE_DESC_GATT_CPF_FORMAT_STRUCT		0x1B /**< Opaque Structure. */

typedef struct {
	uint8_t		format;
	int8_t		exponent;
	uint16_t	unit;
	uint8_t		name_space;
	uint16_t	desc;
} ble_desc_gatt_cpf_t;

ble_error_t bleGattAttrWrite(GattAttribute::Handle_t handle, const uint8_t *value, uint16_t size);

#endif /* BLE_COMM_H_ */
