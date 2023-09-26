#include "Particle.h"
#include "bike_canbus.h"
#include "bcycle_ble.h"

BCycleBLE *BCycleBLE::_instance = nullptr;

// From the BBT firmware
#define DEVICE_NAME             "BBT_T1"    /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME       "BCycle"    /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL        300         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION        18000       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define BCYCLE_APP_VERSION      "4.0.0"     // Made this up — BBT firmware I have on hand is v3.0.0

// BBT code uses this as a vendor-specific base UUID, using a call to sd_ble_uuid_vs_add()
// We need to use this in the BleUuid calls as a second argument, otherwise the Particle module's base UUID will be used
// Bytes 12 and 13 (0x00) are used to represent the 16-bit UUIDs of the characteristics associated with the service
#define TREK_BLE_BASE_UUID          {0xca, 0x89, 0x16, 0xa4, 0x2b, 0x47, 0x11, 0xe7, 0x93, 0xae, 0x92, 0x36, 0x00, 0x00, 0x26, 0x71}
#define GENERIC_SERVICE_UUID        0x3000

// BBT Characteristics
#define BATTERY_CHAR_UUID       0x3001  // Notify, read
#define ODOMETER_CHAR_UUID      0x3002  // Notify, read
#define SERNUM_CHAR_UUID        0x3003  // Read
#define FWVERSION_CHAR_UUID     0x3004  // Read
#define CONTROL_CHAR_UUID       0x3005  // Notify, read, write (unimplemented for now)

// Set up our service and characteristc UUIDs
const uint8_t BASE_UUID[BLE_SIG_UUID_128BIT_LEN] = TREK_BLE_BASE_UUID;
BleUuid genericServiceUUID  (BASE_UUID, GENERIC_SERVICE_UUID);
BleUuid charBatteryUUID     (BASE_UUID, BATTERY_CHAR_UUID);
BleUuid charOdometerUUID    (BASE_UUID, ODOMETER_CHAR_UUID);
BleUuid charSerNumUUID      (BASE_UUID, SERNUM_CHAR_UUID);
BleUuid charFwVerUUID       (BASE_UUID, FWVERSION_CHAR_UUID);

// Set up characteristics
BleCharacteristic charBattery   ("Battery",         BleCharacteristicProperty::READ | BleCharacteristicProperty::NOTIFY, charBatteryUUID,  genericServiceUUID);
BleCharacteristic charOdometer  ("Odometer",        BleCharacteristicProperty::READ | BleCharacteristicProperty::NOTIFY, charOdometerUUID, genericServiceUUID);
BleCharacteristic charSerNum    ("Serial Number",   BleCharacteristicProperty::READ,                                     charSerNumUUID,   genericServiceUUID);
BleCharacteristic charFwVer     ("fw version",      BleCharacteristicProperty::READ,                                     charFwVerUUID,    genericServiceUUID);

void BCycleBLE::setup() {

    char sernumstring[30] = { 0 };
	sprintf(sernumstring, "%lu%lu", NRF_FICR->DEVICEID[0], NRF_FICR->DEVICEID[1]);
    Log.info("BLE Serial Number = \"%s\" (len=%d)", sernumstring, strlen(sernumstring));

    BLE.selectAntenna(BleAntennaType::EXTERNAL);

    BLE.addCharacteristic(charBattery);
    BLE.addCharacteristic(charOdometer);
    BLE.addCharacteristic(charSerNum);
    BLE.addCharacteristic(charFwVer);

    // Set initial values for static values
    charSerNum.setValue((uint8_t *)sernumstring, strlen(sernumstring));
    charFwVer.setValue(BCYCLE_APP_VERSION);
    
    // Set initial values for dynamic values
    charBattery.setValue("0");
    charOdometer.setValue("0");

    // Advertising setup: device name and our generic service UUID
    BleAdvertisingData advData;
    advData.appendLocalName(DEVICE_NAME);                       // NOTE: Trek does NOT have a BLE Sig Company ID
    advData.appendServiceUUID(genericServiceUUID);              // BBT firmware only advertises the generic service ID
    BLE.setAdvertisingInterval(APP_ADV_INTERVAL);
    BLE.advertise(&advData);

    Log.info("Bluetooth Address: %s", BLE.address().toString().c_str());
}

void BCycleBLE::loop() {
}

void BCycleBLE::updateData(bike_data_t &data) {
    static char valueStr[20] = {0};
    
    memset(valueStr, 0, 20);
    sprintf(valueStr, "% 4d", data.battery_pct);
    charBattery.setValue(valueStr);

    memset(valueStr, 0, 20);
    sprintf(valueStr, "% 12ld", data.odometer);
    charOdometer.setValue(valueStr);
}