/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Particle.h"

#include "location_service.h"
#include "tracker_config.h"
#include "tracker.h"

#include "bike_canbus.h"
#include "bike_config.h"

#include "bcycle_ble.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#if TRACKER_PRODUCT_NEEDED
PRODUCT_ID(TRACKER_PRODUCT_ID);
#endif // TRACKER_PRODUCT_NEEDED
PRODUCT_VERSION(TRACKER_PRODUCT_VERSION);

// Prototypes
void locationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context);
void wakeCallback(TrackerSleepContext context);
void prepareSleepCallback(TrackerSleepContext context);
void sleepCallback(TrackerSleepContext context);

// for publishing the nRF serial number
bool publish_serial_number_flag = false;
int publishSerialNumber(String extra) {
    publish_serial_number_flag = true;
    return 0;
}

int sendBikeOff(String extra) {
    if (BikeCANBus::instance().isActive()) {
        BikeCANBus::instance().turnBikeOff();
        return 0;
    } else {
        return -1;
    }
}
STARTUP(
    Tracker::startup();
);

SerialLogHandler logHandler(115200, LOG_LEVEL_ALL, {
    // { "app.gps.nmea", LOG_LEVEL_INFO },
    // { "app.gps.ubx",  LOG_LEVEL_INFO },
    // // Added by Nate
    // { "ncp.at", LOG_LEVEL_INFO },
    // { "net.ppp.client", LOG_LEVEL_INFO },
    // { "otp", LOG_LEVEL_INFO },
});

bike_data_t data;

typedef enum {
    STATE_BIKE_INACTIVE = 0,
    STATE_BIKE_ACTIVE,    
} system_state_t;

inline const char * stateToStr(system_state_t state) {
    switch(state) {
        case STATE_BIKE_INACTIVE:   return "STATE_BIKE_INACTIVE"; break;
        case STATE_BIKE_ACTIVE:     return "STATE_BIKE_ACTIVE"; break;
        default:                    return "STATE_UNKNOWN"; break;
    }
};

system_state_t state = STATE_BIKE_INACTIVE;
system_state_t next_state = STATE_BIKE_INACTIVE;

void setup()
{
    // Uncomment to make it easier to see the serial logs at startup
    // waitFor(Serial.isConnected, 15000);
    // delay(1000);

    // Limit charging current for bike 12V output
    // PMIC pmic;
    // pmic.setChargeCurrent(0,1,1,0,0,0);

    // Location Service specific configuration
    // Yaw: 180 degrees ��Tracker mounted backwards (ports forward)
    // Pitch: 18 degrees ��Tracker mounted on frame, tilted backwards
    // Roll: 180 degrees ��M8U is mounted on the bottom of the Tracker One PCB
    LocationServiceConfiguration locConfig;
    locConfig.enableUDR(false)
            //  .udrModel(UBX_DYNAMIC_MODEL_BIKE)
            //  .enableIMUAutoAlignment(false)             // Auto-alignment is disabled with UBX_DYNAMIC_MODEL_BIKE
            //  .imuOrientationAngles(180.0, 18.0, 180.0)
            //  .imuToVRP(-75, -50, 0)
             .enableFastLock(false)
             .enableHotStartOnWake(true)
             .enableAssistNowAutonomous(true);

    // Leave CAN 5V power enabled during sleep so we can wake on CAN data
    // Also set up our location service configuration
    TrackerConfiguration config;
    config.enableIoCanPowerSleep(false)
          .enableIoCanPower(true)
          .locationServiceConfig(locConfig);

    // Tracker configuration
    Tracker::instance().init(config);
    Tracker::instance().location.regLocGenCallback(locationGenerationCallback, &data);

    // Sleep configuration
    TrackerSleep::instance().registerSleepPrepare(prepareSleepCallback);
    TrackerSleep::instance().registerSleep(sleepCallback);
    TrackerSleep::instance().registerWake(wakeCallback);
    TrackerSleep::instance().wakeFor(CAN_INT, CHANGE);     // Wake on CAN activity

    // Custom configs
    BikeConfig::instance().setup();

    // Initialize Bike CAN Bus
    BikeCANBus::instance().setup();

    // Initialize BCycle BBT BLE stack
    BCycleBLE::instance().setup();

    // Functions
    Particle.function("Get Serial Number", publishSerialNumber);
    Particle.function("Bike Off", sendBikeOff);

    // Connect to the cloud!
    Particle.connect();
    Particle.publishVitals(5*60);   // Every 5 minutes
}

void locationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context) {
    writer.name("can").beginObject();
        writer.name("sp").value(data.speed, 2);
        writer.name("bt").value(data.battery_pct);
        writer.name("odo").value(data.odometer);
        writer.name("pas").value(data.pas_level);
    writer.endObject();
}

void loop()
{
    Tracker::instance().loop();
    BikeCANBus::instance().loop();

    switch(state) {
        // CAN Bus is inactive
        case STATE_BIKE_INACTIVE: {
            if (BikeCANBus::instance().isActive()) {
                // If we get CAN data, stay awake and publish
                TrackerSleep::instance().pauseSleep();
                TrackerLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "active");
                next_state = STATE_BIKE_ACTIVE;
            } else {
                TrackerSleep::instance().resumeSleep();
                next_state = STATE_BIKE_INACTIVE;
            }
            break;
        }

        // CAN Bus is active: constantly publish at maximum rate while bike is moving more than 5 mph (8 kmph)
        case STATE_BIKE_ACTIVE: {
            if (!BikeCANBus::instance().isActive()) {
                // We've gone inactive
                TrackerSleep::instance().resumeSleep();

                int32_t idle_timeout_s = BikeConfig::instance().getIdleTimeout();
                TrackerSleep::instance().extendExecutionFromNow(idle_timeout_s);
                Log.info("Bike Idle: sleeping in %li seconds", idle_timeout_s);
                TrackerLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "inactive");
                next_state = STATE_BIKE_INACTIVE;
            } else {
                // Refresh our shadow copy of bike data
                if (BikeCANBus::instance().isDataFresh()) { 

                    BikeCANBus::instance().getBikeData(data);
                    BCycleBLE::instance().updateData(data);

                    static long unsigned int last_publish = 0;
                    long unsigned int max_interval_ms = TrackerLocation::instance().getMaxInterval() * 1000;
                    
                    if ((millis() - last_publish >= max_interval_ms)) {
                        if (data.speed > (float)(BikeConfig::instance().getPublishTriggerSpeed())) {
                            Log.info("Publishing due to speed (%0.2f km/h)", (float)data.speed);
                            TrackerLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "speed");
                            last_publish = millis();
                        }
                    }
                }
                next_state = STATE_BIKE_ACTIVE;
            }
            break;
        }
    }

    if (next_state != state) {
        Log.info("State Transition: %s -> %s", stateToStr(state), stateToStr(next_state));
        state = next_state;
    }

    if (publish_serial_number_flag == true) {
        char pub_str[64] = { 0 };
	    sprintf(pub_str, "{\"sn\":%lu%lu}", NRF_FICR->DEVICEID[0], NRF_FICR->DEVICEID[1]);
        Particle.publish("serial_number", pub_str);
        publish_serial_number_flag = false;
    }
}

void sleepCallback(TrackerSleepContext context)
{
    // Called before we go to sleep. Adjust the time so we do a short wake every 60 minutes
    Log.info("sleep callback ��shutting MCP down");
    BikeCANBus::instance().sleepCallback();
}

void prepareSleepCallback(TrackerSleepContext context)
{
    BikeCANBus::instance().sleepPrepareCallback();
}

void wakeCallback(TrackerSleepContext context)
{
    // Called when we wake from sleep
    if (!TrackerSleep::instance().isFullWakeCycle()) {
        if (context.result.wakeupReason() == SystemSleepWakeupReason::BY_GPIO) {
            if (context.result.wakeupPin() == CAN_INT) {
                Log.info("Wakeup: CAN Interrupt. Forcing full wake");
                TrackerSleep::instance().forceFullWakeCycle();
            } else {
                Log.info("Wakeup: GPIO (pin=%d)", context.result.wakeupPin());
            }
        }
        else if(!digitalRead(CAN_INT)) {
            Log.info("Wakeup: no CAN Interrupt, but CAN is active. Forcing full wake");
            TrackerSleep::instance().forceFullWakeCycle();
        }
        else {
            Log.info("Wakeup: Other (%u)", (uint16_t)context.result.wakeupReason());
        }
    }
    else {
        Log.info("Wakeup: Is full wake cycle");
    }

    // always wake up the CAN bus
    BikeCANBus::instance().wakeup();
}