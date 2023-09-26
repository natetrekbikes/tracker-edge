#include "bike_config.h"
#include "config_service.h"

BikeConfig *BikeConfig::_instance;

void BikeConfig::setup() {
    static ConfigObject bikeConfigDesc("bike", {
        ConfigInt("can_idle_timeout", &can_idle_timeout_s, 10, 60*60),
        ConfigFloat("publish_trigger_speed", &publish_trigger_speed_kmph, 0.1, 36.0),
        ConfigBool("udr_enable", 
            [this](bool &value, const void *context) {
                // Get thing from class
                value = this->enable_udr;
                return 0;
            },
            [this](bool value, const void *context) {
                // Set thing in class
                this->enable_udr = value;
                Log.info("udr_enable = %s", value ? "true" : "false");
                return 0;
            }
        )
    });
    ConfigService::instance().registerModule(bikeConfigDesc);

    logSettings();
}

void BikeConfig::loop() {

    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 10000) {
        lastLog = millis();
        logSettings();
    }
}

void BikeConfig::logSettings() {
    Log.info("Settings: {idleTimeout=%li, publishTriggerSpeed=%0.2f kmph, UDREnable=%s}", 
        can_idle_timeout_s, publish_trigger_speed_kmph, enable_udr ? "true" : "false");
} 

// static 
BikeConfig &BikeConfig::instance() {
    if (!_instance) {
        _instance = new BikeConfig();
    }
    return *_instance;
}

BikeConfig::BikeConfig() {
}

BikeConfig::~BikeConfig() {
}
