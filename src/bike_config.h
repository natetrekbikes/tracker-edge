#pragma once

#include "Particle.h"

class BikeConfig {
public:
    void setup();
    void loop();

    void logSettings();

    int32_t getIdleTimeout() const { return can_idle_timeout_s; };
    double getPublishTriggerSpeed() const { return publish_trigger_speed_kmph; };
    bool getUDREnable() const { return enable_udr; };

    static BikeConfig &instance();

protected:
    BikeConfig();
    virtual ~BikeConfig();

    int32_t can_idle_timeout_s = 60;
    double publish_trigger_speed_kmph = 8.0;
    bool enable_udr = true;

    static BikeConfig *_instance;
};