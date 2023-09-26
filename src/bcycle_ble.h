#pragma once

#include "bike_canbus.h"

class BCycleBLE {

    public:

        static BCycleBLE &instance()
        {
            if(!_instance)
            {
                _instance = new BCycleBLE();
            }
            return *_instance;
        }

        BCycleBLE() {};

        void setup();
        void loop();
        void updateData(bike_data_t &data);

    private:
        static BCycleBLE *_instance;
};