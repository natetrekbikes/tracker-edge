// CANBUS class to support talking to an eBike
#ifndef _BIKE_CANBUS_H
#define _BIKE_CANBUS_H

#include "mcp_can.h"
#include "mcp_can_dfs.h"

#define BIKE_CAN_SPEED                  CAN_500KBPS
#define BIKE_CAN_INACTIVITY_PERIOD_S    5

#define BOSCH_DISPLAY_COMMAND_ID        (0x131)
#define BOSCH_OFF_COMMAND_ID            (0x061)
#define BOSCH_ON_COMMAND_ID             (0x055)

typedef struct {
    uint8_t pas_level;
    float speed;
    float battery_capacity;
    uint8_t battery_pct;
    uint32_t odometer;
    uint32_t battery_time_since_full;
} bike_data_t;

typedef struct {
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];
} can_frame_t;

typedef enum {
    asst_plus   = 0x00,
    asst_minus  = 0x02,
    on_off      = 0x0B
} display_cmd_t;

class BikeCANBus : public MCP_CAN {

    public:

        static BikeCANBus &instance()
        {
            if(!_instance)
            {
                _instance = new BikeCANBus();
            }
            return *_instance;
        }

        BikeCANBus() : 
            MCP_CAN{ CAN_CS, SPI1 },
            _bike_data({
                .pas_level = 0,
                .speed = 0.0f,
                .battery_capacity = 0.0f,
                .battery_pct = 0,
                .odometer = 0,
                .battery_time_since_full = 0
            }),
            _last_can_frame_time_ms(0),
            _fresh_data(false)
        {
        };

        void setup();
        void loop();

        void getBikeData(bike_data_t &data);
        bool processCANFrame(can_frame_t &frame);
        void canThreadFunction(void *param);

        void sleepPrepareCallback();
        void sleepCallback();
        void wakeup();

        inline bool isActive() {
            return _is_active;
        }

        // TODO: Add activity change callback!

        inline bool isDataFresh() {
            return _fresh_data;
        }

        inline long unsigned int getLastFrameTimeMs() {
            return _last_can_frame_time_ms;
        }

        void sendDisplayCommand(display_cmd_t cmd);
        void turnBikeOff();

    private:
        static BikeCANBus *_instance;
        byte _status;

        bike_data_t _bike_data;
        long unsigned int _last_can_frame_time_ms;
        bool _fresh_data;
        bool _is_active;
};

#endif // _BIKE_CANBUS_H