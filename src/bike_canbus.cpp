#include "Particle.h"
#include "bike_canbus.h"
#include "tracker.h"

BikeCANBus *BikeCANBus::_instance = nullptr;

Logger mcp_log("app.mcp25625");
Logger bike_can("app.bike_can");
Logger bike_can_raw("app.bike_can.raw");

void BikeCANBus::setup() {
    // Make sure the last parameter is MCP_20MHZ; this is dependent on the crystal
    // connected to the CAN chip and it's 20 MHz on the Tracker SoM.
    _status = begin(MCP_RX_STDEXT, BIKE_CAN_SPEED, MCP_20MHZ);
    if(_status == CAN_OK) {        
        mcp_log.info("Init OK");

        // Change to normal mode to allow messages to be transmitted. If you don't do this,
        // the CAN chip will be in loopback mode.
        setMode(MCP_MODE_NORMAL);
    }
    else {
        mcp_log.error("Init FAILED (%d)", _status);
    }
}

void BikeCANBus::sleepPrepareCallback() {
    // Clear all interrupts and disable RX buffer interrupts
    // We enable the wake interrupt later on in the sleep code, and this allows us to guarantee any interrupt received is for CAN activity
    mcp2515_setRegister(MCP_CANINTE, 0x00);     // Clear all interrupt enables so INT stays HIGH
    mcp2515_setRegister(MCP_CANINTF, 0x00);     // Clear all interrupt flags (flags RX buffers as empty as well)
}

// TODO: Verify power consumption in sleep to see if this actually works or not
void BikeCANBus::sleepCallback() {
    // Put the MCP into sleep mode (lowest power mode)
    if (sleep() == CAN_OK) {
        mcp_log.trace("Enter sleep mode OK");
    } else {
        mcp_log.error("Enter sleep mode FAILED");
    }

    digitalWrite(CAN_STBY, HIGH);                   // Put CAN TXR in standby using the STBY pin (disables oscillator)
    
    mcp2515_initCANBuffers();                       // Clear out CAN buffers before sleep, as they may have stale data in them
    mcp2515_setRegister(MCP_CANINTE, MCP_WAKIF);    // Enable CAN wake interrupt only

    // Flag our RX buffers as empty again, in case we have received anything in between sleepPrepare and here
    mcp2515_setRegister(MCP_CANINTF, 0x00);

    // The hope here is that it will take longer than the sleep code to fill the RX buffers
    // and trigger the interrupt if the CAN bus is active when we hit this routine.
}

void BikeCANBus::wakeup() {
    mcp_log.trace("Wakeup");
    digitalWrite(CAN_STBY, LOW);    // Bring MCP out of standby
    delayMicroseconds(10);          // MCP needs 128 TOsc to begin: 128/20MHz = 6.4us

    // Check for errors here, report out, clear error register
    uint8_t error;
    if (CAN_CTRLERROR == checkError(&error)) {
        mcp_log.warn("EFLG = 0x%02X", error);
        mcp2515_setRegister(MCP_EFLG, 0x00);
    } 
    else {
        mcp_log.trace("no errors on wake");
    }

    // TODO: see if we really need to clear this out
    mcp2515_setRegister(MCP_CANINTF, 0x00);                     // Clear interrupt flags and RX buffers. We throw out some data here, since we woke on CAN
    mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);    // Enable our RX buffer interrupts, disable WAK interrupt

    setMode(MCP_MODE_NORMAL);       // The MCP wakes up in Listen Only mode, so we need to reset it to Normal mode

    mcp_log.info("UP!");
}

void BikeCANBus::loop() {
    // Handle received CAN data
    if (!digitalRead(CAN_INT)) {
        // Validate interrupt is because we have a message available
        if (CAN_MSGAVAIL == checkReceive()) {
            can_frame_t frame;
            readMsgBufID(&frame.rxId, &frame.len, frame.rxBuf);
            _last_can_frame_time_ms = millis();

            if (_is_active == false) {
                _is_active = true;
                bike_can.info("CAN Bus activity detected");
            }
            
            _fresh_data = processCANFrame(frame);
        } else {
            // some other trigger for INT 
            mcp_log.warn("INT triggered, no messages: {CANINTE: 0x%02X, CANINTF: 0x%02X}", mcp2515_readRegister(MCP_CANINTE), mcp2515_readRegister(MCP_CANINTF));

            uint8_t error;
            if (CAN_CTRLERROR == checkError(&error)) {
                // Getting a RX buffer overflow error (0xC0), presumably because
                // the buffer fills up and we don't empty it if the tracker is caught unaware
                // When waking from sleep, we need to clear the buffer errors and interrupt errors
                // sort of a "soft reset"
                // CHALLENGE: we won't wake up if the buffers are full....perhaps we turn this interrupt off
                // and just clear the buffers occasionally?
                if ( (error & MCP_EFLG_ERRORMASK) & (MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR) ) {
                    Log.warn("RX Buffer(s) full: {0: %s, 1: %s}, clearing error", 
                        (error & MCP_EFLG_RX0OVR) ? "TRUE" : "FALSE", 
                        (error & MCP_EFLG_RX1OVR) ? "TRUE" : "FALSE"
                    );
                    mcp2515_modifyRegister(MCP_EFLG, (MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR), 0x00);
                } else {
                    mcp_log.error("Other MCP Error: 0x%02X", error);
                }
            }
        }
    }

    if (_is_active == true && millis() - _last_can_frame_time_ms > BIKE_CAN_INACTIVITY_PERIOD_S*1000) {
        _is_active = false;
        bike_can.info("CAN Bus inactivity detected (idle for %u seconds)", BIKE_CAN_INACTIVITY_PERIOD_S);
    }
}

void BikeCANBus::getBikeData(bike_data_t &data) {
    memcpy(&data, &_bike_data, sizeof(bike_data_t));
    _fresh_data = false;
}

// This section contains vehicle-specific CAN frame processing code
bool BikeCANBus::processCANFrame(can_frame_t &frame) {
    bool success = false;
    
    // Log raw bytes
    if (bike_can_raw.isTraceEnabled()) {
        char data_str[32];
        for (unsigned char i = 0; i < frame.len; i++) {
            sprintf(&(data_str[i*3]), " %02x", frame.rxBuf[i]);
        }
        bike_can_raw.trace("%08lx:%s", frame.rxId, data_str);
    }

    switch(frame.rxId) {
        // Assist Level (4 bytes)
        // [0]   Assist level
        // [3:1] Unknown / 0
        case 0x03B: {
            if (frame.len == 4) {
                _bike_data.pas_level = frame.rxBuf[0];
                bike_can_raw.info("Assist Level Frame: {pas_level: %d}", _bike_data.pas_level);
                success = true;
            }
            break;
        }

        // Speed (2 bytes)
        // [1:0] Speed in 0.01 km/h or 1/360 m/s
        case 0x0D1: {
            if (frame.len == 2) {
                uint16_t speed_temp;
                speed_temp  = (uint16_t)(frame.rxBuf[0] << 8);
                speed_temp |= (uint16_t)(frame.rxBuf[1]);
                _bike_data.speed = ((float)speed_temp) / 100.0f; // to km/h
                // _bike_data.speed = ((float)speed_temp) / 360.0f; // to m/s
                bike_can_raw.info("Speed Frame: {speed: %0.2f km/h (raw=%d)}", _bike_data.speed, speed_temp);
                success = true;
            }
            break;
        }
        
        // Charge Status (7 bytes)
        // [4:0] Unknown
        // [5]   Battery capacity, 0.1 Ah
        // [6]   Battery percent, %
        case 0x111: {
            if (frame.len == 7) {
                _bike_data.battery_capacity = ((float)frame.rxBuf[5]) / 10.0f;
                _bike_data.battery_pct      = frame.rxBuf[6];
                bike_can_raw.info("Charge Status Frame: {capacity: %0.1fAh, charge: %u%%}", _bike_data.battery_capacity, _bike_data.battery_pct);
                success = true;
            }
            break;
        }
        
        // Odometer Frame (8 bytes)
        // [3:0] Odometer reading, meters
        // [7:4] Unknown
        case 0x202: {
            if (frame.len == 8) {
                _bike_data.odometer  = (uint32_t)(frame.rxBuf[0] << 24);
                _bike_data.odometer |= (uint32_t)(frame.rxBuf[1] << 16);
                _bike_data.odometer |= (uint32_t)(frame.rxBuf[2] <<  8);
                _bike_data.odometer |= (uint32_t)(frame.rxBuf[3]);
                bike_can_raw.info("Odometer Frame: {odo: %12ldm}", _bike_data.odometer);
                success = true;
            }
            break;
        }

        // Battery (8 bytes)
        // [3:0] Time since full, seconds
        // [4]   Battery percent, %
        // [5:7] Unknown
        case 0x203: {
            if (frame.len == 8) {
                _bike_data.battery_time_since_full  = (uint32_t)(frame.rxBuf[0] << 24);
                _bike_data.battery_time_since_full |= (uint32_t)(frame.rxBuf[1] << 16);
                _bike_data.battery_time_since_full |= (uint32_t)(frame.rxBuf[2] <<  8);
                _bike_data.battery_time_since_full |= (uint32_t)(frame.rxBuf[3]);
                uint8_t pct_temp = frame.rxBuf[4];
                bike_can_raw.info("Battery Frame: {time_since_full: %lus, charge: %u%%}", _bike_data.battery_time_since_full, pct_temp);
                success = true;
            }
            break;
        }

        default:
            bike_can_raw.trace("Unknown Frame: 0x%04lX (%u bytes)", frame.rxId, frame.len);
            break;
        
    }

    return success;
}

void BikeCANBus::sendDisplayCommand(display_cmd_t cmd) {
    // byte trySendMsgBuf(unsigned long id, byte ext, byte rtrBit, byte len, const byte* buf, byte iTxBuf = 0xff); 
    // as sendMsgBuf, but does not have any wait for free buffer
    uint8_t data[] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, (uint8_t)cmd};

    if (CAN_OK == trySendMsgBuf(BOSCH_DISPLAY_COMMAND_ID, 0, 0, 8, data)) {
        bike_can.info("Display Command 0x%02X SUCCESS", (uint8_t)cmd);
    } else {
        bike_can.error("Display Command 0x%02X FAIL", (uint8_t)cmd);
    }
}

void BikeCANBus::turnBikeOff() {
    sendDisplayCommand(display_cmd_t::on_off);
    for (int i = 0; i < 3; i++) {
        if (CAN_OK == trySendMsgBuf(BOSCH_OFF_COMMAND_ID, 0, 0, 1, {0x00})) {
            bike_can.info("Sent Off Command 0x61 ��%u", i);
        } else {
            bike_can.error("Off Command 0x61  ��%u FAIL", i);
            return;
        }
        delay(10);
    }
    bike_can.info("Bike should be off?");
}