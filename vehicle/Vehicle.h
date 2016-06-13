#ifndef VEHICLE_H
#define VEHICLE_H

// Gives us named indexes in the vehicle state arrays
#include "VehicleDefs.h"

#define CAN_BUFFER_SIZE 32

// Used to convert 32bit int <-> 4 bytes
union CanFrame
{
   uint8_t data[4];
   uint32_t full;
   struct {
      uint8_t type;
      uint8_t index;
      uint16_t val;
   } parts;
};

// The Vehicle state machine.

class Vehicle {
    public:
        // The state of the vehicle inputs
        bool switches[SW_COUNT];
        // Analog values
        uint16_t values[VAL_COUNT];
        // Constructor
        Vehicle();
        // Set up the vehicle
        void setup();
        // Process state changes
        void process();
        // Set state of a switch
        void setSwitch(uint8_t index, bool state, bool transmitCanMessage=true);
        // Set state of a value
        void setValue(uint8_t index, uint16_t val, bool transmitCanMessage=true);

    protected:
        uint8_t detectState();
        uint8_t state = STATE_STARTING;
        // Time that we woke up. lets us stay awake for a short time to allow 
        // debouncing stray wakeup interrupts.
        unsigned long wakingTime = 0;
        // Ability to queue and send CAN bus messages
        uint32_t canTxQueue[CAN_BUFFER_SIZE];
        uint8_t canTxQueueLength = 0;
        void queueCanMessage(uint8_t type, uint8_t index, uint16_t val);
        void sendCanMessage();
        // Receieve CAN messages
        void receiveCanMessage();
};

#endif
