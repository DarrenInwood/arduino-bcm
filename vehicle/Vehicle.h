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
        // Set state of a switch
        void setSwitch(uint8_t index, bool state, bool transmitCanMessage=true);
        // Set state of a value
        void setValue(uint8_t index, uint16_t val, bool transmitCanMessage=true);

    protected:
        void queueCanMessage(uint8_t type, uint8_t index, uint16_t val);
};

#endif
