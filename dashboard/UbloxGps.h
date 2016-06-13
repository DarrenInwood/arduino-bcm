#ifndef UBLOXGPS_H
#define UBLOXGPS_H

// GPS needs some configuration on setup, and needs to turn itself off
// when the dashboard does.

class UbloxGps
{
    public:
        float speed;
        uint32_t odometer;
        uint8_t satellites;

        void setup();
        void process();
    protected:
};

#endif
