#include "Arduino.h"
#include "UbloxGps.h"

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#ifndef PIN_GPS_RX
#define PIN_GPS_RX 7
#endif

#ifndef PIN_GPS_TX
#define PIN_GS_TX 8
#endif

TinyGPSPlus gps;
SoftwareSerial ss(PIN_GPS_RX, PIN_GS_TX);

void UbloxGps::setup()
{
    ss.begin(9600);
}

void UbloxGps::process()
{
    if (!ss.available()) {
        // Nothing to process
        return;
    }

    // Let TinyGPS decode the NMEA string
    while (ss.available()) {
        gps.encode(
            ss.read()
        );
    }

    satellites = gps.satellites.value();
    
    if (!gps.location.isValid()) {
        // Exit if we have no fix
        return;
    }

    speed = gps.speed.kmph();

#ifdef DEBUG
    Serial.print("GPS fix! lat=");
#endif    
}

