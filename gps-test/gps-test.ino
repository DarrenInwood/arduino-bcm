#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include "UbloxGps.h"
UbloxGps ubloxGps;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ubloxGps.begin();
}

void loop() {
  ubloxGps.process();
  // put your main code here, to run repeatedly:
  Serial.print("satellites = ");
  Serial.println(ubloxGps.satellites);
  Serial.print("speed = ");
  Serial.println(ubloxGps.speed);
  delay(2000);
}
