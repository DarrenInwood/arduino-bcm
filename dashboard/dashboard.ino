/*
 * Pro mini programming with Uno:
 * Use settings Programmer = AVRISP mkII
 * Press reset button on pro mini shortly after IDE starts trying to upload.
 */

//#define DEBUG 1

// Controls the 16-output 12-bit PWM chip
// http://www.adafruit.com/products/815
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Tell our CAN bus interface which pins to use
#define CAN_CS_PIN    4
#define CAN_INT_PIN   2

// Vehicle sets up CAN bus
#include "Vehicle.h"
Vehicle vehicle;

// Semaphore to enable detecting update timing
bool updateOutputsFlag = false;

// Counter so we know how often to update the gauges (2Hz)
#define UPDATE_GAUGE_MILLIS 500
uint8_t updateGaugeCount = 0;

// Counter so we know how often to update the Nextion screen (10Hz)
// Also controls how often we check whether things are turned on/off
#define UPDATE_DISPLAY_MILLIS 100
uint8_t updateDisplayCount = 0;

// Counter so we know how often to poll the GPS while we're running (4Hz)
#define UPDATE_GPS_MILLIS 250
uint32_t updateGpsCount = 0;

// Tells us which input is connected where
#include "DashboardDefs.h"

// Set up our initial state
struct Dashboard {
    // State
    bool gaugesOn;
    bool displayOn;
    bool gpsOn;
    // Gauges
    float afr;
    float vacuum;
    float volt1;
    float volt2;
    uint8_t clt;
    uint8_t fuel;
    uint16_t rpm;
    // Display values
};

Dashboard dashboard = {
    // State
    false, // gauges on
    false, // display on
    false, // GPS on
    // Gauges
    14.7, // air-fuel ratio
    -1.0, // vacuum (bar)
    12.0, // volts 1 (V)
    12.0, // volts 2 (V)
    20,   // coolant temperature (degrees C)
    50,   // Fuel level (percent)
    800   // RPM (revs/minute)
    // Display values
};

#include "UbloxGps.h"
UbloxGps ubloxGps;

// Set up the application
void setup()
{
  Serial.begin(9600);

  // show the Ford logo for 3s
  String cmd = "";

  cmd = "dim 100";
  sendNextionCommand(cmd.c_str());

  // show ford logo
//  cmd = "page 0";
//  sendNextionCommand(cmd.c_str());
//  cmd = "ref 0";
//  sendNextionCommand(cmd.c_str());

  cmd = "page 1";
  sendNextionCommand(cmd.c_str());
  cmd = "vis indl,0";
  sendNextionCommand(cmd.c_str());
  cmd = "vis brake,0";
  sendNextionCommand(cmd.c_str());
  cmd = "vis high,0";
  sendNextionCommand(cmd.c_str());
  cmd = "vis indr,0";
  sendNextionCommand(cmd.c_str());
  cmd = "vis clt,0";
  sendNextionCommand(cmd.c_str());
  cmd = "vis bat,0";
  sendNextionCommand(cmd.c_str());
  cmd = "vis oil,0";
  sendNextionCommand(cmd.c_str());
  cmd = "vis fuel,0";
  sendNextionCommand(cmd.c_str());
  cmd = "ref 0";
  sendNextionCommand(cmd.c_str());

    setupOutputs();

    // Trigger an interrupt 
    OCR0A = 0xAF;  // when Timer0 equals this value, A timer interrupt is triggered
    TIMSK0 |= _BV(OCIE0A); // Enable the Timer0 compare interrupt

    enableWakeupInterrupts();

    // set up CAN bus etc
    vehicle.setup();

    // stay on!
    vehicle.switches[SW_ACC] = true;

    // GPS setup
    ubloxGps.setup();
}

// Set the things that can wake us back up again from sleep mode
void enableWakeupInterrupts() {
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), wakeup_ISR, LOW);
}

void setupOutputs()
{
    pwm.begin();
    pwm.setPWMFreq(1000);  // This is the maximum PWM frequency

    // Set output pin modes
    pinMode(DOUT_METER_RELAY, OUTPUT);
    pinMode(DOUT_BACKLIGHT_RELAY, OUTPUT);
    pinMode(DOUT_GPS_RX, OUTPUT);
    pinMode(DOUT_GPS_TX, OUTPUT);
    pinMode(DOUT_TACHO, OUTPUT);

    // Default settings
    digitalWrite(DOUT_METER_RELAY, 1);     // 1=off (relay module)
    digitalWrite(DOUT_BACKLIGHT_RELAY, 1); // 1=off (relay module)
}

// Timer compare interrupt has happened
SIGNAL(TIMER0_COMPA_vect) 
{
    updateOutputsFlag = true;
}

// Wake up interrupt handler
void wakeup_ISR()
{
    // noop - loop() will take care of everything
}

void loop()
{
    // Update things on a schedule
    if (updateOutputsFlag) {
        updateOutputsFlag = false; // triggers every ms
        
        updateGaugeCount++;
        if (updateGaugeCount > UPDATE_GAUGE_MILLIS) {
            updateGaugeCount = 0;
            updateGauges();
        }

        updateDisplayCount++;
        if (updateDisplayCount > UPDATE_DISPLAY_MILLIS) {
            updateDisplayCount = 0;
            updateDisplay();
        }

        updateGpsCount++;
        if (updateGpsCount > UPDATE_GPS_MILLIS) {
            updateGpsCount = 0;
            updateGps();
        }
    }
    
    // Let the vehicle handle comms, sleep etc
    vehicle.process();
}

// Take the vehicle state and send it to the display
// Also takes care of how often things are turned on/off
void updateDisplay()
{
    String cmd = "";
    String on = "1";
    String off = "2";

    // IG means 'turn on the gauges'
    if (vehicle.switches[SW_IGNITION] != dashboard.gaugesOn) {
        dashboard.gaugesOn = vehicle.switches[SW_IGNITION];
        digitalWrite(DOUT_METER_RELAY, !dashboard.gaugesOn); // 0=on
        Serial.print("Gauges: ");
        Serial.println(dashboard.gaugesOn);
    }
    // IG means 'turn on the screen'
    if (vehicle.switches[SW_IGNITION] != dashboard.displayOn) {
        dashboard.displayOn = vehicle.switches[SW_IGNITION];
        if (dashboard.displayOn) {
            cmd = "sleep 0";
            sendNextionCommand(cmd.c_str());
        } else {
            cmd = "sleep 1";
            sendNextionCommand(cmd.c_str());
        }
    }

    // If our screen is off, don't bother updating anything else
    if (dashboard.displayOn) {

        cmd = "vis indl,";
        if (vehicle.switches[SW_WARN_TURNLEFT]) {
            cmd = cmd + on;
        } else {
            cmd = cmd + off;
        }
        sendNextionCommand(cmd.c_str());

        cmd = "vis brake,";
        if (vehicle.switches[SW_HANDBRAKE]) {
            cmd = cmd + on;
        } else {
            cmd = cmd + off;
        }
        sendNextionCommand(cmd.c_str());
        
        cmd = "vis high,";
        if (vehicle.switches[SW_HIGHBEAM]) {
            cmd = cmd + on;
        } else {
            cmd = cmd + off;
        }
        sendNextionCommand(cmd.c_str());

        cmd = "vis indr,";
        if (vehicle.switches[SW_WARN_TURNRIGHT]) {
            cmd = cmd + on;
        } else {
            cmd = cmd + off;
        }
        sendNextionCommand(cmd.c_str());

        cmd = "vis clt,";
        if (vehicle.switches[SW_WARN_COOLANTLEVEL]) {
            cmd = cmd + on;
        } else {
            cmd = cmd + off;
        }
        sendNextionCommand(cmd.c_str());

//        cmd = "vis bat,0";
//        sendNextionCommand(cmd.c_str());

        cmd = "vis oil,";
        if (vehicle.switches[SW_WARN_OILPRESSURE]) {
            cmd = cmd + on;
        } else {
            cmd = cmd + off;
        }
        sendNextionCommand(cmd.c_str());

        cmd = "vis fuel,";
        if (vehicle.switches[SW_WARN_FUEL]) {
            cmd = cmd + on;
        } else {
            cmd = cmd + off;
        }
        sendNextionCommand(cmd.c_str());

    }

    // Refresh the screen
    cmd = "ref 0";
    sendNextionCommand(cmd.c_str());
}

void updateGauges()
{
    // IG means 'turn on the gauges'
    if (vehicle.switches[SW_IGNITION] != dashboard.gaugesOn) {
        dashboard.gaugesOn = vehicle.switches[SW_IGNITION];
        digitalWrite(DOUT_METER_RELAY, !dashboard.gaugesOn); // 0=on
        Serial.print("Gauges: ");
        Serial.println(dashboard.gaugesOn);
    }
    // IG means 'turn on the screen'
    if (vehicle.switches[SW_IGNITION] != dashboard.displayOn) {
        dashboard.displayOn = vehicle.switches[SW_IGNITION];
        if (dashboard.displayOn) {
            // Nextion.write("sleep = 0");
            // go to page 2
        } else {
            // Nextion.write("sleep = 1");
        }
    }

    // If our gauges are off, don't bother updating anything else
    if (!dashboard.gaugesOn) {
        return;
    }

    if (vehicle.values[VAL_COOLANT_TEMP] != dashboard.clt) {
        dashboard.clt = vehicle.values[VAL_COOLANT_TEMP];
        // convert value 30-200 degrees -> 0-4096
        float clt = dashboard.clt;
        clt = max(0, clt - 30);
        clt = clt * (4096/170);
        clt = min(4095, clt);        
        pwm.setPWM(AOUT_CLT, 0, clt);
    }
}

void updateGps()
{
    ubloxGps.process();

    if (vehicle.values[VAL_SPEED] != ubloxGps.speed) {
        vehicle.setValue(VAL_SPEED, ubloxGps.speed);
    }
}

// Sends a command to the Nextion display
void sendNextionCommand(const char* cmd){
  Serial.print(cmd);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}
