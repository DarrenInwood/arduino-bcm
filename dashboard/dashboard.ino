#include <SCoop.h>

#include "DashboardDefs.h"

// --------------------------------------------------------------------
// Vehicle state
// --------------------------------------------------------------------
#include <VehicleDefs.h>
#include <Vehicle.h>
Vehicle vehicle;

// --------------------------------------------------------------------
// PCA9865 16-channel 12-bit PWM expander
// --------------------------------------------------------------------
// http://www.adafruit.com/products/815
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// --------------------------------------------------------------------
// U-Blox NEO-6M GPS Receiver
// --------------------------------------------------------------------
#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include "UbloxGps.h"
UbloxGps ubloxGps;

// --------------------------------------------------------------------
// CAN bus interface
// --------------------------------------------------------------------
#define CAN_MSG_ID    0x07

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN    4
#define CAN_INT_PIN   2
MCP_CAN CAN(CAN_CS_PIN);

#define CAN_TX_QUEUE_LEN  8
defineFifo(canTxQueue, uint32_t, CAN_TX_QUEUE_LEN);

#define CAN_RX_QUEUE_LEN  8
defineFifo(canRxQueue, uint32_t, CAN_RX_QUEUE_LEN);

// Flag used to indicate that there is something to read
// on the CAN bus
volatile bool canRxSema = false;

// Used to convert messages between various integer formats
#define CAN_TYPE_SWITCH   0x00
#define CAN_TYPE_VALUE    0x01
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
CanFrame buf;

/*
 * Sets up the interrupt to trigger reading incoming CAN messages.
 */
void setupCanbus()
{
  // Interrupt on pin D2 triggere ISR
  attachInterrupt(
    digitalPinToInterrupt(CAN_INT_PIN),
    canInterrupt,
    LOW
  );
  // Start the CAN bus at 100Kbps
  CAN.begin(CAN_100KBPS);
}

/*
 * Add a switch type CAN message to the transmit queue
 */
void queueCanSwitch(uint8_t index)
{
  buf.parts.type = CAN_TYPE_SWITCH;
  buf.parts.index = index;
  buf.parts.val = (uint16_t)vehicle.switches[index];
  canTxQueue.put(&(buf.full));
}

/*
 * Add a value type CAN message to the transmit queue
 */
void queueCanValue(uint8_t index)
{
  buf.parts.type = CAN_TYPE_VALUE;
  buf.parts.index = index;
  buf.parts.val = vehicle.values[index];
  canTxQueue.put(&(buf.full));
}

// --------------------------------------------------------------------
// Dashboard state
// --------------------------------------------------------------------

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

// --------------------------------------------------------------------
// ISRs
// --------------------------------------------------------------------

/*
 * ISR:  Pin change on pin D2 (INT0) means CAN interrupt. Signal
 * the CAN receive thread to process it.
 */
void canInterrupt()
{
  // Tell the read task to read.
  canRxSema = true;
}

// --------------------------------------------------------------------
// Tasks
// --------------------------------------------------------------------

/*
 * TASK:  When there's a CAN message waiting to be read, read it
 * and add it to the receieve queue.
 */
uint8_t len;
defineTaskLoop(readCanMessage, 64)
{
  // Wait until the semaphore is set
  // (ie. when the interrupt handler has set it)
  sleepUntil(canRxSema);
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf.data);
    canRxQueue.put(&(buf.full));
  }
}

/*
 * TASK:  When there's a message in the CAN receive buffer, process it.
 */
defineTaskLoop(processCanRxQueue, 64)
{
  if (canRxQueue.get(&(buf.full))) {
    // Set the new state on the vehicle state
    switch (buf.parts.type) {
       case CAN_TYPE_SWITCH:
           // Don't send a can message if we changed state
           vehicle.setSwitch(buf.parts.index, buf.parts.val);
       break;
       case CAN_TYPE_VALUE:
           // Don't send a can message if we changed state
           vehicle.setValue(buf.parts.index, buf.parts.val);
       break;
    }
  }
}

/*
 * TASK:  When there's a message in the CAN transmit buffer, send it.
 */
defineTaskLoop(processCanTxQueue, 64)
{
  if (canTxQueue.get(&(buf.full))) {
    CAN.sendMsgBuf(CAN_MSG_ID, 0, sizeof(buf.data), buf.data);
  }
}

// Take the vehicle state and send it to the display
// Also takes care of how often things are turned on/off
defineTaskLoop(updateDisplay, 128)
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

    sleep(100);
}

defineTaskLoop(updateGauges, 128)
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

    sleep(500);
}

defineTaskLoop(updateGps, 64)
{
    ubloxGps.process();

    if (vehicle.values[VAL_SPEED] != ubloxGps.speed) {
        vehicle.setValue(VAL_SPEED, ubloxGps.speed);
    }

    sleep(250);
}

// --------------------------------------------------------------------
// Arduino setup
// --------------------------------------------------------------------

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

// Sends a command to the Nextion display
void sendNextionCommand(const char* cmd){
  Serial.print(cmd);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void setupNextion()
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
}

void setup()
{
  // Nextion display is on hardware serial at 9600 baud
  setupNextion();
  // Set up hardware outputs 
  setupOutputs();
  // Set up CAN bus comms
  setupCanbus();

  mySCoop.start();
}

void loop()
{
  // Empty
}


