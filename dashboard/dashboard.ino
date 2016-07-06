#include <Arduino.h>
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
#include "NMEAGPS.h"
#include <NeoSWSerial.h>
#include "GPSport.h"
#include "Streamers.h"

static NMEAGPS   gps;
static gps_fix  fix_data;

void setupGps()
{
  gps_port.attachInterrupt( GPSisr );
  gps_port.begin( 9600 );
}

// --------------------------------------------------------------------
// CAN bus communications
// --------------------------------------------------------------------
#include <CAN.h>
#include <SPI.h>

#define CAN_MSG_HEADER  0x02DA
#define CAN_TYPE_SWITCH 0x00
#define CAN_TYPE_VALUE  0x01

// Allows us to easily convert between the CAN message ID
// and the intent of the message
union CanMessageId
{
   uint8_t data[4];
   uint32_t full;
   struct {
      uint8_t index;
      uint8_t type;
      uint16_t header;
   } parts;
};

// Allows us to easily convert between various interpretations
// of values sent via CAN bus
union CanData
{
   uint8_t data[4];
   uint32_t full;
};

void setupCanbus(void)
{
  CAN.begin(CAN_BPS_500K);
}

/*
 * Add a switch type CAN message to the transmit queue
 */
void queueCanMessage(uint8_t swtype, uint8_t index)
{
  static CAN_Frame can_message;
  CanMessageId messageId;
  CanData messageData;

  // Encode the CAN message ID
  messageId.parts.header = CAN_MSG_HEADER;
  messageId.parts.type = swtype;
  messageId.parts.index = index;

  can_message.id = messageId.full;
  can_message.valid = true;
  can_message.rtr = 0;
  can_message.extended = CAN_EXTENDED_FRAME;
  can_message.length = 4;

  switch (swtype) {
    case CAN_TYPE_SWITCH:
      messageData.data[0] = 0x00;
      messageData.data[1] = 0x00;
      messageData.data[2] = 0x00;
      messageData.data[3] = (vehicle.switches[index] ? 0x01 : 0x00);
    break;
    case CAN_TYPE_VALUE:
      messageData.full = (uint32_t)vehicle.switches[index];
    break;
  }
  can_message.data[0] = messageData.data[0];
  can_message.data[1] = messageData.data[1];
  can_message.data[2] = messageData.data[2];
  can_message.data[3] = messageData.data[3];
  
  CAN.write(can_message);

//  Serial.print("send ");
//  Serial.print(millis());
//  Serial.print(F(",0x"));
//  Serial.print(can_message.id, HEX); //display message ID
//  Serial.print(',');
//  Serial.print(can_message.rtr); //display message RTR
//  Serial.print(',');
//  Serial.print(can_message.extended); //display message EID
//  Serial.print(',');
//  Serial.print(can_message.data[0], HEX);
//  Serial.print(':');
//  Serial.print(can_message.data[1], HEX);
//  Serial.print(':');
//  Serial.print(can_message.data[2], HEX);
//  Serial.print(':');
//  Serial.println(can_message.data[3], HEX);

}

/* 
 * Fetch and process any waiting receieved CAN messages.
 * Ensure this is called every 1ms so as not to lose packets.
 */
void processCanMessage()
{
  CAN_Frame message; // Create message object to use CAN message structure
  CanMessageId messageId;
  CanData messageData;
  if (CAN.available() == true) { // Check to see if a valid message has been received.
    message = CAN.read();
  
//    Serial.print("receive ");
//    Serial.print(millis());
//    Serial.print(F(",0x"));
//    Serial.print(message.id, HEX); //display message ID
//    Serial.print(',');
//    Serial.print(message.rtr); //display message RTR
//    Serial.print(',');
//    Serial.print(message.extended); //display message EID
//    Serial.print(',');
//    if (message.rtr == 1)
//    {
//      Serial.print(F(" REMOTE REQUEST MESSAGE ")); //technically if its RTR frame/message it will not have data So display this
//    }
//    else
//    {
//      Serial.print(message.length, HEX); //display message length
//      for (byte i = 0; i < message.length; i++)
//      {
//        Serial.print(',');
//        if (message.data[i] < 0x10) // If the data is less than 10 hex it will assign a zero to the front as leading zeros are ignored...
//        {
//          Serial.print('0');
//        }
//        Serial.print(message.data[i], HEX); //display data based on length
//      }
//    }
//    Serial.println();
  
    if (message.valid != 1) {
      return; // discard invalid messages 
    }
    if (message.extended != CAN_EXTENDED_FRAME) {
      return; // discard standard frame size messages
    }
    messageId.full = message.id;
    messageData.data[0] = message.data[0];
    messageData.data[1] = message.data[1];
    messageData.data[2] = message.data[2];
    messageData.data[3] = message.data[3];
    if (message.rtr == 1) {
      // We've requested a value
      queueCanMessage(messageId.parts.type, messageId.parts.index);
      return;
    }
    switch (messageId.parts.header) {
      // Normal message
      case CAN_MSG_HEADER:
        switch (messageId.parts.type)
        {
          case CAN_TYPE_SWITCH:
            vehicle.switches[messageId.parts.index] = (messageData.data[3] == 0x01);
          break;
          case CAN_TYPE_VALUE:
            vehicle.values[messageId.parts.index] = messageData.full;
          break;
        }
      break;
      default:
        // No default - other messages are discarded
        return;
      break;
    }
  }
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

static void GPSisr( uint8_t c )
{
  gps.handle( c );
}


// --------------------------------------------------------------------
// Tasks
// --------------------------------------------------------------------


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
//    cmd = "ref 0";
//    sendNextionCommand(cmd.c_str());
}

void updateGauges()
{
    // IG means 'turn on the gauges'
    if (vehicle.switches[SW_IGNITION] != dashboard.gaugesOn) {
        dashboard.gaugesOn = vehicle.switches[SW_IGNITION];
        digitalWrite(DOUT_METER_RELAY, !dashboard.gaugesOn); // 0=on
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
    String cmd = "speed.val=";
    float speed;
    while (gps.available()) {
        fix_data = gps.read();
        speed = fix_data.speed_kph() * 100;
        cmd = cmd + String(speed);
        sendNextionCommand(cmd.c_str());
    }
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

//// Sends a command to the Nextion display
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
  // Ublox GPS on pins 7/8 at 9600 baud
  setupGps();
  // Nextion display is on hardware serial at 9600 baud
  setupNextion();
  // Set up hardware outputs 
  setupOutputs();
  // Set up CAN bus comms
  setupCanbus();
}

uint32_t lastTime = millis();
void loop()
{
  processCanMessage();
  updateGps();
  if ((millis() - lastTime) > 1000) {
    updateDisplay();
    lastTime = millis();  
  }
}


