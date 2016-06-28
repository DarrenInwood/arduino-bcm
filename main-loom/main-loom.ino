// --------------------------------------------------------------------
// Physical hardware
// --------------------------------------------------------------------
#include "MainLoomDefs.h"

// --------------------------------------------------------------------
// Vehicle state
// --------------------------------------------------------------------
#include <VehicleDefs.h>
#include <Vehicle.h>
Vehicle vehicle;

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
  Serial.begin(115200);  
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

  Serial.print("send ");
  Serial.print(millis());
  Serial.print(F(",0x"));
  Serial.print(can_message.id, HEX); //display message ID
  Serial.print(',');
  Serial.print(can_message.rtr); //display message RTR
  Serial.print(',');
  Serial.print(can_message.extended); //display message EID
  Serial.print(',');
  Serial.print(can_message.data[0], HEX);
  Serial.print(':');
  Serial.print(can_message.data[1], HEX);
  Serial.print(':');
  Serial.print(can_message.data[2], HEX);
  Serial.print(':');
  Serial.println(can_message.data[3], HEX);
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
  
    Serial.print("receive ");
    Serial.print(millis());
    Serial.print(F(",0x"));
    Serial.print(message.id, HEX); //display message ID
    Serial.print(',');
    Serial.print(message.rtr); //display message RTR
    Serial.print(',');
    Serial.print(message.extended); //display message EID
    Serial.print(',');
    if (message.rtr == 1)
    {
      Serial.print(F(" REMOTE REQUEST MESSAGE ")); //technically if its RTR frame/message it will not have data So display this
    }
    else
    {
      Serial.print(message.length, HEX); //display message length
      for (byte i = 0; i < message.length; i++)
      {
        Serial.print(',');
        if (message.data[i] < 0x10) // If the data is less than 10 hex it will assign a zero to the front as leading zeros are ignored...
        {
          Serial.print('0');
        }
        Serial.print(message.data[i], HEX); //display data based on length
      }
    }
    Serial.println();
  
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
// PCF8574 I/O expanders
// --------------------------------------------------------------------
#include <Wire.h>
#include "PCF8574.h"
PCF8574 expander1(0x20);
PCF8574 expander2(0x21);

// --------------------------------------------------------------------
// Digital input debouncing
// --------------------------------------------------------------------
#include <button_debounce.h>
//Debouncer debouncePB(0x00); // Constructor takes 'pulledUpPins' - ie. normally high = 1, ie. "on" = 0V before 4049
//Debouncer debouncePD(DIN_TURNLEFT & DIN_TURNRIGHT);
//Debouncer debouncePCF1(DIN_BRAKESWITCH & DIN_FUELLOW & DIN_CHOKE & DIN_COOLANTLOW & DIN_OILPRESSURELOW & DIN_WIPERINT);
//Debouncer debouncePCF2(DIN_WASHERMOTOR & DIN_WIPERLOW & DIN_WIPERHIGH & DIN_UNKNOWN);
Debouncer debouncePB(0x00); // Constructor takes 'pulledUpPins' - ie. normally high = 1, ie. "on" = 0V before 4049
Debouncer debouncePD(0x00);
Debouncer debouncePCF1(0x00);
Debouncer debouncePCF2(0x00);

// --------------------------------------------------------------------
// Set up inputs etc
// --------------------------------------------------------------------
void setupInputs() {
  // Set pin modes
  pinMode(PIN_TURNLEFT, INPUT);   // D5 <- Turn Left switch on indicator stalk
  pinMode(PIN_TURNRIGHT, INPUT);  // D6 <- Turn Right switch on indicator stalk
  pinMode(PIN_ACC, INPUT);        // D8 <- Accessories key switch
  pinMode(PIN_IGNITION, INPUT);   // D9 <- IG (Ignition key switch / engine run)
  pinMode(PIN_PARKLIGHTS, INPUT); // D10 <- Park lights

  expander1.begin();
  expander2.begin();
}

uint8_t last_PINB = 0;
uint8_t last_PIND = 0;
uint8_t last_expander1 = 0;
uint8_t last_expander2 = 0;

/*
 * TASK:  Debounce physical inputs.
 */
void debounceInputs()
{
  // Debounce D8/D9/D10
  debouncePB.ButtonProcess(PINB);
  // Debounce D5/D6
  debouncePD.ButtonProcess(PIND);
  // Debounce PCF8574 #1
  debouncePCF1.ButtonProcess(expander1.read8());
  // Debounce PCF8574 #2
  debouncePCF2.ButtonProcess(expander2.read8());

  if (PINB != last_PINB) {
    last_PINB = PINB;
    Serial.print("PINB: ");
    Serial.println(PINB);
  }
  if (PIND != last_PIND) {
    last_PIND = PIND;
    Serial.print("PIND: ");
    Serial.println(last_PIND);
  }
  if (expander1.read8() != last_expander1) {
    last_expander1 = expander1.read8();
    Serial.print("expander1: ");
    Serial.println(last_expander1);
  }
  if (expander2.read8() != last_expander2) {
    last_expander2 = expander2.read8();
    Serial.print("expander2: ");
    Serial.println(last_expander2);
  }
}

/*
 * TASK:  Changes in debounced inputs map to changes in vehicle state.
 */
void processInputs()
{
  bool debouncedValue;
  bool debouncedValue2;

  debouncedValue = debouncePD.ButtonCurrent(DIN_TURNLEFT);
  debouncedValue2 = debouncePD.ButtonCurrent(DIN_TURNRIGHT);

  // Left & right turn signals at once means 'Hazard lights'
  if (debouncedValue && debouncedValue2) {
    if (!vehicle.switches[SW_HAZARD]) {
      vehicle.setSwitch(SW_HAZARD, true);
      queueCanMessage(CAN_TYPE_SWITCH, SW_HAZARD);
    }
    // If Hazards go on, L&R go off
    if (vehicle.switches[SW_TURNLEFT]) {
      vehicle.setSwitch(SW_TURNLEFT, false);
      queueCanMessage(CAN_TYPE_SWITCH, SW_TURNLEFT);
    }
    if (vehicle.switches[SW_TURNRIGHT]) {
      vehicle.setSwitch(SW_TURNRIGHT, false);
      queueCanMessage(CAN_TYPE_SWITCH, SW_TURNRIGHT);
    }
  } else {
    // Hazards go off
    if (vehicle.switches[SW_HAZARD]) {
      vehicle.setSwitch(SW_HAZARD, false);
      queueCanMessage(CAN_TYPE_SWITCH, SW_HAZARD);
    }
    // Check L&R inputs
    if (vehicle.switches[SW_TURNLEFT] != debouncedValue) {
      vehicle.setSwitch(SW_TURNLEFT, debouncedValue);
      queueCanMessage(CAN_TYPE_SWITCH, SW_TURNLEFT);
    }
    if (vehicle.switches[SW_TURNRIGHT] != debouncedValue2) {
      vehicle.setSwitch(SW_TURNRIGHT, debouncedValue2);
      queueCanMessage(CAN_TYPE_SWITCH, SW_TURNRIGHT);
    }
  }

  debouncedValue = !debouncePB.ButtonCurrent(DIN_ACC);
  if (vehicle.switches[SW_ACC] != debouncedValue) {
    vehicle.setSwitch(SW_ACC, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_ACC);
  }

  debouncedValue = !debouncePB.ButtonCurrent(DIN_IGNITION);
  if (vehicle.switches[SW_IGNITION] != debouncedValue) {
    vehicle.setSwitch(SW_IGNITION, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_IGNITION);
  }

  debouncedValue = !debouncePB.ButtonCurrent(DIN_PARKLIGHTS);
  if (vehicle.switches[SW_PARKLIGHTS] != debouncedValue) {
    vehicle.setSwitch(SW_PARKLIGHTS, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_PARKLIGHTS);
  }

  // PCF8574 #1 inputs

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_BRAKESWITCH);
  if (vehicle.switches[SW_BRAKE] != debouncedValue) {
    vehicle.setSwitch(SW_BRAKE, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_BRAKE);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_FUELLOW);
  if (vehicle.switches[SW_WARN_FUEL] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_FUEL, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WARN_FUEL);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_CHOKE);
  if (vehicle.switches[SW_CHOKE] != debouncedValue) {
    vehicle.setSwitch(SW_CHOKE, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_CHOKE);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_COOLANTLOW);
  if (vehicle.switches[SW_WARN_COOLANTLEVEL] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_COOLANTLEVEL, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WARN_COOLANTLEVEL);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_OILPRESSURELOW);
  if (vehicle.switches[SW_WARN_OILPRESSURE] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_OILPRESSURE, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WARN_OILPRESSURE);
  }

  debouncedValue = !debouncePCF1.ButtonCurrent(DIN_TURNLEFTLIGHT); // invert
  if (vehicle.switches[SW_WARN_TURNLEFT] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_TURNLEFT, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WARN_TURNLEFT);
  }

  debouncedValue = !debouncePCF1.ButtonCurrent(DIN_TURNRIGHTLIGHT); // invert
  if (vehicle.switches[SW_WARN_TURNRIGHT] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_TURNRIGHT, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WARN_TURNRIGHT);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_WIPERINT);
  if (vehicle.switches[SW_WIPERS_INT] != debouncedValue) {
    vehicle.setSwitch(SW_WIPERS_INT, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WIPERS_INT);
  }

  // PCF8574 #2 inputs

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_WASHERMOTOR);
  if (vehicle.switches[SW_WASHER] != debouncedValue) {
    vehicle.setSwitch(SW_WASHER, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WASHER);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_WIPERLOW);
  if (vehicle.switches[SW_WIPERS_LOW] != debouncedValue) {
    vehicle.setSwitch(SW_WIPERS_LOW, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WIPERS_LOW);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_WIPERHIGH);
  if (vehicle.switches[SW_WIPERS_HIGH] != debouncedValue) {
    vehicle.setSwitch(SW_WIPERS_HIGH, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WIPERS_HIGH);
  }

  // Not sure what this pin does!
  debouncedValue = debouncePCF2.ButtonCurrent(DIN_UNKNOWN);
  if (vehicle.switches[SW_UNKNOWN] != debouncedValue) {
    vehicle.setSwitch(SW_UNKNOWN, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_UNKNOWN);
  }

  debouncedValue = !debouncePCF2.ButtonCurrent(DIN_HEADLIGHTS);
  if (vehicle.switches[SW_HEADLIGHTS] != debouncedValue) {
    vehicle.setSwitch(SW_HEADLIGHTS, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_HEADLIGHTS);
  }

  debouncedValue = !debouncePCF2.ButtonCurrent(DIN_HIGHBEAM);
  if (vehicle.switches[SW_HIGHBEAM] != debouncedValue) {
    vehicle.setSwitch(SW_HIGHBEAM, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_HIGHBEAM);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_HANDBRAKE);
  if (vehicle.switches[SW_HANDBRAKE] != debouncedValue) {
    vehicle.setSwitch(SW_HANDBRAKE, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_HANDBRAKE);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_BRAKEFLUIDLOW);
  if (vehicle.switches[SW_WARN_BRAKEFLUID] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_BRAKEFLUID, debouncedValue);
    queueCanMessage(CAN_TYPE_SWITCH, SW_WARN_BRAKEFLUID);
  }
}

// --------------------------------------------------------------------
// Arduino setup
// --------------------------------------------------------------------

void setup()
{
  setupInputs();
  setupCanbus();
}

void loop()
{
  processCanMessage();
  debounceInputs();
  processInputs();
  delay(10);
}

