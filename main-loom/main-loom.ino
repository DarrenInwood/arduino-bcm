#include <SCoop.h>

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
// PCF8574 I/O expanders
// --------------------------------------------------------------------
#include <Wire.h>
#include "PCF8574.h"
PCF8574 expander1(0x20);
PCF8574 expander2(0x21);

// --------------------------------------------------------------------
// Digital input debouncing
// --------------------------------------------------------------------
#include "button_debounce.h"
Debouncer debouncePB(0x00); // Constructor takes 'pulledUpPins' - ie. normally high = 1, ie. "on" = 0V before 4049
Debouncer debouncePD(DIN_TURNLEFT & DIN_TURNRIGHT);
Debouncer debouncePCF1(DIN_BRAKESWITCH & DIN_FUELLOW & DIN_CHOKE & DIN_COOLANTLOW & DIN_OILPRESSURELOW & DIN_WIPERINT);
Debouncer debouncePCF2(DIN_WASHERMOTOR & DIN_WIPERLOW & DIN_WIPERHIGH & DIN_UNKNOWN);

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
}

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
       case 0:
           // Don't send a can message if we changed state
           vehicle.setSwitch(buf.parts.index, buf.parts.val);
       break;
       case 1:
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

/*
 * TASK:  Debounce physical inputs.
 */
defineTaskLoop(processInputs, 64)
{
  // Debounce D8/D9/D10
  debouncePB.ButtonProcess(PINB);
  // Debounce D5/D6
  debouncePD.ButtonProcess(PIND);
  // Debounce PCF8574 #1
  debouncePCF1.ButtonProcess(expander1.read8());
  // Debounce PCF8574 #2
  debouncePCF2.ButtonProcess(expander2.read8());
  // Debounce every 10ms
  sleep(10);
}

/*
 * TASK:  Changes in debounced inputs map to changes in vehicle state.
 */
volatile bool debouncedValue;
volatile bool debouncedValue2;
defineTaskLoop(copyInputsToState, 128)
{
  debouncedValue = debouncePD.ButtonCurrent(DIN_TURNLEFT);
  debouncedValue2 = debouncePD.ButtonCurrent(DIN_TURNRIGHT);

  // Left & right turn signals at once means 'Hazard lights'
  if (debouncedValue && debouncedValue2) {
    if (!vehicle.switches[SW_HAZARD]) {
      vehicle.setSwitch(SW_HAZARD, true);
      queueCanSwitch(SW_HAZARD);
    }
    // If Hazards go on, L&R go off
    if (vehicle.switches[SW_TURNLEFT]) {
      vehicle.setSwitch(SW_TURNLEFT, false);
      queueCanSwitch(SW_TURNLEFT);
    }
    if (vehicle.switches[SW_TURNRIGHT]) {
      vehicle.setSwitch(SW_TURNRIGHT, false);
      queueCanSwitch(SW_TURNRIGHT);
    }
  } else {
    // Hazards go off
    if (vehicle.switches[SW_HAZARD]) {
      vehicle.setSwitch(SW_HAZARD, false);
      queueCanSwitch(SW_HAZARD);
    }
    // Check L&R inputs
    if (vehicle.switches[SW_TURNLEFT] != debouncedValue) {
      vehicle.setSwitch(SW_TURNLEFT, debouncedValue);
      queueCanSwitch(SW_TURNLEFT);
    }
    if (vehicle.switches[SW_TURNRIGHT] != debouncedValue2) {
      vehicle.setSwitch(SW_TURNRIGHT, debouncedValue2);
      queueCanSwitch(SW_TURNRIGHT);
    }
  }

  debouncedValue = !debouncePB.ButtonCurrent(DIN_ACC);
  if (vehicle.switches[SW_ACC] != debouncedValue) {
    vehicle.setSwitch(SW_ACC, debouncedValue);
    queueCanSwitch(SW_ACC);
  }

  debouncedValue = !debouncePB.ButtonCurrent(DIN_IGNITION);
  if (vehicle.switches[SW_IGNITION] != debouncedValue) {
    vehicle.setSwitch(SW_IGNITION, debouncedValue);
    queueCanSwitch(SW_IGNITION);
  }

  debouncedValue = !debouncePB.ButtonCurrent(DIN_PARKLIGHTS);
  if (vehicle.switches[SW_PARKLIGHTS] != debouncedValue) {
    vehicle.setSwitch(SW_PARKLIGHTS, debouncedValue);
    queueCanSwitch(SW_PARKLIGHTS);
  }

  // PCF8574 #1 inputs

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_BRAKESWITCH);
  if (vehicle.switches[SW_BRAKE] != debouncedValue) {
    vehicle.setSwitch(SW_BRAKE, debouncedValue);
    queueCanSwitch(SW_BRAKE);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_FUELLOW);
  if (vehicle.switches[SW_WARN_FUEL] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_FUEL, debouncedValue);
    queueCanSwitch(SW_WARN_FUEL);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_CHOKE);
  if (vehicle.switches[SW_CHOKE] != debouncedValue) {
    vehicle.setSwitch(SW_CHOKE, debouncedValue);
    queueCanSwitch(SW_CHOKE);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_COOLANTLOW);
  if (vehicle.switches[SW_WARN_COOLANTLEVEL] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_COOLANTLEVEL, debouncedValue);
    queueCanSwitch(SW_WARN_COOLANTLEVEL);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_OILPRESSURELOW);
  if (vehicle.switches[SW_WARN_OILPRESSURE] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_OILPRESSURE, debouncedValue);
    queueCanSwitch(SW_WARN_OILPRESSURE);
  }

  debouncedValue = !debouncePCF1.ButtonCurrent(DIN_TURNLEFTLIGHT); // invert
  if (vehicle.switches[SW_WARN_TURNLEFT] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_TURNLEFT, debouncedValue);
    queueCanSwitch(SW_WARN_TURNLEFT);
  }

  debouncedValue = !debouncePCF1.ButtonCurrent(DIN_TURNRIGHTLIGHT); // invert
  if (vehicle.switches[SW_WARN_TURNRIGHT] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_TURNRIGHT, debouncedValue);
    queueCanSwitch(SW_WARN_TURNRIGHT);
  }

  debouncedValue = debouncePCF1.ButtonCurrent(DIN_WIPERINT);
  if (vehicle.switches[SW_WIPERS_INT] != debouncedValue) {
    vehicle.setSwitch(SW_WIPERS_INT, debouncedValue);
    queueCanSwitch(SW_WIPERS_INT);
  }

  // PCF8574 #2 inputs

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_WASHERMOTOR);
  if (vehicle.switches[SW_WASHER] != debouncedValue) {
    vehicle.setSwitch(SW_WASHER, debouncedValue);
    queueCanSwitch(SW_WASHER);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_WIPERLOW);
  if (vehicle.switches[SW_WIPERS_LOW] != debouncedValue) {
    vehicle.setSwitch(SW_WIPERS_LOW, debouncedValue);
    queueCanSwitch(SW_WIPERS_LOW);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_WIPERHIGH);
  if (vehicle.switches[SW_WIPERS_HIGH] != debouncedValue) {
    vehicle.setSwitch(SW_WIPERS_HIGH, debouncedValue);
    queueCanSwitch(SW_WIPERS_HIGH);
  }

  // Not sure what this pin does!
  debouncedValue = debouncePCF2.ButtonCurrent(DIN_UNKNOWN);
  if (vehicle.switches[SW_UNKNOWN] != debouncedValue) {
    vehicle.setSwitch(SW_UNKNOWN, debouncedValue);
    queueCanSwitch(SW_UNKNOWN);
  }

  debouncedValue = !debouncePCF2.ButtonCurrent(DIN_HEADLIGHTS);
  if (vehicle.switches[SW_HEADLIGHTS] != debouncedValue) {
    vehicle.setSwitch(SW_HEADLIGHTS, debouncedValue);
    queueCanSwitch(SW_HEADLIGHTS);
  }

  debouncedValue = !debouncePCF2.ButtonCurrent(DIN_HIGHBEAM);
  if (vehicle.switches[SW_HIGHBEAM] != debouncedValue) {
    vehicle.setSwitch(SW_HIGHBEAM, debouncedValue);
    queueCanSwitch(SW_HIGHBEAM);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_HANDBRAKE);
  if (vehicle.switches[SW_HANDBRAKE] != debouncedValue) {
    vehicle.setSwitch(SW_HANDBRAKE, debouncedValue);
    queueCanSwitch(SW_HANDBRAKE);
  }

  debouncedValue = debouncePCF2.ButtonCurrent(DIN_BRAKEFLUIDLOW);
  if (vehicle.switches[SW_WARN_BRAKEFLUID] != debouncedValue) {
    vehicle.setSwitch(SW_WARN_BRAKEFLUID, debouncedValue);
    queueCanSwitch(SW_WARN_BRAKEFLUID);
  }

  // Sleep for 10ms
  sleep(10);
}

// --------------------------------------------------------------------
// Arduino setup
// --------------------------------------------------------------------

void setup()
{
  setupInputs();
  setupCanbus();

  mySCoop.start();
}

void loop()
{
  // Empty  
}

