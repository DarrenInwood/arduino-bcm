//#include <avr_heap.h>
//#include <nil.h>
//#include <nilconf.h>
//#include <nilcore.h>
//#include <niltypes.h>
//#include <NilFIFO.h>
#include <NilRTOS.h>
//#include <NilSerial.h>
//#define Serial NilSerial

#include <Wire.h>
#include "PCF8574.h"
PCF8574 expander1(0x20);
PCF8574 expander2(0x21);

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#define CAN_CS_PIN    4
#define CAN_INT_PIN   2
MCP_CAN CAN(CAN_CS_PIN);

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

const size_t CAN_FIFO_SIZE_BYTES = 128;
const uint16_t CAN_FIFO_SIZE = CAN_FIFO_SIZE_BYTES/sizeof(CanFrame));

// nilfifo?

#include "Vehicle.h"
#include "VehicleDefs.h"
Vehicle vehicle;

// Tells us which input is connected where
#include "MainLoomDefs.h"

// Button debouncers
#include "button_debounce.h"
Debouncer debouncePB(0x00); // Constructor takes 'pulledUpPins' - ie. normally high = 1, ie. "on" = 0V before 4049
Debouncer debouncePD(DIN_TURNLEFT & DIN_TURNRIGHT);
Debouncer debouncePCF1(DIN_BRAKESWITCH & DIN_FUELLOW & DIN_CHOKE & DIN_COOLANTLOW & DIN_OILPRESSURELOW & DIN_WIPERINT);
Debouncer debouncePCF2(DIN_WASHERMOTOR & DIN_WIPERLOW & DIN_WIPERHIGH & DIN_UNKNOWN);

// DEBOUNCE INPUTS
NIL_WORKING_AREA(waDebounce, 64);
NIL_THREAD(Debounce, arg) {
  while (1) {
    // Debounce D8/D9/D10
    debouncePB.ButtonProcess(PINB); // pullups on all ins
    // Debounce D5/D6
    debouncePD.ButtonProcess(PIND);
    // Debounce PCF8574 #1
    debouncePCF1.ButtonProcess(expander1.read8());
    // Debounce PCF8574 #2
    debouncePCF2.ButtonProcess(expander2.read8());
    // Debounce every 10ms
    nilThdSleepMilliseconds(10);
  }
}

NIL_WORKING_AREA(waMeasureInputs, 64);
NIL_THREAD(MeasureInputs, arg) {

  // Let debouncers warm up first
  nilThdSleepMilliseconds(50);
  
  while (1) {
    // Sync vehicleSwitches and vehicleValues to actual inputs
    bool debouncedValue = debouncePD.ButtonCurrent(DIN_TURNLEFT);
    bool debouncedValue2 = debouncePD.ButtonCurrent(DIN_TURNRIGHT);

    // Left & right turn signals at once means 'Hazard lights'
    if (debouncedValue && debouncedValue2) {
      if (!vehicle.switches[SW_HAZARD]) {
        vehicle.setSwitch(SW_HAZARD, true);
      }
      // If Hazards go on, L&R go off
      if (vehicle.switches[SW_TURNLEFT]) {
        vehicle.setSwitch(SW_TURNLEFT, false);
      }
      if (vehicle.switches[SW_TURNRIGHT]) {
        vehicle.setSwitch(SW_TURNRIGHT, false);
      }
    } else {
      // Hazards go off
      if (vehicle.switches[SW_HAZARD]) {
        vehicle.setSwitch(SW_HAZARD, false);
      }
      // Check L&R inputs
      if (vehicle.switches[SW_TURNLEFT] != debouncedValue) {
        vehicle.setSwitch(SW_TURNLEFT, debouncedValue);
      }
      if (vehicle.switches[SW_TURNRIGHT] != debouncedValue2) {
        vehicle.setSwitch(SW_TURNRIGHT, debouncedValue2);
      }
    }

    debouncedValue = !debouncePB.ButtonCurrent(DIN_ACC);
    vehicle.setSwitch(SW_ACC, debouncedValue);

    debouncedValue = !debouncePB.ButtonCurrent(DIN_IGNITION);
    vehicle.setSwitch(SW_IGNITION, debouncedValue);

    debouncedValue = !debouncePB.ButtonCurrent(DIN_PARKLIGHTS);
    vehicle.setSwitch(SW_PARKLIGHTS, debouncedValue);

//    // PCF8574 #1 inputs

    debouncedValue = debouncePCF1.ButtonCurrent(DIN_BRAKESWITCH);
    vehicle.setSwitch(SW_BRAKE, debouncedValue);
    
    debouncedValue = debouncePCF1.ButtonCurrent(DIN_FUELLOW);
    vehicle.setSwitch(SW_WARN_FUEL, debouncedValue);

    debouncedValue = debouncePCF1.ButtonCurrent(DIN_CHOKE);
    vehicle.setSwitch(SW_CHOKE, debouncedValue);

    debouncedValue = debouncePCF1.ButtonCurrent(DIN_COOLANTLOW);
    vehicle.setSwitch(SW_WARN_COOLANTLEVEL, debouncedValue);
    
    debouncedValue = debouncePCF1.ButtonCurrent(DIN_OILPRESSURELOW);
    vehicle.setSwitch(SW_WARN_OILPRESSURE, debouncedValue);

    debouncedValue = !debouncePCF1.ButtonCurrent(DIN_TURNLEFTLIGHT); // invert
    vehicle.setSwitch(SW_WARN_TURNLEFT, debouncedValue);

    debouncedValue = !debouncePCF1.ButtonCurrent(DIN_TURNRIGHTLIGHT); // invert
    vehicle.setSwitch(SW_WARN_TURNRIGHT, debouncedValue);

    debouncedValue = debouncePCF1.ButtonCurrent(DIN_WIPERINT);
    vehicle.setSwitch(SW_WIPERS_INT, debouncedValue);

//    // PCF8574 #2 inputs

    debouncedValue = debouncePCF2.ButtonCurrent(DIN_WASHERMOTOR);
    vehicle.setSwitch(SW_WASHER, debouncedValue);

    debouncedValue = debouncePCF2.ButtonCurrent(DIN_WIPERLOW);
    vehicle.setSwitch(SW_WIPERS_LOW, debouncedValue);
    
    debouncedValue = debouncePCF2.ButtonCurrent(DIN_WIPERHIGH);
    vehicle.setSwitch(SW_WIPERS_HIGH, debouncedValue);
    
    // Not sure what this pin does!
    debouncedValue = debouncePCF2.ButtonCurrent(DIN_UNKNOWN);
    vehicle.setSwitch(SW_UNKNOWN, debouncedValue);

    debouncedValue = !debouncePCF2.ButtonCurrent(DIN_HEADLIGHTS);
    vehicle.setSwitch(SW_HEADLIGHTS, debouncedValue);
    
    debouncedValue = !debouncePCF2.ButtonCurrent(DIN_HIGHBEAM);
    vehicle.setSwitch(SW_HIGHBEAM, debouncedValue);

    debouncedValue = debouncePCF2.ButtonCurrent(DIN_HANDBRAKE);
    vehicle.setSwitch(SW_HANDBRAKE, debouncedValue);

    debouncedValue = debouncePCF2.ButtonCurrent(DIN_BRAKEFLUIDLOW);
    vehicle.setSwitch(SW_WARN_BRAKEFLUID, debouncedValue);
    // Sleep for 10ms
    nilThdSleepMilliseconds(10);
  }
}

NIL_WORKING_AREA(waCanTx, 64);
NIL_THREAD(CanTx, arg) {
  while (1) {
    nilSemWait(&canTxSem);

    // Let something else use the CAN bus
    nilThdSleep(100);
  }
}

NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY(NULL, Debounce, NULL, waDebounce, sizeof(waDebounce))
NIL_THREADS_TABLE_ENTRY(NULL, MeasureInputs, NULL, waMeasureInputs, sizeof(waMeasureInputs))
NIL_THREADS_TABLE_ENTRY(NULL, CanTx, NULL, waCanTx, sizeof(waCanTx))
NIL_THREADS_TABLE_END()

void setup() {
  Serial.begin(9600);
  Serial.println("GO!");
  
  setupInputs();
  nilSysBegin();
}

void setupInputs() {
    // Set pin modes   
    pinMode(PIN_TURNLEFT, INPUT);   // D5 <- Turn Left switch on indicator stalk
    pinMode(PIN_TURNRIGHT, INPUT);  // D6 <- Turn Right switch on indicator stalk
    pinMode(PIN_ACC, INPUT);        // D8 <- Accessories key switch
    pinMode(PIN_IGNITION, INPUT);   // D9 <- IG (Ignition key switch / engine run)
    pinMode(PIN_PARKLIGHTS, INPUT); // D10 <- Park lights
}

// Idle thread
void loop() {
  /* */

}

