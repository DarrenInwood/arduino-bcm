/*
 * Pro mini programming with Uno:
 * Use settings Programmer = AVRISP mkII
 * Press reset button on pro mini shortly after IDE starts trying to upload.
 */

#define DEBUG 1

// Tell our CAN bus interface which pins to use
#define CAN_CS_PIN    4
#define CAN_INT_PIN   2

#include "Vehicle.h"

Vehicle vehicle;

// Tells us which input is connected where
#include "MainLoomDefs.h"

// PCF8574 I/O expanders

#include <Wire.h>    // Required for I2C communication
#include "PCF8574.h" // Required for PCF8574

PCF8574 expander1;
PCF8574 expander2;

// Debounce inputs

// Should we measure the inputs on the next trip through loop()?
bool measureInputsFlag = true;

// Number of milliseconds between measurements
#define MEASURE_MILLIS 50

// Number of milliseconds since last measurement
uint8_t measureCount = 0;

// Number of milliseconds between debounces
#define DEBOUNCE_MILLIS 5

// Number of milliseconds since last debounce
uint8_t debounceCount = 0;

// http://www.ganssle.com/debouncing-pt2.htm
#include "button_debounce.h"

//// Debounce D8/9/10 
//Debouncer debouncePB(~(DIN_ACC & DIN_IGNITION & DIN_PARKLIGHTS)); // Constructor takes 'pulledUpPins' - ie. normally high = 1
//// Debounce D5/D6
//Debouncer debouncePD(DIN_TURNLEFT & DIN_TURNRIGHT);
//// Debounce PCF8574 #1
//Debouncer debouncePCF1(~(DIN_TURNLEFTLIGHT & DIN_TURNRIGHTLIGHT));
//// Debounce PCF8574 #1
//Debouncer debouncePCF2(~(DIN_HEADLIGHTS & DIN_HIGHBEAM));

// Debounce D8/9/10 
Debouncer debouncePB(0x00); // Constructor takes 'pulledUpPins' - ie. normally high = 1, ie. "on" = 0V before 4049
// Debounce D5/D6
Debouncer debouncePD(DIN_TURNLEFT & DIN_TURNRIGHT);
// Debounce PCF8574 #1
Debouncer debouncePCF1(DIN_BRAKESWITCH & DIN_FUELLOW & DIN_CHOKE & DIN_COOLANTLOW & DIN_OILPRESSURELOW & DIN_WIPERINT);
// Debounce PCF8574 #1
Debouncer debouncePCF2(DIN_WASHERMOTOR & DIN_WIPERLOW & DIN_WIPERHIGH & DIN_UNKNOWN);

// Set up the application
void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");

    setupInputs();

    // Trigger an interrupt 
    OCR0A = 0xAF;  // when Timer0 equals this value, A timer interrupt is triggered
    TIMSK0 |= _BV(OCIE0A); // Enable the Timer0 compare interrupt

    enableWakeupInterrupts();

    // set up CAN bus etc
    vehicle.setup();
}

void setupInputs() {
    // Set addresses for I/O expanders
    expander1.begin(0x20);
    expander2.begin(0x21); 

    // Set pin modes   
    pinMode(PIN_TURNLEFT, INPUT);   // D5 <- Turn Left switch on indicator stalk
    pinMode(PIN_TURNRIGHT, INPUT);  // D6 <- Turn Right switch on indicator stalk
    pinMode(PIN_ACC, INPUT);        // D8 <- Accessories key switch
    pinMode(PIN_IGNITION, INPUT);   // D9 <- IG (Ignition key switch / engine run)
    pinMode(PIN_PARKLIGHTS, INPUT); // D10 <- Park lights
}

void enableWakeupInterrupts() {
    // Pin change on any pins in D8-D13 trigger pin change interrupt
    // We have to use the version in the PCF8574 library or we get errors.
    expander1.enableInterrupt(5, wakeup_ISR);
    expander1.enableInterrupt(6, wakeup_ISR);
    expander1.enableInterrupt(8, wakeup_ISR);
    expander1.enableInterrupt(9, wakeup_ISR);
    expander1.enableInterrupt(10, wakeup_ISR);
}

// Wake up interrupt handler
void wakeup_ISR() {
    // noop - loop() will take care of everything
}

void loop() {
    // Debounce our input values and set them onto the vehicle
    if (measureInputsFlag) {
        measureInputsFlag = false;
        debounceCount++;
        measureCount++;
        if (debounceCount > DEBOUNCE_MILLIS) {
            debounceCount = 0;
            debounceInputs();
        }
        if (measureCount > MEASURE_MILLIS) {
            measureCount = 0;
            measureInputs();
        }
    }
    // Let the vehicle handle comms, sleep etc
    vehicle.process();
}

// Timer compare interrupt has happened
SIGNAL(TIMER0_COMPA_vect) 
{
    measureInputsFlag = true;
}

void debounceInputs()
{
    // Debounce D8/D9/D10
    debouncePB.ButtonProcess(PINB); // pullups on all ins
    // Debounce D5/D6
    debouncePD.ButtonProcess(PIND);
    // Debounce PCF8574 #1
    debouncePCF1.ButtonProcess(expander1.read());
    // Debounce PCF8574 #2
    debouncePCF2.ButtonProcess(expander2.read());
}

// Measure and debounce our inputs, move the values into the vehicle state.
// Note: inputs that go to 0V -> read() == 0x01
//       inputs that go to 12V -> read() == 0x00, ie. negate these
void measureInputs() {
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
}



