#ifndef MAINLOOMDEFS_H
#define MAINLOOMDEFS_H

// What is physically connected where

// Digital pins directly connected
#define PIN_TURNLEFT         5
#define PIN_TURNRIGHT        6
#define PIN_ACC              8
#define PIN_IGNITION         9
#define PIN_PARKLIGHTS      10

// Port numbers for direct-connected pins
// PORTB / PINB
#define DIN_ACC              0x0001
#define DIN_IGNITION         0x0002
#define DIN_PARKLIGHTS       0x0004
// PORTD / PIND
#define DIN_TURNLEFT         0x0020
#define DIN_TURNRIGHT        0x0040

// Digital inputs on PCF8574 #1
#define DIN_BRAKESWITCH      0x0001
#define DIN_FUELLOW          0x0002
#define DIN_CHOKE            0x0004
#define DIN_COOLANTLOW       0x0008
#define DIN_OILPRESSURELOW   0x0010
#define DIN_TURNLEFTLIGHT    0x0020
#define DIN_TURNRIGHTLIGHT   0x0040
#define DIN_WIPERINT         0x0080
// Digital inputs on PCF8574 #2
#define DIN_WASHERMOTOR      0x0001
#define DIN_WIPERLOW         0x0002
#define DIN_WIPERHIGH        0x0004
#define DIN_UNKNOWN          0x0008
#define DIN_HEADLIGHTS       0x0010
#define DIN_HIGHBEAM         0x0020
#define DIN_HANDBRAKE        0x0040
#define DIN_BRAKEFLUIDLOW    0x0080


// Analog inputs
#define ADC_FUEL            A0
#define ADC_ALTERNATOR      A1
#define ADC_COOLANT         A2

#endif
