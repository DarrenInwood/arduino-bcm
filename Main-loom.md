# Arduino BCM System

## Main loom

To identify all the signals I had to work with at the main loom, I acquired a
copy of the workshop manual for my vehicle which included the wiring diagram.

You may be able to find a copy for your vehicle in a second hand bookshop, or
even online as a PDF.

### Inputs by function

Starting/Charging system:
*  ACC (Accessory - 12V=on)
*  IG (motor run - 12V=on)
*  St (Starter motor - 12V=on)

Ignition system:
*  Tachometer (12V square wave from coil)

Front wiper & washer, rear wiper & washer:
*  Front wiper Low (connected to ground=on)
*  Front wiper high (connected to ground=on)
*  Front wiper one-shot (connects low to ground in parallel)
*  Front wiper intermittent (connected to ground=on)
*  Front washer (connected to ground=on)
*  Rear wiper (connected to ground=on)
*  Rear washer (connected to ground=on)

Meters & warning lights:
*  Water temp gauge (resistance to ground)
*  Fuel sender gauge (resistance to ground)
*  Fuel low light (connected to ground=on)
*  Alternator
*  Brake fluid low level sensor (connected to ground=on)
*  Parking brake switch (connected to ground=on)
*  Choke switch (connected to ground=on)
*  Oil pressure switch (connected to ground=on)
*  Coolant level sensor (connected to ground via transistor=on)
*  Headlights on (12V=on)
*  Turn light L (12V=on)
*  Turn light R (12V=on)
*  High beam (12V=on)

Lights, horn:
*  Reverse switch (12V=on)
*  Hazard lights (12V=on)
*  Turn L and Turn R stalk switches have to be detected via turn signal and hazard signal at the same time, with a delay to detect whether there is another pulse from the switcher unit.  There is no actual switched signal available to detect this.
*  Stop light switch (12V=on)
*  Horn switch (connected to ground=on)

Heater:
*  Voltage in (switch is a series of switched resistors)

### Inputs by type

12V on signals: (11 of)
*  ACC (Accessory)
*  IG (motor run)
*  Starter motor
*  Headlights on
*  High beam
*  Turn L light
*  Turn R light
*  Reverse switch
*  Hazard lights
*  Stop light
*  Tachometer (square wave)

Ground on signals: (11 of)
*  Front wiper low
*  Front wiper High
*  Front wiper intermittent
*  Front washer
*  Rear wiper (N/A - removed)
*  Rear washer (N/A - removed)
*  Fuel low light
*  Brake fluid low sensor
*  Parking Brake
*  Choke switch
*  Oil pressure warning switch
*  Horn
*  Coolant low level sensor (via transistor)

Analog signals: (3 of)
*  Water temperature
*  Fuel sender
*  Heater level

## Unit design

I determined that I have 22 on/off signals, and two analog things to
measure.  For this I will use two PCF8574A units providing 16 channels of I/O
via I2C, and 6 channels of digital I/O on the Arduino, plus two channels of
analog I/O.

To save power, the unit should sleep most of the time, but wake up whenever
something happens.  As most of the signals are switched off the ACC line anyway,
we only really need to wake up for a few signals:

*  ACC (stay on as long as this is on)
*  headlights on state change (send messages when this turns on/off)
*  hazard lights (send messages when this turns on/off)
*  Horn (send messages when this turns on/off)

We can use a 'Pin Change Interrupt' to wake up from sleep mode, which is
triggered by a change in state on an input pin.  For the Arduino Uno, there are
three Pin Change Interrupts, in groups:

    Pin Change Interrupt Request 0 (pins D8 to D13) (PCINT0_vect)
    Pin Change Interrupt Request 1 (pins A0 to A5)  (PCINT1_vect)
    Pin Change Interrupt Request 2 (pins D0 to D7)  (PCINT2_vect)

If we connect our four interrupt-generating inputs to D8-D13 they will all
generate a wake up interrupt.

Some of our inputs need series diodes and pull-up resistors (connect-to-ground)
to work correctly.  The pullup resistor will ensure that the input lives at 5V
when it's not grounded.  The series diode between the input and the Arduino pin
ensures that we're not going to draw current between the 12V supply that will be
on the input wire, and the 5V that our pin will be floating at when the input isn't grounded.

The others will need a step-down circuit to ensure we don't destroy our inputs
by applying more than 5V.  We will use CD4049 Hex Inverter chips, as they let us
accept voltages up to 20V and give a nice steady 5V on/off output.  Luckily we
have 11 signals so can do this with just two hex inverters.
