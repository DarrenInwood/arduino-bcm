#ifndef VEHICLEDEFS_H
#define VEHICLEDEFS_H

// Amount of milliseconds that we should stay in waking state before
// going back to sleep - lets us debounce stray wakeup interrupts that
// never actually amount to anything
#define TIMEOUT_WAKING_MILLIS   150

// We are setting things up
#define STATE_STARTING 0
// We should be running
#define STATE_RUNNING  1
// We should be sleeping
#define STATE_SLEEPING 2
// We are currently waking up from sleep
#define STATE_WAKING   3

// Number of switches total
#define SW_COUNT             22
// Which switch is which index in the vehicle boolean state array
// Indexes must be 0-126 for the CAN protocol to work
#define SW_ACC                0
#define SW_IGNITION           1
#define SW_PARKLIGHTS         2
#define SW_HEADLIGHTS         3
#define SW_HIGHBEAM           4
#define SW_TURNLEFT           5
#define SW_TURNRIGHT          6
#define SW_HAZARD             7
#define SW_WIPERS_INT         8
#define SW_WIPERS_LOW         9
#define SW_WIPERS_HIGH       10
#define SW_UNKNOWN           11
#define SW_WASHER            12
#define SW_BRAKE             13
#define SW_HANDBRAKE         14
#define SW_CHOKE             15
#define SW_WARN_BRAKEFLUID   16
#define SW_WARN_FUEL         17
#define SW_WARN_COOLANTLEVEL 18
#define SW_WARN_OILPRESSURE  19  
#define SW_WARN_TURNLEFT     20
#define SW_WARN_TURNRIGHT    21

// Number of values total
#define VAL_COUNT             4
// Which value is which index in the vehicle int state array
// Indexes must be 0-126 for CAN protocol to work
#define VAL_FUEL              0
#define VAL_ALTERNATOR_VOLTS  1
#define VAL_COOLANT_TEMP      2
#define VAL_SPEED             3

#endif

