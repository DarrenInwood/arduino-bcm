#include "Arduino.h"
#include "Vehicle.h"

#define DEBUG

// https://github.com/rocketscream/Low-Power
#include <LowPower.h>

// http://www.seeedstudio.com/depot/wifi-bee-v20-p-1637.html
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

#ifndef CAN_MSG_ID
#define CAN_MSG_ID    0x32
#endif

#ifndef CAN_CS_PIN
#define CAN_CS_PIN    4
#endif

#ifndef CAN_INT_PIN
#define CAN_INT_PIN   2
#endif

MCP_CAN CAN(CAN_CS_PIN);

// Constructor
Vehicle::Vehicle()
{
    state = STATE_WAKING;
}

void Vehicle::setup()
{
START_INIT:

    if(CAN_OK == CAN.begin(CAN_125KBPS))
    {
#ifdef DEBUG
        Serial.println("CAN BUS Shield init ok!");
#endif
    }
    else
    {
#ifdef DEBUG
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
#endif
        delay(100);
        goto START_INIT;
    }
}

void Vehicle::process()
{
    // Did we just wake up?
    if (state == STATE_WAKING) {        
        // Sync the peripherals
    }

    // If there is a CAN message to receieve, buffer it
    receiveCanMessage();

    // If we have any CAN messages to send, send one
    sendCanMessage();

    // Detect what state we're in according to the inputs
    state = detectState();

    if (state != STATE_SLEEPING) {
        return;
    }

    // Power down
    //enableWakeupInterrupts();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // As soon as we wake up, turn off interrupts so they don't retrigger
    //disableWakeupInterrupts();
    state = STATE_WAKING;
    wakingTime = millis();
#ifdef DEBUG
    Serial.println("Waking up...");
#endif
}

// Set state of a switch
// send a CAN message if it's changed
void Vehicle::setSwitch(uint8_t index, bool state, bool transmitCanMessage)
{
    if (switches[index] == state) {
        return; // no change
    }
    switches[index] = state;
#ifdef DEBUG
    Serial.print("Set switch: ");
    Serial.print(index);
    Serial.print(" ");
    Serial.println(state);
#endif
    if (transmitCanMessage) {
        queueCanMessage(0, index, state);
    }
}

// Set state of a value
// Send a CAN message if it's changed
void Vehicle::setValue(uint8_t index, uint16_t val, bool transmitCanMessage)
{
    if (values[index] == val) {
        return; // no change
    }
    values[index] = val;
#ifdef DEBUG
    Serial.print("Set value: ");
    Serial.print(index);
    Serial.print(" ");
    Serial.println(val);
#endif
    if (transmitCanMessage) {
      queueCanMessage(1, index, val);
    }
}

// type == 0x00 -> switch
// type == 0x01 -> value
// index == index of switch/value
// val -> value to set
void Vehicle::queueCanMessage(uint8_t type, uint8_t index, uint16_t val)
{
    if (canTxQueueLength == CAN_BUFFER_SIZE) {
        // lose packets :-(
        return;
    }
    union CanFrame buf;
    buf.parts.type = type;
    buf.parts.index = index;
    buf.parts.val = val;
    canTxQueue[canTxQueueLength] = buf.full;
    canTxQueueLength++;
#ifdef DEBUG
    Serial.print("Queued CAN message: ");
    Serial.println(buf.full, BIN);
#endif
}

// Sends a CAN message if there are any in the queue
void Vehicle::sendCanMessage()
{
    if (canTxQueueLength == 0) {
        // nothing to send
        return;
    }
    union CanFrame buf;
    canTxQueueLength--;
    buf.full = canTxQueue[canTxQueueLength];
    CAN.sendMsgBuf(CAN_MSG_ID, 0, 4, buf.data);
#ifdef DEBUG
    Serial.print("Sent CAN message: ");
    Serial.println(buf.full, BIN);
#endif
}

void Vehicle::receiveCanMessage()
{
    if (CAN_MSGAVAIL != CAN.checkReceive()) {
        return; // nothing to receive
    }
    union CanFrame buf;
    uint8_t len;
    CAN.readMsgBuf(&len, buf.data);
    // Set the new value according to the incoming message
    switch (buf.parts.type) {
        case 0:
            // Don't send a can message if we changed state
            setSwitch(buf.parts.index, buf.parts.val, false);
#ifdef DEBUG
    Serial.print("Received CAN switch message: ");
    Serial.print(buf.parts.index);
    Serial.print(" ");
    Serial.println(buf.parts.val);
#endif
        break;
        case 1:
            // Don't send a can message if we changed state
            setValue(buf.parts.index, buf.parts.val, false);
#ifdef DEBUG
    Serial.print("Received CAN value message: ");
    Serial.print(buf.parts.index);
    Serial.print(" ");
    Serial.println(buf.parts.val);
#endif
        break;
    }
}

// Detect whether we should be staying awake or not
uint8_t Vehicle::detectState()
{
    // If the accessory switch is on, we're running
    if (switches[SW_ACC]) {
        return STATE_RUNNING;
    }

    // If the engine is running, we're running
    if (switches[SW_IGNITION]) {
        return STATE_RUNNING;
    }

    // If the park lights are on, we're running
    if (switches[SW_PARKLIGHTS]) {
        return STATE_RUNNING;
    }

    // If we have a CAN message to send, stay running
    if (canTxQueueLength > 0) {
        return STATE_RUNNING;
    }

    // If we're waking up, and nothing has told us to be awake, but
    // we're inside the wakeup timeout, stay awake a bit longer...
    if (state == STATE_WAKING && (millis() - wakingTime) <= TIMEOUT_WAKING_MILLIS) {
        return STATE_WAKING;
    }

    // Default - go to sleep
    return STATE_SLEEPING;
}  


