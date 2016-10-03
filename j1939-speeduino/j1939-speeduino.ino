#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "ARD1939.h"

ARD1939 j1939;

SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xJ1939Semaphore;
SemaphoreHandle_t xRegSemaphore;

// just freertos includes:
// Sketch uses 7,228 bytes (22%) of program storage space. Maximum is 32,256 bytes.
// Global variables use 159 bytes (7%) of dynamic memory, leaving 1,889 bytes for local variables. Maximum is 2,048 bytes.

// freertos includes, serial read task
// Sketch uses 8,176 bytes (25%) of program storage space. Maximum is 32,256 bytes.
// Global variables use 371 bytes (18%) of dynamic memory, leaving 1,677 bytes for local variables. Maximum is 2,048 bytes.

// freertos includes, serial read task, J1939 library, j1939 setup
// Sketch uses 9,620 bytes (29%) of program storage space. Maximum is 32,256 bytes.
// Global variables use 486 bytes (23%) of dynamic memory, leaving 1,562 bytes for local variables. Maximum is 2,048 bytes.

// freertos, serial read task, J1939 library, j1939 setup, j1939 read
// Sketch uses 12,130 bytes (37%) of program storage space. Maximum is 32,256 bytes.
// Global variables use 637 bytes (31%) of dynamic memory, leaving 1,411 bytes for local variables. Maximum is 2,048 bytes.

// freertos, serial read speeduino, J1939 library, J1939 setup, J1939 read, internal read / J1939 write
// Sketch uses 12,530 bytes (38%) of program storage space. Maximum is 32,256 bytes.
// Global variables use 651 bytes (31%) of dynamic memory, leaving 1,397 bytes for local variables. Maximum is 2,048 bytes.

// Sketch uses // freertos, serial read speeduino, J1939 libray, J1939 setup, read, internal read, semaphores
// 13,728 bytes (42%) of program storage space. Maximum is 32,256 bytes.
// Global variables use 721 bytes (35%) of dynamic memory, leaving 1,327 bytes for local variables. Maximum is 2,048 bytes.

/**
 * Number of values in the Tuner Studio serial response from Speeduino.
 * This may change from time to time as more features are added.
 */
#define MS_TABLE_LENGTH 35

/**
 * Local storage of the Speeduino values sent via the serial bus.
 */
byte regTable[MS_TABLE_LENGTH];

void setup() {

  // Set the serial interface baud rate
  Serial.begin(115200);

  // Initialize the J1939 protocol including CAN settings
  if(j1939.Init(SYSTEM_TIME) == 0) {
    Serial.print("CAN Controller Init OK.\n\r\n\r");
  } else {
    Serial.print("CAN Controller Init Failed.\n\r");
  }

  // Initialise the J1939 stack
  j1939.SetPreferredAddress(SA_PREFERRED);
  j1939.SetAddressRange(ADDRESSRANGEBOTTOM, ADDRESSRANGETOP);
  j1939.SetNAME(
    NAME_IDENTITY_NUMBER,
    NAME_MANUFACTURER_CODE,
    NAME_FUNCTION_INSTANCE,
    NAME_ECU_INSTANCE,
    NAME_FUNCTION,
    NAME_VEHICLE_SYSTEM,
    NAME_VEHICLE_SYSTEM_INSTANCE,
    NAME_INDUSTRY_GROUP,
    NAME_ARBITRARY_ADDRESS_CAPABLE
  );

  // Set up filters
  // j1939.SetMessageFilter(long lPGN1);
  // j1939.SetMessageFilter(long lPGN2);
  // j1939.SetMessageFilter(long lPGN3);
  // j1939.SetMessageFilter(long lPGN4);
  // ... up to 10

  if ( xSerialSemaphore == NULL ) {
    xSerialSemaphore = xSemaphoreCreateMutex();
    if ( (xSerialSemaphore) != NULL ) {
      xSemaphoreGive((xSerialSemaphore));
    }
  }

  if ( xJ1939Semaphore == NULL ) {
    xJ1939Semaphore = xSemaphoreCreateMutex();
    if ( (xJ1939Semaphore) != NULL ) {
      xSemaphoreGive((xJ1939Semaphore));
    }
  }

  if ( xRegSemaphore == NULL ) {
    xRegSemaphore = xSemaphoreCreateMutex();
    if ( (xRegSemaphore) != NULL ) {
      xSemaphoreGive((xRegSemaphore));
    }
  }

  xTaskCreate(
    TaskReadSpeeduinoValues,
    (const portCHAR *)"Speeduino",   // A name just for humans
    128,  // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    2,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL
  );

  xTaskCreate(
    TaskProcessJ1939,
    (const portCHAR *)"Receive",   // A name just for humans
    128,  // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    3,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL
  );

  xTaskCreate(
    TaskTransmitJ1939,
    (const portCHAR *)"Transmit",   // A name just for humans
    128,  // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    1,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL
  );
}

// (no loop)
void loop() {
}

/**
 * Periodically reads the values from the Speeduino ECU and stores them locally.
 */
void TaskReadSpeeduinoValues(void *pvParameters)
{
  (void) pvParameters;
  uint8_t counter = 0;
  byte receiveBuffer;
  bool serialAvailable = false;
  for (;;)
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
      Serial.flush();
      Serial.write('S');
      serialAvailable = Serial.available();
      xSemaphoreGive( xSerialSemaphore );
    }
    counter = 0;
    while(counter < MS_TABLE_LENGTH && serialAvailable) {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
        receiveBuffer = Serial.read();
        serialAvailable = Serial.available();
        xSemaphoreGive( xSerialSemaphore );
      }
      if ( xSemaphoreTake( xRegSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
        regTable[counter] = receiveBuffer;
        xSemaphoreGive( xRegSemaphore );
      }
      counter++;
    }
    
    // 7 * 15ms = 105ms delay between reads
    vTaskDelay(7);
  }
}

void TaskProcessJ1939(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  // J1939 Variables
  byte nMsgId;
  byte nDestAddr;
  byte nSrcAddr;
  byte nPriority;
  byte nJ1939Status;
  
  int nMsgLen;
  
  long lPGN;
  
  byte pMsg[J1939_MSGLEN]; // 8 since we've turned off TP
  
  for (;;)
  {
    // Process any waiting CAN messages
    if ( xSemaphoreTake( xJ1939Semaphore, ( TickType_t ) 3 ) == pdTRUE ) {
      nJ1939Status = j1939.Operate(&nMsgId, &lPGN, &pMsg[0], &nMsgLen, &nDestAddr, &nSrcAddr, &nPriority);
      xSemaphoreGive(xJ1939Semaphore);
    }
// We don't respond to any incoming J1939 messages.
//    if (nJ1939Status == NORMALDATATRAFFIC && nMsgId == J1939_MSG_APP) {
//        switch (lPGN) {
//          case 0x0F004:
//            // Do stuff here for each PGN we want to respond to
//            digitalWrite(LED_PIN, (bool)(pMsg[0]));
//          break;
//        }
//    }
    // The J1939 Operate() runs every 15ms (1 tick)
    vTaskDelay(1);
  }
}

void TaskTransmitJ1939(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  uint8_t counter = 0;
  for (;;) {
    if (counter%1 == 0) {
      // Every 45ms
      sendTPS();
    }
    if (counter%2 == 0) {
      // Every 100ms
      sendEngineSpeed();
    }
    if (counter%12 == 0) {
      // Every 500ms
    }
    if (counter%24 == 0) {
      // Every 1s
      sendEngineTemp();
    }
    if (counter%120 == 0) {
      // Every 5s
    }

    counter++;
    if (counter == 240) {
      counter = 0;
    }
    vTaskDelay(4); // check for changes every 45ms
  }
}

/**
 * Send the engine temperature via J1939 (Speeduino CLT value)
 */
void sendEngineTemp()
{
  uint8_t clt;
  if ( xSemaphoreTake( xRegSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
    clt = regTable[6];
    xSemaphoreGive( xRegSemaphore );
  }
  byte nSrcAddr = j1939.GetSourceAddress();
  byte pData[8] = {
    clt, // CLT
    0xFF, // Fuel temp
    0xFF, 0xFF, // Oil temp
    0xFF, 0xFF, // Turbo temp
    0xFF, // Intercooler temp
    0xFF // Intercooler thermostat opening
  };
  // Engine Temperature 1
  if ( xSemaphoreTake( xJ1939Semaphore, ( TickType_t ) 3 ) == pdTRUE ) {
    // j1939.Transmit(byte nPriority, long lPGN, byte nSourceAddress, byte nDestAddress, byte* pData, int nDataLen);
    j1939.Transmit(6, 0xFEEE, nSrcAddr, 0x7F, pData, 8);
    xSemaphoreGive(xJ1939Semaphore);
  }
}

/**
 * Send the engine speed via J1939 (Speeduino RPM value)
 */
void sendEngineSpeed()
{
  uint16_t rpm;
  if ( xSemaphoreTake( xRegSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
    rpm = (regTable[14] << 8) + regTable[13];
    xSemaphoreGive( xRegSemaphore );
  }
  byte nSrcAddr = j1939.GetSourceAddress();
  // 0.125RPM/BIT
  rpm = rpm << 3;
  byte pData[5] = {
    0xFF, // Engine torque mode
    0xFF, // Driver's demand torque
    0xFF, // Actual engine torque
    highByte(rpm), lowByte(rpm) // RPM
  };
  // EEC1 (0xF004)
  if ( xSemaphoreTake( xJ1939Semaphore, ( TickType_t ) 3 ) == pdTRUE ) {
    j1939.Transmit(6, 0xF004, nSrcAddr, 0x7F, pData, 5);
    xSemaphoreGive(xJ1939Semaphore);
  }
}

/**
 * Send the TPS sensor via J1939 (Speeduino TPS sensor)
 */
void sendTPS()
{
  uint8_t tps;
  if ( xSemaphoreTake( xRegSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
    tps = regTable[21];
    xSemaphoreGive( xRegSemaphore );
  }
  byte nSrcAddr = j1939.GetSourceAddress();
  byte pData[6] = {
    0xFF, // Accelerator switches
    tps, // Accelerator position (0.4% per bit = 0-255)
    0xFF, // Engine load
    0xFF, // Remote accelerator cable position
    0xFF, // Accelerator pedal position 2
    0xFF  // Acceleration rate limit
  };
  // EEC2 (0xF003)
  if ( xSemaphoreTake( xJ1939Semaphore, ( TickType_t ) 3 ) == pdTRUE ) {
    j1939.Transmit(6, 0xF003, nSrcAddr, 0x7F, pData, 6);
    xSemaphoreGive(xJ1939Semaphore);
  }
}

