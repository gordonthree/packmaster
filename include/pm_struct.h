#pragma once
#include <Arduino.h>
#define pm_struct_h

const uint8_t txBufferSize = 50;
const uint8_t rxBufferSize = 50;
const uint8_t adcBufferSize = 4;        // adc1 current, adc2 pack voltage, adc3 bus voltage, adc4 coulumb counter

struct I2C_RX_DATA {
  uint8_t cmdAddr               = 0;    // single byte command register
  char cmdData[rxBufferSize] = {};   // room for N bytes of data
  size_t  dataLen               = 0;    // number of bytes in buffer
};

struct I2C_TX_DATA {
  uint8_t cmdData[txBufferSize] = {};   // room for N bytes of data
  size_t  dataLen = 50;
};

typedef struct  
{
  uint32_t ts;                                             // Timestamp for when this record was created (4 bytes)
  uint8_t  array[4];                                       // Data for this address
  uint32_t raw;                                            // raw data storage (adc raw)
} eedata_t;

static const int eedata_size = 24;                       // save constant for size of eedata structure

union EERECORD                                           // union that converts client data structure into byte array
{
  eedata_t data;                                         // userland data
  uint8_t  byteArray[eedata_size];                       // byte array to send over i2c or store in fram
} ;
typedef struct 
{
  uint8_t     clientAddr;                               // i2c address for the client
  uint32_t    lastSeen;                                 // last seen timestamp for the client
} clientdata_t;

struct ADC_DATA {
  uint8_t adcPin   = 0;                 // Arduino pin number?
  int32_t adcRaw   = 0;                 // raw value
  int32_t adcMin   = 0;                 // raw value
  int32_t adcMax   = 0;                 // raw value
  double  Amps     = 0.0;               // formatted value
  double  Volts    = 0.0;               // formatted value
};

union ulongArray
{
    uint32_t longNumber=0;
    uint8_t  byteArray[4];
};

union floatArray
{
    float   floatNumber=0.0;
    uint8_t byteArray[4];
};



// volatile I2C_RX_DATA rxData;
// volatile I2C_TX_DATA txData;
// volatile ADC_DATA    adcDataBuffer[adcBufferSize];  // Enough room to store three adc readings
