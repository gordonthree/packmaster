#include "esphome.h"
//#include "sensor.h"
#include <Wire.h>

class PackMonitor : public Component {
 public:
  //Sensor *voltage_sensor1 = new Sensor();
  //Sensor *voltage_sensor2 = new Sensor();
  Sensor *current_sensor = new Sensor();

  MyCustomSensor() : PollingComponent(15000) { }

  void setup() {
    // Initialize the device here. Usually Wire.begin() will be called in here,
    // though that call is unnecessary if you have an 'i2c:' entry in your config

    // Wire.begin();
  }
  void update() override {
    char   buffer[20];
    float  result    = 0.0;
    int    readBytes = 6;
 
    // Example: write the value 0x42 to register 0x78 of device with address 0x21
    Wire.beginTransmission(0x37);
    Wire.write(0x33);                                 // tell slave we want to read this register
    Wire.endTransmission(false);
    Wire.write(0x42);
    Wire.requestFrom(0x37, readBytes, (bool)true);            // request 6 bytes from slave device and then release bus
    Wire.readBytes(buffer, readBytes);
    Wire.endTransmission();

    result = strtof(buffer, readBytes);

    current_sensor->publish_state(result);
  }
};
