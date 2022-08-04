#include <Arduino.h>
#include <Wire.h>
#include <PolledTimeout.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "ESPTelnet.h"          
#include <time.h>
//#include  <SPI.h>
//#include "TimeLib.h"
#include "sntp.h"
#include "RTClib.h"
//#include "pm_i2croutines.h" // include my personal blend of herbs and spices

#define       SERIAL_SPEED         115200

#define       SDA_PIN              4 // D2
#define       SCL_PIN              5 // D1

#define       STASSID              "Tell my WiFi I love her"
#define       STAPSK               "2317239216"

const char*   ssid               = STASSID;
const char*   password           = STAPSK;

const char*   ntpServerName      = "pool.ntp.org";   // NTP server name
const long    gmtOffset_sec      = -14400;           //Replace with your GMT offset (seconds)
const int     daylightOffset_sec = 0;           //Replace with your daylight offset (seconds)

const uint8_t I2C_SLAVE_LIST[]   = {0x37, 0x39};
const uint8_t I2C_MASTER         = 0x42;
const uint8_t I2C_SLAVE          = 0x37;

String        newHostname        = "packmaster";
IPAddress     ntpServerIP;                        // time.nist.gov NTP server address

RTC_DS3231    rtc;
ESPTelnet     telnet;
IPAddress     ip;

uint16_t      port               = 23;
uint16_t      loopCnt            = 0;                        // loop counter to update slave time

uint32_t      lasttimeSync       = 0;                      // when did we last send slaves the time?
uint16_t      timesyncInterval   = 600;              // sync time every 600 seconds, 10 minutes

volatile bool readTimestamps     = false;
volatile bool readUptimes        = false;
volatile bool readVBus           = false;
volatile bool readVPack          = false;
volatile bool readIPack          = false;

char buff[100];
// char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

uint32_t now() {
  uint32_t rtcTS = rtc.now().unixtime();
  uint32_t ntpTS = sntp_get_current_timestamp();

  if (ntpTS>rtcTS) return ntpTS;
  else return rtcTS;
}

void printLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  Serial.println(asctime(timeinfo));
  delay(1000);
}

// union Uint32Buff {      // Union to break down single long into four bytes.
//   uint32_t longNumber;
//   char longBytes[4];
// };

void i2cWriteUL(uint8_t slaveAddress, uint8_t cmdAddress, uint32_t cmdData) {
  // Uint32Buff txbuffer;
  char txbuffer[15];
  sprintf(txbuffer, "%lu", cmdData);
  Wire.beginTransmission(slaveAddress);               // begin transaction with slave address
  Wire.write(0x60);                                   // send register address byte
  Wire.write(txbuffer);                                // send bytes
  Wire.endTransmission(true);                         // end transaction with a stop
  // sprintf(buff, "Wrote %s to slave 0x%X at address 0x%X", txbuffer, slaveAddress, cmdAddress);
  // telnet.println(buff);
}

float i2cReadF(uint8_t slaveAddress, uint8_t cmdAddress) {
  // uint8_t byteCnt = 0;
  uint8_t readBytes = 5;
  char rxBuffer[20];
  float theResult = 0.0;
  char stopChar = '\0';                             // unix null char
  Wire.beginTransmission(slaveAddress);             // start transaction
  Wire.write(cmdAddress);                                 // tell slave we want to read this register
  Wire.endTransmission(false);                      // send instruction, retain control of bus
  Wire.requestFrom(slaveAddress, readBytes, (bool) true);     // request 6 bytes from slave device and then release bus
  Wire.readBytesUntil(stopChar, rxBuffer, readBytes);   // read five bytes or until the first null

  theResult = strtof(rxBuffer, NULL);

  return theResult;
}

uint32_t i2cReadUL(uint8_t slaveAddress, uint8_t cmdAddress) {
  char      rxBuffer[20];
  char      stopChar  = '\0';                             // unix null char
  uint8_t   readBytes = 11;
  uint32_t  theResult = 0.0;
  Wire.beginTransmission(slaveAddress);             // start transaction
  Wire.write(cmdAddress);                                 // tell slave we want to read this register
  Wire.endTransmission(false);                      // send instruction, retain control of bus
  Wire.requestFrom(slaveAddress, readBytes, (bool) true);     // request 6 bytes from slave device and then release bus
  Wire.readBytesUntil(stopChar, rxBuffer, readBytes);   // read five bytes or until the first null
  // telnet.println(rxBuffer);
  theResult = strtoul(rxBuffer, NULL, 10);
  
  return theResult;
}

long i2cReadI(int slaveAddress, int cmdAddress) {
    // uint8_t byteCnt = 0;
  uint8_t readBytes = 5;
  char rxBuffer[20];
  long theResult = 0.0;
  char stopChar = '\0';                                       // unix null char
  Wire.beginTransmission(slaveAddress);                       // start transaction
  Wire.write(cmdAddress);                                     // tell slave we want to read this register
  Wire.endTransmission(false);                                // send instruction, retain control of bus
  Wire.requestFrom(slaveAddress, readBytes, (bool) true);     // request bytes from slave device and then release bus
  Wire.readBytesUntil(stopChar, rxBuffer, readBytes);         // read bytes or until the first null

  theResult = strtol(rxBuffer, NULL, 10);

  return theResult;
}

void telnetLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  telnet.println(asctime(timeinfo));
}

void onTelnetConnect(String ip) {
  Serial.print("Telnet connection from ");
  Serial.print(ip);
  Serial.println(" connected!");

  telnet.println("\n\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");

  telnetLocalTime();
}

void onTelnetDisconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
}

void onTelnetReconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connected");
}

void errorMsg(String error, bool restart = true) {
  Serial.println(error);
  if (restart) {
    Serial.println("Rebooting now...");
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

void scanI2C() {
  byte error, address;
  int nDevices;

  telnet.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      telnet.print("I2C device found at address 0x");
      if (address<16) 
      telnet.print("0");
      telnet.print(address,HEX);
      telnet.println(" !");

      nDevices++;
    }
     else if (error==4) 
    {
      telnet.print("Unknow error at address 0x");
      if (address<16) 
      telnet.print("0");
      telnet.println(address,HEX);
    } 
  }
  
  if (nDevices == 0)
    telnet.println("No I2C devices found\n");
  else
    telnet.println("done\n");
 
}

void syncNTP() {
  uint32_t rtcTS = rtc.now().unixtime();
  uint32_t ntpTS = sntp_get_current_timestamp();

  sprintf(buff, "RTC time is %u\nNTP time is %u", rtcTS, ntpTS);
  telnet.println(buff);

  if (ntpTS>rtcTS) {
    rtc.adjust(DateTime(ntpTS)); // set rtc time using ntp timestamp?
    telnet.println("Copied NTP time to RTC, retesting.");

    rtcTS = rtc.now().unixtime();
    ntpTS = sntp_get_current_timestamp();

    sprintf(buff, "RTC time is %u\nNTP time is %u", rtcTS, ntpTS);
    telnet.println(buff);
  } else {
    telnet.println("Clocks match, no update needed.");
  }
}

void syncSlaveTime(uint8_t slaveAddress) {
  sprintf(buff, "Sending time to slave 0x%X", slaveAddress);
  telnet.println(buff);
  i2cWriteUL(slaveAddress, 0x60, now());
}

void setupTelnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  
  // passing a lambda function
  telnet.onInputReceived([](String str) {
    // checks for a certain command
    if (str == "ping") {
      telnet.println("> pong");
      Serial.println("- Telnet: pong");
    // disconnect the client
    } else if (str == "ts") {
      readTimestamps = readTimestamps ^ 1;
    } else if (str == "up") {
      readUptimes = readUptimes ^ 1;
    } else if (str == "vbus") {
      readVBus = readVBus ^ 1;
    } else if (str == "vpack") {
      readVPack = readVPack ^ 1;
    } else if (str == "ipack") {
      readIPack = readIPack ^ 1;
    } else if (str == "scan") {
      scanI2C();
    } else if (str == "sync") {
      syncNTP();
    } else if (str == "set") {
      syncSlaveTime(0x37);
      syncSlaveTime(0x39);
      lasttimeSync = now();                           // update last sync timestamp
    } else if (str == "bye") {
      telnet.println("> good bye...");
      telnet.disconnectClient();
      }
  });

  Serial.print("- Telnet: ");
  if (telnet.begin(port)) {
    Serial.println("running");
  } else {
    Serial.println("error.");
    errorMsg("Will reboot...");
  }
}

void syncProvider() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServerName);
}

void setup() {
  Serial.begin(115200);  // start serial for output
  Wire.begin(SDA_PIN, SCL_PIN);        // join i2c bus (address optional for master)
  Wire.setClock(100000);  // 100khz i2c clock

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
//    while (1) delay(10);
  }  

  delay(2000);
  
  Serial.println("\n\nBooting");
  
  WiFi.mode(WIFI_STA);
  WiFi.hostname(newHostname.c_str());
  
  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }


  setupTelnet();

  if(!WiFi.hostByName(ntpServerName, ntpServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed.");
    //Serial.flush();
    //ESP.reset();
  } else {
    Serial.print("Time server IP: ");
    Serial.println(ntpServerIP);
    
  //init and get the time
    Serial.println("Sending NTP request ...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServerName);
    //setSyncInterval(600);
    //printLocalTime();
  }

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    telnet.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    telnet.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  telnet.println("Ready");
  telnet.print("IP address: ");
  telnet.println(WiFi.localIP());
  
}


void loop() {
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);
  uint32_t timeStamp = now();

  telnet.loop();  // handle telnet events
  ArduinoOTA.handle();  // handle OTA events

  if (timeStamp > lasttimeSync + timesyncInterval) {    // check to see if we need to refresh the time on slaves
    syncSlaveTime(0x37);
    syncSlaveTime(0x39);
    lasttimeSync = now();                           // update last sync timestamp
  }

  if (nextPing) {
    // telnet.println("Tick");

    if (readTimestamps) {
      uint32_t fnctimeStamp = now();
      sprintf(buff, "Master timestamp: %u sec", fnctimeStamp);
      telnet.println(buff);

      uint32_t theResult = 0;
      
      theResult = i2cReadUL(0x37, 0x62);
      sprintf(buff, "Slave 0x37 timestamp: %u sec", theResult);
      telnet.println(buff);

      theResult = i2cReadUL(0x39, 0x62);
      sprintf(buff, "Slave 0x39 timestamp: %u sec", theResult);
      telnet.println(buff);
    }

    if (readUptimes) {
      uint32_t theResult = 0;
      
      theResult = i2cReadUL(0x37, 0x64);
      sprintf(buff, "Slave 0x37 uptime: %u sec", theResult);
      telnet.println(buff);

      theResult = i2cReadUL(0x39, 0x64);
      sprintf(buff, "Slave 0x39 uptime: %u sec", theResult);
      telnet.println(buff);
    }

    if (readVBus) {
     float theResult = 0.0;
      
      theResult = i2cReadF(0x37, 0x3E);
      sprintf(buff, "Slave 0x37 bus: %.2f volts dc", theResult);
      telnet.println(buff);

      theResult = i2cReadF(0x39, 0x3E);
      sprintf(buff, "Slave 0x39 bus: %.2f volts dc", theResult);
      telnet.println(buff);
    }

    if (readVPack) {
     float theResult = 0.0;
      
      theResult = i2cReadF(0x37, 0x39);
      sprintf(buff, "Slave 0x37 pack: %.2f volts dc", theResult);
      telnet.println(buff);

      theResult = i2cReadF(0x39, 0x39);
      sprintf(buff, "Slave 0x39 pack: %.2f volts dc", theResult);
      telnet.println(buff);
    }

    if (readIPack) {
      float theResult = 0.0;
      
      theResult = i2cReadF(0x37, 0x33);
      sprintf(buff, "Slave 0x37 load: %.2f amps", theResult);
      telnet.println(buff);

      theResult = i2cReadF(0x39, 0x33);
      sprintf(buff, "Slave 0x39 load: %.2f amps", theResult);
      telnet.println(buff);
    }

    // now try writing some data
    //telnet.print("TX: ");

    // uint32_t timeStamp = sntp_get_current_timestamp();  // grab most rececnt timestamp
    // ltoa(timeStamp, buff, 10);                          // convert timestamp into C string
    
    // if (tickTock) {
    //   Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    //   Wire.write(0x2E);                                 // register address
    //   Wire.endTransmission(true);                       // end transaction with a stop
    //   tickTock = false;
    // } else {
    //   Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    //   Wire.write(0x2F);                                 // register address
    //   Wire.endTransmission(true);                       // end transaction with a stop   
    //   tickTock = true;
    // }                 

  }

}

