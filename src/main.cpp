#include <Arduino.h>

#ifdef ESP8266
#include <PolledTimeout.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#elif ESP32
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NTPClient.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
#endif

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESPTelnet.h>
#include <Wire.h>

#include <time.h>
// #include <I2C_eeprom.h>

#include <packmonlib.h>                                      // include my personal blend of herbs and spices

//#include  <SPI.h>
//#include "TimeLib.h"
#include <sntp.h>
// #include "RTClib.h"

#include "pm_struct.h"


#define       SERIAL_SPEED         115200

#define       SDA_PIN              4  // D2
#define       SCL_PIN              5  // D1
#define       BUS_RDY              12 // D6
#define       CLI_ENABLE           14 // D5

#define       STASSID              "Tell my WiFi I love her"
#define       STAPSK               "2317239216"

const char*   ssid               = STASSID;
const char*   password           = STAPSK;

const char*   ntpServerName      = "pool.ntp.org";               // NTP server name
const long    gmtOffset_sec      = -14400;                       // Replace with your GMT offset (seconds)
const int     daylightOffset_sec = 0;                            // Replace with your daylight offset (seconds)


//const uint8_t I2C_MASTER         = 0x42;


String        newHostname        = "packmaster";
IPAddress     ntpServerIP;                                       // time.nist.gov NTP server address

// RTC_DS3231    rtc;
ESPTelnet     telnet;
IPAddress     ip;
//I2C_eeprom    fram(0x50, I2C_DEVICESIZE_24LC64);                 // ferro-electric memory baby!
PackMonLib    toolbox;


uint16_t      port               = 23;
uint16_t      loopCnt            = 0;                            // loop counter to update slave time

uint32_t      lasttimeSync       = 0;                            // when did we last send slaves the time?
uint16_t      timesyncInterval   = 600;                          // sync time every 600 seconds, 10 minutes

volatile bool readTimestamps     = false;
volatile bool readUptimes        = false;
volatile bool readVBus           = false;
volatile bool readVPack          = false;
volatile bool readIPack          = false;

const int ClientA                = 0x35;
const int ClientB                = 0x36;
char buff[100];
// char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void framWriteFloat(int dataAddress, float framData) {
  const uint16_t dataLen = 4;
  union floatArray buffer;
  buffer.floatNumber = framData;                               // convert float into byte array 
  fram.writeBlock(dataAddress, buffer.byteArray, dataLen);
}

void framWriteUlong(int dataAddress, uint32_t framData) {
  const uint16_t dataLen = 4;
  union ulongArray buffer;
  buffer.longNumber = framData;                               // convert float into byte array 
  fram.writeBlock(dataAddress, buffer.byteArray, dataLen);
}

float framReadFloat(int dataAddress) {
  const uint16_t dataLen = 4;
  union floatArray buffer;
  fram.readBlock(dataAddress, buffer.byteArray, dataLen);
  return buffer.floatNumber;
}

uint32_t framReadUlong(int dataAddress) {
  const uint16_t dataLen = 4;
  union ulongArray buffer;
  fram.readBlock(dataAddress, buffer.byteArray, dataLen);
  return buffer.longNumber;
}

uint32_t now() {
  // uint32_t rtcTS = rtc.now().unixtime();
  #ifdef ESP32
  uint32_t ntpTS = timeClient.getEpochTime();
  #else
  uint32_t ntpTS = sntp_get_current_timestamp();
  #endif

  // if (ntpTS>rtcTS) return ntpTS;
  // else return rtcTS;
  return ntpTS;
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

float readI2Cfloat(uint8_t clientAddr, uint8_t cmdAddr);

int scanI2C() {
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
 
  return nDevices;
}

void syncNTP() {
  // uint32_t rtcTS = rtc.now().unixtime();
  #ifdef ESP32
  uint32_t ntpTS = timeClient.getEpochTime();
  #else
  uint32_t ntpTS = sntp_get_current_timestamp();
  #endif
  
  // sprintf(buff, "RTC time is %u\nNTP time is %u", rtcTS, ntpTS);
  sprintf(buff, "NTP time is %u\n", ntpTS);
  telnet.println(buff);

  // if ((ntpTS / 10)!=(rtcTS / 10)) {
  //   rtc.adjust(DateTime(ntpTS)); // set rtc time using ntp timestamp?
  //   telnet.println("Copied NTP time to RTC, retesting.");

  //   rtcTS = rtc.now().unixtime();
  //   #ifdef ESP32
  //   uint32_t ntpTS = timeClient.getEpochTime();
  //   #else
  //   uint32_t ntpTS = sntp_get_current_timestamp();
  //   #endif

  //   sprintf(buff, "RTC time is %u\nNTP time is %u", rtcTS, ntpTS);
  //   telnet.println(buff);
  // } else {
  //   telnet.println("Clocks match, no update needed.");
  // }
}

void syncSlaveTime(uint8_t slaveAddress) {
  // sprintf(buff, "Sending time to slave 0x%X", slaveAddress);
  // telnet.println(buff);
  // toolbox.i2cWriteUlong(slaveAddress, 0x60, now());
  // i2cWriteUL(slaveAddress, 0x60, now());
  // union ulongArray buffer;
  // buffer.longNumber = now();
  toolbox.i2cWriteUlong(slaveAddress, 0x60, now());
  sprintf(buff, "Wrote timestamp to slave 0x%X\n", slaveAddress);
  telnet.println(buff);
  // Wire.beginTransmission(slaveAddress);               // begin transaction with slave address
  // Wire.write(0x60);                                   // send register address byte
  // for (int x = 0; x<4; x++)
  // {
  //   sprintf(buff, "byte %u = 0x%X\n", x, buffer.byteArray[x]);
  //   Wire.write(buffer.byteArray[x]);
  //   telnet.print(buff);
  // }
  // Wire.endTransmission(true);  
  // telnet.println(" ");

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
    } else if (str == "time") {
      union ulongArray buffer;
      buffer.longNumber = now();
      telnet.println("Timestamp bytes:");
      for (int x = 0; x<4; x++)
      {
        sprintf(buff, "byte %u = 0x%X\n", x, buffer.byteArray[x]);
        telnet.print(buff);
      }
      telnet.println(" ");
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
      if (digitalRead(BUS_RDY)==HIGH && digitalRead(CLI_ENABLE)!=HIGH) telnet.println("Warning: Client bus ready but not connected!");
      else if (digitalRead(BUS_RDY)!=HIGH) telnet.println("Warning: Hot swap buffer reports client bus error!");
      
    } else if (str == "sync") {
      syncNTP();
    } else if (str == "set") {
      syncSlaveTime(ClientA);
      syncSlaveTime(ClientB);
      lasttimeSync = now();                           // update last sync timestamp
    } else if (str == "cli+") {
      telnet.println("Enabling hotswap buffer.");
      digitalWrite(CLI_ENABLE, HIGH);                 // enable drivers on hotswap buffer
      delay(1);

      if (digitalRead(BUS_RDY)!=HIGH) telnet.println("Hotswap buffer reports client bus not ready, disconnected from client bus.");
      else telnet.println("Hotswap buffer has been enabled, client bus available.");
    } else if (str == "cli-") {
      digitalWrite(CLI_ENABLE, LOW);
      delay(1);

      if (digitalRead(BUS_RDY)==HIGH) telnet.println("Hot swap buffer disabled, client bus remains available.");
      else telnet.println("Hotswap buffer has been disabled, client bus unavailable.");
      
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
  pinMode(BUS_RDY, INPUT);
  pinMode(CLI_ENABLE, OUTPUT);
  
  digitalWrite(CLI_ENABLE, LOW);  // client interface shutdown 

  Serial.begin(115200);  // start serial for output

  Wire.begin(SDA_PIN, SCL_PIN);        // join i2c bus (address optional for master)
  Wire.setClock(100000);  // 100khz i2c clock

  fram.begin();
//   if (! rtc.begin()) {
//     Serial.println("Couldn't find RTC");
//     Serial.flush();
// //    while (1) delay(10);
//   }  

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
    pinMode(CLI_ENABLE, INPUT); // disable client bus interface before OTA
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

#ifndef ESP8266
  const bool nextPing = true;
#endif

uint32_t readSlaveTs(uint8_t slaveAddr);

float raw2amps(uint32_t rawVal);
float raw2volts(uint32_t rawVal, float scale);
float raw2temp(uint32_t rawVal);

void loop() {
  #ifdef ESP8266 // use esp8266 specific delay, esp32 delay at the bottom of loop()
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);
  #endif
  uint32_t timeStamp = now();

  telnet.loop();  // handle telnet events
  ArduinoOTA.handle();  // handle OTA events

  if (timeStamp > lasttimeSync + timesyncInterval) {    // check to see if we need to refresh the time on slaves
    syncSlaveTime(ClientA);
    syncSlaveTime(ClientB);
    lasttimeSync = now();                           // update last sync timestamp
  }

  if (nextPing) {
    if (readTimestamps) {
      uint32_t fnctimeStamp = now();
      sprintf(buff, "Master timestamp: %u sec", fnctimeStamp);
      telnet.println(buff);

      uint32_t theResult = 0;
      
      //theResult = i2cReadUL(ClientA, 0x62);
      theResult = toolbox.i2cReadUlong(ClientA, 0x62);
      sprintf(buff, "Slave ClientA timestamp: %u sec", theResult);
      telnet.println(buff);

      theResult = toolbox.i2cReadUlong(ClientB, 0x62);
      sprintf(buff, "Slave ClientB timestamp: %u sec", theResult);
      telnet.println(buff);
    }

    if (readUptimes) {
      uint32_t theResult = 0;
      
      theResult = toolbox.i2cReadUlong(ClientA, 0x64);
      sprintf(buff, "Slave ClientA uptime: %u sec", theResult);
      telnet.println(buff);

      theResult = toolbox.i2cReadUlong(ClientB, 0x64);
      sprintf(buff, "Slave ClientB uptime: %u sec", theResult);
      telnet.println(buff);
    }

    if (readVBus) {
      float theResult = 0.0;
      uint32_t rawAdc = 0;

      rawAdc = toolbox.i2cReadUlong(ClientA, 0x3E);
      theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientA bus: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);

      rawAdc = toolbox.i2cReadUlong(ClientB, 0x3E);
      theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientA bus: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);
    }

    if (readVPack) {
      float theResult = 0.0;
      uint32_t rawAdc = 0;

      rawAdc = toolbox.i2cReadUlong(ClientA, 0x39);
      theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientA bus: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);

      rawAdc = toolbox.i2cReadUlong(ClientB, 0x39);
      theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientA bus: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);
    }

    if (readIPack) {
      uint32_t theResult = 0.0;
      
      theResult = toolbox.i2cReadUlong(ClientA, 0x33);
      sprintf(buff, "Slave ClientA load: %.2f amps", theResult);
      telnet.println(buff);

      theResult = toolbox.i2cReadUlong(ClientB, 0x33);
      sprintf(buff, "Slave ClientB load: %.2f amps", theResult);
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

  #ifdef ESP32
  for (int xcnt = 0; xcnt < 1000; xcnt++) {
    ArduinoOTA.handle();
    telnet.loop();  // handle telnet events    
    delay(1);
  }
  #endif
} // end of loop()

uint32_t readSlaveTs(uint8_t slaveAddr) {
  union ulongArray buffer;
  const char stopChar = '\0';
  const uint8_t readBytes = 4;
  Wire.beginTransmission(slaveAddr);                            // start transaction
  Wire.write(0x62);                                             // tell slave we want to read this register
  Wire.endTransmission(false);                                  // send instruction, retain control of bus
  Wire.requestFrom(slaveAddr, readBytes, (bool) true);          // request 6 bytes from slave device and then release bus
  Wire.readBytesUntil(stopChar, buffer.byteArray , readBytes);  // read five bytes or until the first null

  return buffer.longNumber;
}

float readI2Cfloat(uint8_t clientAddr, uint8_t cmdAddr) {
  union ulongArray buffer;
  const uint8_t readBytes = 4;
  uint8_t ptr = 0;

  Wire.beginTransmission(clientAddr);                          // start transaction
  Wire.write(cmdAddr);                                        // tell slave we want to read this register
  Wire.endTransmission(false);                                   // send instruction, retain control of bus
  Wire.requestFrom(clientAddr, readBytes, (bool) true);   
  while (Wire.available() && ptr < readBytes) {
    buffer.byteArray[ptr] = Wire.read();
  }     

  return buffer.longNumber;
}
float raw2amps(uint32_t rawVal)
{
    float mvPa    = 0.136;  // 0.136v or 136mV per amp
    float Amps    = 0.0;
    float Volts   = 0.0;
    float sysVcc  = 5.09;

    Volts = (float)(rawVal * (sysVcc / 1024.0)) - (sysVcc / 2);
    Amps =  (float)Volts / mvPa;

    return Amps;
}

float raw2volts(uint32_t rawVal, float scale)
{
    float Volts   = 0.0;
    float sysVcc  = 5.09;
  
    Volts = (float)(rawVal * (sysVcc / 1024.0)) / scale;
    //Amps =  (float)Volts / acsmvA;
    //adcDataBuffer[0].Amps  = Amps;
    return Volts;
}

float raw2temp(uint32_t rawVal)
{
    float SeriesR    = 47000.0;
    float Resistance = 0.0;
    // convert to resistance
    Resistance = (1024 / rawVal) - 1;
    Resistance = SeriesR / Resistance;

    return Resistance;
}
