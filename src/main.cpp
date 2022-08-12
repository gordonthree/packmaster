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
#include "pm_defs.h"

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
PackMonLib    toolbox;


uint16_t      port               = 23;
uint16_t      loopCnt            = 0;                            // loop counter to update slave time

uint32_t      lasttimeSync       = 0;                            // when did we last send slaves the time?
uint16_t      timesyncInterval   = 600;                          // sync time every 600 seconds, 10 minutes

volatile bool readTimestamps     = false;
volatile bool readUptimes        = false;
volatile bool readVBus           = false;
volatile bool readTemps          = false;
volatile bool readVPack          = false;
volatile bool readIPack          = false;
volatile bool readStatus         = false;
volatile bool writeConfig        = false;
volatile bool readConfig         = false;

const int ClientA                = 0x35;
const int ClientB                = 0x36;
char buff[100];
// char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

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
    } else if (str == "temps") {
      readTemps = readTemps ^ 1;
    } else if (str == "dump") {
      toolbox.i2cWriteUlong(ClientA, 0x77,0);
      toolbox.i2cWriteUlong(ClientB, 0x77,0);
    } else if (str == "up") {
      readUptimes = readUptimes ^ 1;
    } else if (str == "vbus") {
      readVBus = readVBus ^ 1;
    } else if (str == "configw") {
      writeConfig = true;
    } else if (str == "configr") {
      readConfig = true;
    } else if (str == "status") {
      readStatus = readStatus ^ 1;
    } else if (str == "vpack") {
      readVPack = readVPack ^ 1;
    } else if (str == "ipack") {
      readIPack = readIPack ^ 1;
    } else if (str == "scan") {
      if ((digitalRead(BUS_RDY)!=HIGH) && (digitalRead(CLI_ENABLE)==HIGH)) telnet.println("Error condition detected on client bus.");
      else if (digitalRead(CLI_ENABLE)!=HIGH) telnet.println("Client bus not enabled!");
      scanI2C();
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


float raw2amps(uint32_t rawVal);
float raw2volts(uint32_t rawVal, float scale);
double raw2temp(uint32_t rawVal);

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
    if (writeConfig) {
      writeConfig = false;
      /*
        #define PM_REGISTER_HIGHCURRENTLIMIT    0x21 // read / write high current cut off register
        #define PM_REGISTER_HIGHTEMPLIMIT       0x22 // read / write high temperature cut off register
        #define PM_REGISTER_LOWTEMPLIMIT        0x23 // read / write low temperature cut off register
        #define PM_REGISTER_HIGHVOLTLIMIT       0x24 // read / write high voltage cut off register
        #define PM_REGISTER_LOWVOLTLIMIT        0x25 // read / writte low voltage cut off register
      */
      uint8_t CONFIG0 = 0 | (
                              1<<PM_CONFIG0_ENAOVRCURPROT |  // setup config byte
                              1<<PM_CONFIG0_ENAOVRTMPPROT |
                              1<<PM_CONFIG0_ENAUNDTMPPROT |
                              1<<PM_CONFIG0_EMAUNDVLTPROT 
                            );

      double ampLimit  = 15.0;
      double highTemp  = 65.0;
      double lowTemp   = 0.0;
      double highVolt  = 15.20;
      double lowVolt   = 9.90;
      double vbusDiv   = 1.0;
      double vpackDiv  = 0.3333;
      double mvA       = 100;
      double therm     = 1.0;

      telnet.print("Config ClientA... ");
      
      toolbox.i2cWriteUlong(ClientA, PM_REGISTER_CONFIG0BYTE, CONFIG0);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_HIGHCURRENTLIMIT, ampLimit);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_HIGHTEMPLIMIT, highTemp);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_LOWTEMPLIMIT, lowTemp);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_HIGHVOLTLIMIT, highVolt);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_LOWVOLTLIMIT, lowVolt);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_CURRENTMVA, mvA);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_VPACKDIVISOR, vpackDiv);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_VBUSDIVISOR, vbusDiv);
      toolbox.i2cWriteFloat(ClientA, PM_REGISTER_THERMDIVISOR, therm);
      
      
      telnet.print("done!\nConfig ClientB... ");

      toolbox.i2cWriteUlong(ClientB, PM_REGISTER_CONFIG0BYTE, CONFIG0);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_HIGHCURRENTLIMIT, ampLimit);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_HIGHTEMPLIMIT, highTemp);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_LOWTEMPLIMIT, lowTemp);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_HIGHVOLTLIMIT, highVolt);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_LOWVOLTLIMIT, lowVolt);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_VBUSDIVISOR, vbusDiv);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_VPACKDIVISOR, vpackDiv);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_CURRENTMVA, mvA);
      toolbox.i2cWriteFloat(ClientB, PM_REGISTER_THERMDIVISOR, therm);

      telnet.println("done!");

    }
    if (readConfig) {
      readConfig = false;
      uint8_t CONFIG0a = toolbox.i2cReadUlong(ClientA, PM_REGISTER_CONFIG0BYTE);
      uint8_t CONFIG0b = toolbox.i2cReadUlong(ClientB, PM_REGISTER_CONFIG0BYTE);

      sprintf(buff, "ClientA CONFIG0: 0x%X\nClientB CONFIG0: 0x%X\n", CONFIG0a, CONFIG0b);
      telnet.print(buff);
    }

    if (readStatus) {
      uint8_t STATUS0;
      uint8_t STATUS1;
      
      STATUS0 = toolbox.i2cReadUlong(ClientA, PM_REGISTER_STATUS0BYTE);
      STATUS1 = toolbox.i2cReadUlong(ClientA, PM_REGISTER_STATUS1BYTE);
      telnet.print("ClientA STATUS0: 0x");
      telnet.print(STATUS0, HEX);
      telnet.print(" STATUS1: 0x");
      telnet.println(STATUS1, HEX);

      STATUS0 = toolbox.i2cReadUlong(ClientB, PM_REGISTER_STATUS0BYTE);
      STATUS1 = toolbox.i2cReadUlong(ClientA, PM_REGISTER_STATUS1BYTE);
      telnet.print("ClientB STATUS0: 0x");
      telnet.print(STATUS0, HEX);
      telnet.print(" STATUS1: 0x");
      telnet.println(STATUS1, HEX);
    }

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

      theResult = toolbox.i2cReadFloat(ClientA, PM_REGISTER_READBUSVOLTS);
      // theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientA bus: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);

      theResult = toolbox.i2cReadFloat(ClientB, PM_REGISTER_READBUSVOLTS);
      // theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientB bus: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);
    }

    if (readTemps) {
      double t0, t1, t2;

      t0 = toolbox.i2cReadFloat(ClientA, PM_REGISTER_READDEGCT0);
      t1 = toolbox.i2cReadFloat(ClientA, PM_REGISTER_READDEGCT1);
      t2 = toolbox.i2cReadFloat(ClientA, PM_REGISTER_READDEGCT2);
      sprintf(buff, "Slave ClientA t0: %.2f°C t1: %.2f°C t2: %.2f°C", t0, t1, t2);
      telnet.println(buff);

      t0 = toolbox.i2cReadFloat(ClientB, PM_REGISTER_READDEGCT0);
      t1 = toolbox.i2cReadFloat(ClientB, PM_REGISTER_READDEGCT1);
      t2 = toolbox.i2cReadFloat(ClientB, PM_REGISTER_READDEGCT2);
      sprintf(buff, "Slave ClientB t0: %.2f°C t1: %.2f°C t2: %.2f°C", t0, t1, t2);
      telnet.println(buff);
    }


    if (readVPack) {
      double theResult = 0.0;
      uint32_t rawAdc = 0;

      theResult = toolbox.i2cReadFloat(ClientA, 0x39);
      // theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientA pack: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);

      theResult = toolbox.i2cReadFloat(ClientB, 0x39);
      // theResult = raw2volts(rawAdc, 1.0);
      sprintf(buff, "Slave ClientB pack: %.2f volts dc (raw %u)", theResult, rawAdc);
      telnet.println(buff);
    }

    if (readIPack) {
      double theResult = 0.0;
      
      theResult = toolbox.i2cReadFloat(ClientA, 0x33);
      sprintf(buff, "Slave ClientA load: %.2f amps", theResult);
      telnet.println(buff);

      theResult = toolbox.i2cReadFloat(ClientB, 0x33);
      sprintf(buff, "Slave ClientB load: %.2f amps", theResult);
      telnet.println(buff);
    }
  }

  #ifdef ESP32
  for (int xcnt = 0; xcnt < 1000; xcnt++) {
    ArduinoOTA.handle();
    telnet.loop();  // handle telnet events    
    delay(1);
  }
  #endif
} // end of loop()

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

/**
* The NTC table has 1024 interpolation points.
* Unit:0.01 °C
*
*/
int NTC_table[1024] = {
  -7413, -6978, -6543, -6223, -5968, -5754, 
  -5570, -5408, -5263, -5131, -5010, -4899, 
  -4795, -4699, -4608, -4522, -4440, -4363, 
  -4289, -4219, -4152, -4087, -4025, -3965, 
  -3907, -3851, -3797, -3744, -3693, -3644, 
  -3596, -3549, -3504, -3459, -3416, -3374, 
  -3332, -3292, -3252, -3214, -3176, -3139, 
  -3102, -3067, -3032, -2997, -2964, -2930, 
  -2898, -2866, -2834, -2803, -2772, -2742, 
  -2713, -2683, -2655, -2626, -2598, -2571, 
  -2543, -2516, -2490, -2464, -2438, -2412, 
  -2387, -2362, -2337, -2313, -2289, -2265, 
  -2241, -2218, -2195, -2172, -2149, -2127, 
  -2105, -2083, -2061, -2039, -2018, -1997, 
  -1976, -1955, -1935, -1914, -1894, -1874, 
  -1854, -1834, -1815, -1796, -1776, -1757, 
  -1738, -1719, -1701, -1682, -1664, -1646, 
  -1628, -1610, -1592, -1574, -1557, -1539, 
  -1522, -1504, -1487, -1470, -1453, -1437, 
  -1420, -1403, -1387, -1371, -1354, -1338, 
  -1322, -1306, -1290, -1274, -1259, -1243, 
  -1228, -1212, -1197, -1181, -1166, -1151, 
  -1136, -1121, -1106, -1091, -1077, -1062, 
  -1048, -1033, -1019, -1004, -990, -976, -962, 
  -947, -933, -919, -906, -892, -878, -864, 
  -851, -837, -823, -810, -797, -783, -770, 
  -757, -743, -730, -717, -704, -691, -678, 
  -665, -652, -640, -627, -614, -602, -589, 
  -576, -564, -551, -539, -527, -514, -502, 
  -490, -478, -465, -453, -441, -429, -417, 
  -405, -393, -381, -370, -358, -346, -334, 
  -323, -311, -299, -288, -276, -265, -253, 
  -242, -230, -219, -207, -196, -185, -174, 
  -162, -151, -140, -129, -118, -107, -95, 
  -84, -73, -62, -52, -41, -30, -19, -8, 3, 
  14, 24, 35, 46, 56, 67, 78, 88, 99, 109, 
  120, 131, 141, 151, 162, 172, 183, 193, 203, 
  214, 224, 234, 245, 255, 265, 275, 286, 296, 
  306, 316, 326, 336, 346, 356, 366, 376, 386, 
  396, 406, 416, 426, 436, 446, 456, 466, 476, 
  485, 495, 505, 515, 525, 534, 544, 554, 563, 
  573, 583, 592, 602, 612, 621, 631, 641, 650, 
  660, 669, 679, 688, 698, 707, 717, 726, 736, 
  745, 754, 764, 773, 783, 792, 801, 811, 820, 
  829, 839, 848, 857, 867, 876, 885, 895, 904, 
  913, 922, 931, 941, 950, 959, 968, 977, 987, 
  996, 1005, 1014, 1023, 1032, 1041, 1050, 
  1060, 1069, 1078, 1087, 1096, 1105, 1114, 
  1123, 1132, 1141, 1150, 1159, 1168, 1177, 
  1186, 1195, 1204, 1213, 1222, 1231, 1240, 
  1249, 1258, 1266, 1275, 1284, 1293, 1302, 
  1311, 1320, 1329, 1338, 1346, 1355, 1364, 
  1373, 1382, 1391, 1400, 1408, 1417, 1426, 
  1435, 1444, 1452, 1461, 1470, 1479, 1488, 
  1496, 1505, 1514, 1523, 1532, 1540, 1549, 
  1558, 1567, 1575, 1584, 1593, 1602, 1610, 
  1619, 1628, 1637, 1645, 1654, 1663, 1671, 
  1680, 1689, 1698, 1706, 1715, 1724, 1732, 
  1741, 1750, 1758, 1767, 1776, 1785, 1793, 
  1802, 1811, 1819, 1828, 1837, 1845, 1854, 
  1863, 1871, 1880, 1889, 1897, 1906, 1915, 
  1924, 1932, 1941, 1950, 1958, 1967, 1976, 
  1984, 1993, 2002, 2010, 2019, 2028, 2036, 
  2045, 2054, 2062, 2071, 2080, 2088, 2097, 
  2106, 2114, 2123, 2132, 2140, 2149, 2158, 
  2167, 2175, 2184, 2193, 2201, 2210, 2219, 
  2227, 2236, 2245, 2254, 2262, 2271, 2280, 
  2288, 2297, 2306, 2315, 2323, 2332, 2341, 
  2350, 2358, 2367, 2376, 2384, 2393, 2402, 
  2411, 2420, 2428, 2437, 2446, 2455, 2463, 
  2472, 2481, 2490, 2499, 2507, 2516, 2525, 
  2534, 2543, 2551, 2560, 2569, 2578, 2587, 
  2596, 2604, 2613, 2622, 2631, 2640, 2649, 
  2658, 2667, 2675, 2684, 2693, 2702, 2711, 
  2720, 2729, 2738, 2747, 2756, 2765, 2774, 
  2783, 2792, 2801, 2810, 2819, 2828, 2837, 
  2846, 2855, 2864, 2873, 2882, 2891, 2900, 
  2909, 2918, 2927, 2936, 2945, 2954, 2963, 
  2972, 2982, 2991, 3000, 3009, 3018, 3027, 
  3036, 3046, 3055, 3064, 3073, 3082, 3092, 
  3101, 3110, 3119, 3129, 3138, 3147, 3157, 
  3166, 3175, 3184, 3194, 3203, 3213, 3222, 
  3231, 3241, 3250, 3259, 3269, 3278, 3288, 
  3297, 3307, 3316, 3326, 3335, 3345, 3354, 
  3364, 3373, 3383, 3392, 3402, 3411, 3421, 
  3431, 3440, 3450, 3460, 3469, 3479, 3489, 
  3498, 3508, 3518, 3528, 3537, 3547, 3557, 
  3567, 3576, 3586, 3596, 3606, 3616, 3626, 
  3636, 3646, 3655, 3665, 3675, 3685, 3695, 
  3705, 3715, 3725, 3735, 3745, 3756, 3766, 
  3776, 3786, 3796, 3806, 3816, 3827, 3837, 
  3847, 3857, 3868, 3878, 3888, 3899, 3909, 
  3919, 3930, 3940, 3950, 3961, 3971, 3982, 
  3992, 4003, 4013, 4024, 4034, 4045, 4056, 
  4066, 4077, 4088, 4098, 4109, 4120, 4130, 
  4141, 4152, 4163, 4174, 4185, 4195, 4206, 
  4217, 4228, 4239, 4250, 4261, 4272, 4283, 
  4294, 4306, 4317, 4328, 4339, 4350, 4362, 
  4373, 4384, 4395, 4407, 4418, 4429, 4441, 
  4452, 4464, 4475, 4487, 4498, 4510, 4522, 
  4533, 4545, 4557, 4568, 4580, 4592, 4604, 
  4615, 4627, 4639, 4651, 4663, 4675, 4687, 
  4699, 4711, 4723, 4736, 4748, 4760, 4772, 
  4785, 4797, 4809, 4822, 4834, 4846, 4859, 
  4871, 4884, 4897, 4909, 4922, 4935, 4947, 
  4960, 4973, 4986, 4999, 5012, 5025, 5038, 
  5051, 5064, 5077, 5090, 5103, 5116, 5130, 
  5143, 5156, 5170, 5183, 5197, 5210, 5224, 
  5238, 5251, 5265, 5279, 5293, 5306, 5320, 
  5334, 5348, 5362, 5376, 5391, 5405, 5419, 
  5433, 5448, 5462, 5477, 5491, 5506, 5520, 
  5535, 5550, 5564, 5579, 5594, 5609, 5624, 
  5639, 5654, 5670, 5685, 5700, 5715, 5731, 
  5746, 5762, 5778, 5793, 5809, 5825, 5841, 
  5857, 5873, 5889, 5905, 5921, 5937, 5954, 
  5970, 5986, 6003, 6020, 6036, 6053, 6070, 
  6087, 6104, 6121, 6138, 6156, 6173, 6190, 
  6208, 6225, 6243, 6261, 6279, 6297, 6315, 
  6333, 6351, 6369, 6388, 6406, 6425, 6443, 
  6462, 6481, 6500, 6519, 6538, 6557, 6577, 
  6596, 6616, 6636, 6655, 6675, 6695, 6716, 
  6736, 6756, 6777, 6797, 6818, 6839, 6860, 
  6881, 6902, 6924, 6945, 6967, 6989, 7011, 
  7033, 7055, 7077, 7100, 7122, 7145, 7168, 
  7191, 7214, 7238, 7261, 7285, 7309, 7333, 
  7357, 7381, 7406, 7431, 7456, 7481, 7506, 
  7532, 7557, 7583, 7609, 7636, 7662, 7689, 
  7716, 7743, 7770, 7798, 7826, 7854, 7882, 
  7910, 7939, 7968, 7997, 8027, 8057, 8087, 
  8117, 8148, 8178, 8210, 8241, 8273, 8305, 
  8337, 8370, 8403, 8436, 8470, 8504, 8538, 
  8573, 8608, 8643, 8679, 8715, 8752, 8789, 
  8827, 8864, 8903, 8941, 8981, 9020, 9061, 
  9101, 9142, 9184, 9226, 9269, 9312, 9356, 
  9401, 9446, 9491, 9538, 9584, 9632, 9680, 
  9729, 9779, 9830, 9881, 9933, 9986, 10040, 
  10094, 10150, 10206, 10264, 10322, 10381, 
  10442, 10504, 10566, 10630, 10695, 10762, 
  10830, 10899, 10969, 11041, 11115, 11190, 
  11267, 11346, 11426, 11509, 11593, 11680, 
  11768, 11859, 11953, 12049, 12148, 12249, 
  12354, 12462, 12573, 12688, 12806, 12929, 
  13056, 13187, 13324, 13466, 13613, 13767, 
  13927, 14095, 14270, 14454, 14647, 14851, 
  15066, 15293, 15535, 15793, 16068, 16363, 
  16682, 17028, 17405, 17819, 18277, 18790, 
  19369, 20034, 20810, 21737, 22880, 24351, 
  26379, 29519, 35824, 42129
}; 
 
 
/**
* \brief    Converts the ADC result into a temperature value.
*
*           The temperature values are read from the table.
*
* \param    adc_value  The converted ADC result
* \return              The temperature in 0.1 °C
*
*/
double raw2temp(uint32_t adc_value){
 
  /* Read values directly from the table. */
  return (double) NTC_table[ adc_value ] / 100.0;
};

