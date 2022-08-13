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


//#include  <SPI.h>
//#include "TimeLib.h"
#include <sntp.h>
// #include "RTClib.h"

#include "packmonlib.h"                                     // include my personal blend of herbs and spices
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

      sprintf(buff, "ClientA STATUS0: 0x%x STATUS1: 0x%x");
      telnet.println(buff);

      if (bitRead(STATUS0, PM_STATUS0_CONFIGSET)) telnet.println("ClientA has valid config");
      else telnet.println("ClientA config is invalid");
      if (bitRead(STATUS0, PM_STATUS0_RANGEISNS)) telnet.println("ClientA current sensor out of range");
      if (bitRead(STATUS0, PM_STATUS0_RANGETSNS)) telnet.println("ClientA temp sensor out of range");
      if (bitRead(STATUS0, PM_STATUS0_RANGEVSNS)) telnet.println("ClientA pack voltage sensor out of range");
      if (bitRead(STATUS0, PM_STATUS0_TIMESET)) telnet.println("ClientA time has been set");
      if (bitRead(STATUS0, PM_STATUS0_WARNCURRENT)) telnet.println("ClientA high current warning");
      if (bitRead(STATUS0, PM_STATUS0_WARNTEMP)) telnet.println("ClientA temperature warning");
      if (bitRead(STATUS0, PM_STATUS0_WARNVOLTAGE)) telnet.println("ClientA pack voltage warning");
      if (bitRead(STATUS1, PM_STATUS1_RANGEVBUS)) telnet.println("ClientA bus voltage out of range");
      

      sprintf(buff, "ClientB STATUS0: 0x%x STATUS1: 0x%x");
      telnet.println(buff);
      if (bitRead(STATUS0, PM_STATUS0_CONFIGSET)) telnet.println("ClientB has valid config");
      else telnet.println("ClientB config is invalid");
      if (bitRead(STATUS0, PM_STATUS0_RANGEISNS)) telnet.println("ClientB current sensor out of range");
      if (bitRead(STATUS0, PM_STATUS0_RANGETSNS)) telnet.println("ClientB temp sensor out of range");
      if (bitRead(STATUS0, PM_STATUS0_RANGEVSNS)) telnet.println("ClientB pack voltage sensor out of range");
      if (bitRead(STATUS0, PM_STATUS0_TIMESET)) telnet.println("ClientB time has been set");
      if (bitRead(STATUS0, PM_STATUS0_WARNCURRENT)) telnet.println("ClientB high current warning");
      if (bitRead(STATUS0, PM_STATUS0_WARNTEMP)) telnet.println("ClientB temperature warning");
      if (bitRead(STATUS0, PM_STATUS0_WARNVOLTAGE)) telnet.println("ClientB pack voltage warning");
      if (bitRead(STATUS1, PM_STATUS1_RANGEVBUS)) telnet.println("ClientB bus voltage out of range");

      readStatus = false;



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
  return (double) NTC_table[ adc_value ] / 1000.0;
};

