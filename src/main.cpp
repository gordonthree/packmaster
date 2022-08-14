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
#include <sntp.h>

#include "packmonlib.h"                                     // include my personal blend of herbs and spices
#include "pm_struct.h"
#include "pm_defs.h"
#include "pm_fram.h"

#define       SERIAL_SPEED         115200

#define       SDA_PIN              4  // D2
#define       SCL_PIN              5  // D1
#define       BUS_RDY              12 // D6
#define       CLI_ENABLE           14 // D5

#define       STASSID              "Tell my WiFi I love her"
#define       STAPSK               "2317239216"

const char*   ssid               = STASSID;
const char*   password           = STAPSK;

const char*   ntpServerName      = "pool.ntp.org";              // NTP server name
const long    gmtOffset_sec      = -14400;                      // Replace with your GMT offset (seconds)
const int     daylightOffset_sec = 0;                           // Replace with your daylight offset (seconds)


//const uint8_t I2C_MASTER         = 0x42;

static const uint8_t maxClients          = 4;                   // maximum number of clients we can handle
clientdata_t Clients                     [maxClients];          // array to match client number with address and last-seen timestamp
FRAMSTORAGE  fram                        [maxClients];          // array to hold data from X clients
static const uint8_t ee_register_cnt     = 0x7f;                // total number of registers of client data

String        newHostname        = "packmaster";
IPAddress     ntpServerIP;                                      // time.nist.gov NTP server address

// RTC_DS3231    rtc;
ESPTelnet     telnet;
IPAddress     ip;
PackMonLib    toolbox;

uint8_t       clientCount        = 0;

uint16_t      port               = 23;
uint16_t      loopCnt            = 0;                            // loop counter to update slave time
uint16_t      timesyncInterval   = 600;                          // sync time every 600 seconds, 10 minutes

uint32_t      lasttimeSync       = 0;                            // when did we last send slaves the time?

volatile bool readTimestamps     = false;
volatile bool readUptimes        = false;
volatile bool readVolts          = false;
volatile bool readTemps          = false;
volatile bool readLoad           = false;
volatile bool readStatus         = false;
volatile bool writeConfig        = false;
volatile bool readConfig         = false;

char          buff[100];

uint32_t      now();                                                                    // returns unix timestamp
float         raw2amps(uint32_t rawVal);                                                // convert raw adc reading into current
float         raw2volts(uint32_t rawVal, float scale);                                  // cpmvert raw adc readomg into voltage
float         raw2temp(uint32_t rawVal);                                                // convert raw adc reading into temperature using LUT
void          addClient(uint8_t clientID, uint8_t clientAddr);                          // add client address to the Clients list, return false if client could not be added
void          readClientArray(uint8_t clientNumber, uint8_t startReg, uint8_t stopReg); // read range of registers from clientNumber
void          refreshClients();                                                         // loop through the clients, updating our memory buffer
void          errorMsg(String error, bool restart);                                     // error handling routine used by telnet
void          onTelnetConnectionAttempt(String ip);                                     // telnet function
void          onTelnetReconnect(String ip);                                             // telnet function
void          onTelnetDisconnect(String ip);                                            // telnet function
void          onTelnetConnect(String ip);                                               // telnet function
void          printLocalTime();                                                         // print the current time to serial port
void          telnetLocalTime();                                                        // print the current time to telnet port
void          syncNTP();                                                                // check on NTPd
int           scanI2C();                                                                // scan i2c bus
void          syncProvider();                                                           // used by time library to keep clock set
void          setupTelnet();                                                            // setup telnet port
void          syncClientTime();                                                         // send time to the clients
void          configClients();                                                          // send base config to the clients
void          printClientconfigs();                                                     // print base config of the clients to telnet
void          printClientstatus();                                                      // print client status to telnet
void          printClienttimes();                                                       // print client clocks to telnet
void          printClientuptimes();                                                     // print client uptimes to telnet
void          printClienttemps();                                                       // print client temperature readings to telnet
void          printClientvolts();                                                       // print client voltage readings to telnet
void          printClientloads();                                                       // print client amperage readings to telnet
void          dumpFram();                                                               // dump contents of fram array to telnet

void setup() 
{
  pinMode(BUS_RDY, INPUT);
  pinMode(CLI_ENABLE, OUTPUT);
  
  digitalWrite(CLI_ENABLE, LOW);  // client interface shutdown 

  Serial.begin(115200);  // start serial for output

  Wire.begin(SDA_PIN, SCL_PIN);        // join i2c bus (address optional for master)
  Wire.setClock(100000);  // 100khz i2c clock

  digitalWrite(CLI_ENABLE, HIGH);                 // enable drivers on hotswap buffer

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
    Serial.print("Sending NTP request... ");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServerName);
    Serial.println("done.");
    
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

  addClient(0, 0x35);
  addClient(1, 0x36);

} // end setup

#ifndef ESP8266
  const bool nextPing = true;
#endif

void loop() 
{
  #ifdef ESP8266 // use esp8266 specific delay, esp32 delay at the bottom of loop()
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);
  #endif
  uint32_t timeStamp = now();

  telnet.loop();        // handle telnet events
  ArduinoOTA.handle();  // handle OTA events

  if (timeStamp > lasttimeSync + timesyncInterval) {    // check to see if we need to refresh the time on clients
    syncClientTime();
    lasttimeSync = now();                               // update last sync timestamp
  }

  if (nextPing) {
    // refreshClients();                                   // pull updated information from connected clients

    if (writeConfig) {
      configClients();
      writeConfig = false;
    }
    if (readConfig) {
      printClientconfigs();
      readConfig = false;
    }

    if (readStatus) {
      printClientstatus();
      readStatus = false;
    }

    if (readTimestamps) {
      printClienttimes();
      readTimestamps = false;
    }

    if (readUptimes) {
      printClientuptimes();
      readUptimes = false;
    }

    if (readVolts) {
      printClientvolts();
      readVolts = false;
    }

    if (readTemps) {
      printClienttemps();
      readTemps = false;
    }

    if (readLoad) {
      printClientloads();
      readLoad = false;
    }
  } // end of nextping

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
float raw2temp(uint32_t adc_value)
{
  /* Read values directly from the table. */
  return (double) NTC_table[ adc_value ] / 1000.0;
}

void readClientArray(uint8_t clientNumber, uint8_t startReg, uint8_t stopReg)
{ 
  uint8_t byteArray[24];
  uint8_t clientAddr = Clients[clientNumber].clientAddr;                                 // Get client's i2c address

  Wire.beginTransmission(clientAddr);                                                    // start transaction

  for (uint8_t cmdAddress = startReg; cmdAddress < stopReg+1; cmdAddress++)              // step through several registers to get all the data
  {
    Wire.write(cmdAddress);                                                              // tell slave we want to read this register
    Wire.endTransmission(false);                                                         // send instruction, retain control of bus, no stop
    if (cmdAddress==stopReg)                                                             // if we're making our last request, let go of the bus
      Wire.requestFrom(clientAddr, eedata_size, (bool) true);                            // request 6 bytes from slave device and then send stop
    else                                                                                 // keep control of bus until we are finished
      Wire.requestFrom(clientAddr, eedata_size, (bool) false);                           // request 6 bytes from slave device and keep control of bus
    int byteCnt = Wire.readBytes(byteArray, eedata_size);                                              // read i2c bus data into memory
    // telnet.print("RX bytes: ");
    // telnet.println(byteCnt, DEC);
    // Clients[clientNumber] addArrayData(cmdAddress, byteArray);
    fram[clientNumber].addArrayData(cmdAddress, byteArray);                              // add data to class array
  }
}

void refreshClients() // read several records from the client to update our in-ram register 
{
  for (int clientNumber = 0; clientNumber < maxClients; clientNumber++)
  {
    if (Clients[clientNumber].clientAddr>0)
    {
      telnet.print("\nUpdating registers from client ");
      telnet.println(clientNumber, DEC);

      Clients[clientNumber].lastSeen = now();          // record last update time

      readClientArray(clientNumber, 0x21, 0x2e);
      readClientArray(clientNumber, 0x32, 0x3b);
      readClientArray(clientNumber, 0x41, 0x49);
      readClientArray(clientNumber, 0x50, 0x55);
    }
  }
}

void addClient(uint8_t clientID, uint8_t clientAddr)
{
  Clients[clientID].clientAddr = clientAddr;                                  // assign client to requested slot
  clientCount = 0;                                                            // reset client counter
  
  for (int i = 0; i < maxClients; i++ ) 
  {
    if (Clients[i].clientAddr>0) clientCount++; // count total number of clients
  }
}

uint32_t now() 
{
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


void onTelnetConnect(String ip) 
{
  Serial.print("Telnet connection from ");
  Serial.print(ip);
  Serial.println(" connected!");

  telnet.println("\n\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");

  telnetLocalTime();
}

void onTelnetDisconnect(String ip) 
{
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
}

void onTelnetReconnect(String ip) 
{
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) 
{
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connected");
}

void errorMsg(String error, bool restart) 
{
  Serial.println(error);
  if (restart) {
    Serial.println("Rebooting now...");
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

int scanI2C() 
{
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

void syncNTP() 
{
  // uint32_t rtcTS = rtc.now().unixtime();
  #ifdef ESP32
  uint32_t ntpTS = timeClient.getEpochTime();
  #else
  uint32_t ntpTS = sntp_get_current_timestamp();
  #endif
  
  // sprintf(buff, "RTC time is %u\nNTP time is %u", rtcTS, ntpTS);
  sprintf(buff, "NTP time is %u\n", ntpTS);
  telnet.println(buff);
}

void syncClientTime() 
{
  for (int i = 0; i < clientCount; i++)
  {
    uint8_t clientAddr = Clients[i].clientAddr;
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_SETEPOCHTIME, now());
    sprintf(buff, "Wrote timestamp to client 0x%X\n", clientAddr);
    telnet.println(buff);
  }
}

void setupTelnet() 
{  
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
    } else if (str == "register") {
      addClient(0, 0x35);
      telnet.println("Registered client 0x35 as client 0");
      addClient(1, 0x36);
      telnet.println("Registered client 0x36 as client 1");
      telnet.print("Client count: ");
      telnet.println(clientCount, DEC);
    } else if (str == "refresh") {
      telnet.print("Attempting to refresh client information... ");
      refreshClients();
      telnet.println("done!");
    } else if (str == "dump") {
      toolbox.i2cWriteUlong(0x35, 0x77, 0);
      toolbox.i2cWriteUlong(0x36, 0x77, 0);
    } else if (str == "up") {
      readUptimes = readUptimes ^ 1;
    } else if (str == "volts") {
      readVolts = readVolts ^ 1;
    } else if (str == "configw") {
      writeConfig = true;
    } else if (str == "configr") {
      readConfig = true;
    } else if (str == "fram") {
      dumpFram();
    } else if (str == "status") {
      readStatus = readStatus ^ 1;
    } else if (str == "load") {
      readLoad = readLoad ^ 1;
    } else if (str == "scan") {
      if ((digitalRead(BUS_RDY)!=HIGH) && (digitalRead(CLI_ENABLE)==HIGH)) telnet.println("Error condition detected on client bus.");
      else if (digitalRead(CLI_ENABLE)!=HIGH) telnet.println("Client bus not enabled!");
      scanI2C();
    } else if (str == "sync") {
      syncNTP();
    } else if (str == "set") {
      syncClientTime();
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
    errorMsg("Will reboot...", true);
  }
}

void syncProvider() 
{
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServerName);
}

void configClients()
{
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
  uint32_t ts       = now();
  float   ampLimit = 15.0;
  float   highTemp = 65.0;
  float   lowTemp  = 0.0;
  float   highVolt = 15.20;
  float   lowVolt  = 9.90;
  float   vbusDiv  = 1.0;
  float   vpackDiv = 0.3333;
  float   mvA      = 100;
  float   vtmpDiv    = 1.0;

  for (int i = 0; i < clientCount; i++)
  {
    uint8_t clientAddr  = Clients[i].clientAddr;
    Clients[i].lastSeen = ts;
    sprintf(buff, "Updating config for client #%u at address 0x%x\n", i, clientAddr);
    telnet.print(buff);
    
    // first update in ram cache, then write to the clients
    fram[i].addByte  (PM_REGISTER_CONFIG0BYTE,      ts, CONFIG0);
    fram[i].addDouble(PM_REGISTER_HIGHCURRENTLIMIT, ts, ampLimit);
    fram[i].addDouble(PM_REGISTER_HIGHTEMPLIMIT,    ts,highTemp);
    fram[i].addDouble(PM_REGISTER_LOWTEMPLIMIT,     ts, lowTemp);
    fram[i].addDouble(PM_REGISTER_HIGHVOLTLIMIT,    ts, highVolt);
    fram[i].addDouble(PM_REGISTER_LOWVOLTLIMIT,     ts, lowVolt);
    fram[i].addDouble(PM_REGISTER_CURRENTMVA,       ts, mvA);
    fram[i].addDouble(PM_REGISTER_VPACKDIVISOR,     ts, vpackDiv);
    fram[i].addDouble(PM_REGISTER_VBUSDIVISOR,      ts, vbusDiv);
    fram[i].addDouble(PM_REGISTER_THERMDIVISOR,     ts, vtmpDiv);

    toolbox.i2cWriteByte (clientAddr, PM_REGISTER_CONFIG0BYTE, CONFIG0);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_HIGHCURRENTLIMIT, ampLimit);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_HIGHTEMPLIMIT, highTemp);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_LOWTEMPLIMIT, lowTemp);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_HIGHVOLTLIMIT, highVolt);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_LOWVOLTLIMIT, lowVolt);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_CURRENTMVA, mvA);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_VPACKDIVISOR, vpackDiv);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_VBUSDIVISOR, vbusDiv);
    toolbox.i2cWriteUlong(clientAddr, PM_REGISTER_THERMDIVISOR, vtmpDiv);
  }

}

void printClientconfigs()
{
  for (int i = 0; i < clientCount; i++)
  {
    uint8_t  clientAddr = Clients[i].clientAddr;
    uint32_t ts         = fram[i].getTimeStamp (PM_REGISTER_CONFIG0BYTE);
    
    sprintf(buff, "Configuration of client #%u (0x%x) as of %u:\n", i, clientAddr, ts);
    telnet.print(buff);

    uint8_t  CONFIG0  = fram[i].getDataByte  (PM_REGISTER_CONFIG0BYTE);
    float   ampLimit = fram[i].getDataDouble(PM_REGISTER_HIGHCURRENTLIMIT);
    float   highTemp = fram[i].getDataDouble(PM_REGISTER_HIGHTEMPLIMIT);   
    float   lowTemp  = fram[i].getDataDouble(PM_REGISTER_LOWTEMPLIMIT);  
    float   highVolt = fram[i].getDataDouble(PM_REGISTER_HIGHVOLTLIMIT);   
    float   lowVolt  = fram[i].getDataDouble(PM_REGISTER_LOWVOLTLIMIT);    
    float   vbusDiv  = fram[i].getDataDouble(PM_REGISTER_CURRENTMVA);   
    float   vpackDiv = fram[i].getDataDouble(PM_REGISTER_VPACKDIVISOR);    
    float   vtmpDiv  = fram[i].getDataDouble(PM_REGISTER_THERMDIVISOR);
    float   mvA      = fram[i].getDataDouble(PM_REGISTER_VBUSDIVISOR);   
    
    sprintf(buff, "Register contents of CONFIG0: 0x%x", CONFIG0);
    telnet.println(buff);

    sprintf(buff, "High current limit is %.3f amps.", ampLimit);
    telnet.println(buff);

    sprintf(buff, "Temperature limit is %.3f to %.3f", lowTemp, highTemp);
    telnet.println(buff);

    sprintf(buff, "Pack voltage limit is %.3f to %.3f", lowVolt, highVolt);
    telnet.println(buff);

    sprintf(buff, "Divisor: bus voltage: %.3f pack voltage: %.3f temp sensor: %.3f", vbusDiv, vpackDiv, vtmpDiv);
    telnet.println(buff);

    sprintf(buff, "Current sensor millivolts per amp multiplier is %.3f", mvA);
    telnet.println(buff);

  }
}

void printClientstatus()
{
  for (int i=0; i<clientCount; i++)
  {
    uint8_t  clientAddr = Clients[i].clientAddr;
    uint32_t ts         = fram[i].getTimeStamp(PM_REGISTER_STATUS0BYTE);
    uint8_t  STATUS0    = fram[i].getDataByte(PM_REGISTER_STATUS0BYTE);
    uint8_t  STATUS1    = fram[i].getDataByte(PM_REGISTER_STATUS1BYTE);

    sprintf(buff, "Status of client #%u (0x%x) as of %u.\nSTATUS0: 0x%x STATUS1: 0x%x", i, clientAddr, ts, STATUS0, STATUS1);
    telnet.println(buff);

    if (bitRead(STATUS0, PM_STATUS0_CONFIGSET)) telnet.println("Configuration is set");
    else telnet.println("Configuration is not set");
    if (bitRead(STATUS0, PM_STATUS0_TIMESET)) telnet.println("Time has been set");
    if (bitRead(STATUS0, PM_STATUS0_RANGEISNS)) telnet.println("Current sensor out of range");
    if (bitRead(STATUS0, PM_STATUS0_RANGETSNS)) telnet.println("Temp sensor out of range");
    if (bitRead(STATUS0, PM_STATUS0_RANGEVSNS)) telnet.println("Pack voltage out of range");
    if (bitRead(STATUS1, PM_STATUS1_RANGEVBUS)) telnet.println("Bus voltage out of range");
    if (bitRead(STATUS0, PM_STATUS0_WARNCURRENT)) telnet.println("High current warning");
    if (bitRead(STATUS0, PM_STATUS0_WARNTEMP)) telnet.println("Pack temperature warning");
    if (bitRead(STATUS0, PM_STATUS0_WARNVOLTAGE)) telnet.println("Pack voltage warning");
    
    telnet.println("");
  }
}
void printClienttimes()
{
  for (int i=0; i<clientCount; i++)
  {
    uint8_t  clientAddr = Clients[i].clientAddr;
    uint32_t ts         = toolbox.i2cReadUlong(clientAddr, PM_REGISTER_CURRENTTIME);

    sprintf(buff, "Client #%u (0x%x) %u", i, clientAddr, ts);
    telnet.println(buff);
  }
}

void printClientuptimes()
{
  for (int i=0; i<clientCount; i++)
  {
    uint8_t  clientAddr = Clients[i].clientAddr;
    uint32_t ts         = toolbox.i2cReadUlong(clientAddr, PM_REGISTER_UPTIME);

    sprintf(buff, "Client #%u (0x%x) uptime %u", i, clientAddr, ts);
    telnet.println(buff);
  }
}

void printClienttemps()
{
  float t0, t0H, t0L;
  float t1, t1H, t1L;
  float t2, t2H, t2L;
  uint32_t ts;
  uint8_t ca;

  for (int i=0; i<clientCount; i++)
  {
    ca = Clients[i].clientAddr;
    ts = fram[i].getTimeStamp(PM_REGISTER_READDEGCT0);
    t0 = fram[i].getDataDouble(PM_REGISTER_READDEGCT0);
    t0H = fram[i].getDataDouble(PM_REGISTER_READT0HIGH);
    t0L = fram[i].getDataDouble(PM_REGISTER_READT0LOW);
    t1 = fram[i].getDataDouble(PM_REGISTER_READDEGCT1);
    t1H = fram[i].getDataDouble(PM_REGISTER_READT1HIGH);
    t1L = fram[i].getDataDouble(PM_REGISTER_READT1LOW);
    t2 = fram[i].getDataDouble(PM_REGISTER_READDEGCT2);
    t2H = fram[i].getDataDouble(PM_REGISTER_READT2HIGH);
    t2L = fram[i].getDataDouble(PM_REGISTER_READT2LOW);
    sprintf(buff, "Client %u (0x%x) as of %u: t0: %.2f°C t0 low: %.2f°C t0 high: %.2f°C", i, ca, ts, t0, t0L, t0H);
    telnet.println(buff);

    sprintf(buff, "Client %u (0x%x) as of %u: t1: %.2f°C t1 low: %.2f°C t1 high: %.2f°C", i, ca, ts, t1, t1L, t1H);
    telnet.println(buff);

    sprintf(buff, "Client %u (0x%x) as of %u: t2: %.2f°C t2 low: %.2f°C t2 high: %.2f°C", i, ca, ts, t2, t2L, t2H);
    telnet.println(buff);
  }
}

void printClientvolts()
{
  float vB, vP, vPH, vPL;
  uint32_t ts;
  uint8_t ca;

  for (int i=0; i<clientCount; i++)
  {
    ca  = Clients[i].clientAddr;
    ts  = fram[i].getTimeStamp(PM_REGISTER_READBUSVOLTS);
    vB  = fram[i].getDataDouble(PM_REGISTER_READBUSVOLTS);
    vP  = fram[i].getDataDouble(PM_REGISTER_READPACKVOLTS);
    vPH = fram[i].getDataDouble(PM_REGISTER_READHIVOLTS);
    vPL = fram[i].getDataDouble(PM_REGISTER_READLOWVOLTS);

    sprintf(buff, "Client %u (0x%x) as of %u: vBus: %.2fv vPack: %.2fv pack high: %.2fv pack low: %.2f", i, ca, ts, vB, vP, vPH, vPL);
    telnet.println(buff);
  }
}

void printClientloads()
{
  float iLoad, iH, iL;
  uint32_t ts;
  uint8_t ca;

  for (int i=0; i<clientCount; i++)
  {
    ca     = Clients[i].clientAddr;
    ts     = fram[i].getTimeStamp(PM_REGISTER_READLOADAMPS);
    iLoad  = fram[i].getDataDouble(PM_REGISTER_READLOADAMPS);
    // iH = fram[i].getDataDouble(PM_REGISTER_READ);
    // iL = fram[i].getDataDouble(PM_REGISTER_READLOWVOLTS);

    sprintf(buff, "Client %u (0x%x) as of %u: load amps: %.2fv", i, ca, ts, iLoad);
    telnet.println(buff);
  }
}

void dumpFram()
{
  for (int i=0; i<clientCount; i++)
  {
    byte xx=0x21; // start here
    while (xx<0x65) 
    {
      sprintf(buff, 
        "Addr: 0x%X TS: %u Double: %f UINT: %u SINT: %li RAW: %li\n",
        xx,
        fram[i].getTimeStamp(xx), 
        fram[i].getDataDouble(xx),
        fram[i].getDataUInt(xx),
        fram[i].getDataSInt(xx),
        fram[i].getRaw(xx)
      );
      telnet.println(buff);
      xx++;
    }
  }
}