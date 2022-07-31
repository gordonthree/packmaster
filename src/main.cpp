#include <Arduino.h>
#include <Wire.h>
#include <PolledTimeout.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "ESPTelnet.h"          
#include <time.h>
#include "TimeLib.h"
#include "sntp.h"

#define SERIAL_SPEED 115200

#define SDA_PIN 4
#define SCL_PIN 5

#define STASSID "Tell my WiFi I love her"
#define STAPSK  "2317239216"

const char* ssid = STASSID;
const char* password = STAPSK;

const char* ntpServerName = "pool.ntp.org";   // NTP server name
const long  gmtOffset_sec = -14400;            //Replace with your GMT offset (seconds)
const int   daylightOffset_sec = 0;        //Replace with your daylight offset (seconds)

const uint8_t I2C_MASTER = 0x42;
const uint8_t I2C_SLAVE = 0x37;

String newHostname = "packmaster";

IPAddress ntpServerIP;          // time.nist.gov NTP server address

ESPTelnet telnet;
IPAddress ip;
uint16_t  port = 23;
uint16_t  loopCnt = 0;          // loop counter to update slave time

char buff[100];

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
    } else if (str == "bye") {
      telnet.println("> disconnecting you...");
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
  Wire.begin(SDA_PIN, SCL_PIN, I2C_MASTER);        // join i2c bus (address optional for master)
  Wire.setClock(100000);  // 100khz clock
  
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
    printLocalTime();
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

struct timeArray_t{
  byte regAddr;
  uint32_t timeStamp;
};

const uint8_t timeUnion_size = sizeof(timeArray_t);

union I2C_timePacket_t{
  timeArray_t currentTime;
  byte I2CPacket[timeUnion_size];
};

void loop() {
  telnet.loop();
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);

  ArduinoOTA.handle();

  if (nextPing) {
    loopCnt++;                                        // increment loop counter
    uint32 timeNow = sntp_get_current_timestamp();    // get unix style timestamp from ntp provider
    // telnetLocalTime();                                // print the current time
    sprintf(buff, "timestamp = %u", timeNow);
    telnet.println(buff);

    telnet.print("RX: ");
    Wire.requestFrom(I2C_SLAVE, 6);                   // request 6 bytes from slave device followed by a stop condition
    while (Wire.available()) {                        // slave may send less than requested
      char c = Wire.read();                           // receive a byte as character
      telnet.print(c);                                // print the character
    }
    telnet.println(" RX complete.");


    // now try writing some data
    telnet.print("TX: ");

    I2C_timePacket_t txData;
    txData.currentTime.regAddr = 0x20;
    txData.currentTime.timeStamp = sntp_get_current_timestamp();


    Wire.beginTransmission(I2C_SLAVE);              // begin transaction with slave address
    Wire.write(txData.currentTime.regAddr);                             // send time packet populated above
    Wire.write(txData.I2CPacket, timeUnion_size - 1);                             // send time packet populated above
    Wire.endTransmission(true);                     // end transaction with a stop
    //loopCnt = 1;

    Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    Wire.write(0x31);                                 // register address
    Wire.endTransmission(true);                       // end transaction with a stop

    Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    Wire.write(0x32);                                 // register address
    Wire.write("Hello!");                             // send some data
    Wire.endTransmission(true);                       // end transaction with a stop
    
    Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    Wire.write(0x30);                                 // register address
    Wire.endTransmission(true);                       // end transaction with a stop
    telnet.println("complete.\n");
  }
}

