#include <Arduino.h>
#include <Wire.h>
#include <PolledTimeout.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "ESPTelnet.h"          

#define SERIAL_SPEED 115200

#define SDA_PIN 4
#define SCL_PIN 5

#define STASSID "Tell my WiFi I love her"
#define STAPSK  "2317239216"

const char* ssid = STASSID;
const char* password = STAPSK;

const int16_t I2C_MASTER = 0x42;
const int16_t I2C_SLAVE = 0x37;

String newHostname = "packmaster";

IPAddress timeServerIP;          // time.nist.gov NTP server address

const char* NTPServerName = "us.pool.ntp.org";
const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message

byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

WiFiUDP UDP;                     // Create an instance of the WiFiUDP class to send and receive
ESPTelnet telnet;
IPAddress ip;
uint16_t  port = 23;

unsigned long intervalNTP = 60000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t timeUNIX = 0;

unsigned long prevActualTime = 0;

void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages on port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
  Serial.println();
}

uint32_t getTime() {
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}

void sendNTPpacket(IPAddress& address) {
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

inline int getSeconds(uint32_t UNIXTime) {
  return UNIXTime % 60;
}

inline int getMinutes(uint32_t UNIXTime) {
  return UNIXTime / 60 % 60;
}

inline int getHours(uint32_t UNIXTime) {
  return UNIXTime / 3600 % 24;
}

void onTelnetConnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");

  uint32_t time = getTime();                   // Check if an NTP response has arrived and get the (UNIX) time

  if (time) {                                  // If a new timestamp has been received
    timeUNIX = time;
    telnet.print("NTP response: ");
    telnet.println(timeUNIX);
  }
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

void setup() {
  Serial.begin(115200);  // start serial for output
  Wire.begin(SDA_PIN, SCL_PIN, I2C_MASTER);        // join i2c bus (address optional for master)
  Wire.setClock(100000);  // 100khz clock
  
  delay(2000);
  
  Serial.println("\n\fBooting");
  
  WiFi.mode(WIFI_STA);
  WiFi.hostname(newHostname.c_str());
  
  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  setupTelnet();

  startUDP();

  if(!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
    telnet.println("DNS lookup failed.");
    //Serial.flush();
    //ESP.reset();
  } else {
    telnet.print("Time server IP:\t");
    telnet.println(timeServerIP);
    
    telnet.println("\r\nSending NTP request ...");
    sendNTPpacket(timeServerIP); 
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
  unsigned long currentMillis = millis();
  telnet.loop();
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);
  ArduinoOTA.handle();

  if (nextPing) {
    uint32_t time = getTime();                   // Check if an NTP response has arrived and get the (UNIX) time
    
    if (time) {                                  // If a new timestamp has been received
      timeUNIX = time;
      telnet.print("NTP response: ");
      telnet.println(timeUNIX);
      lastNTPResponse = currentMillis;
    } else if ((currentMillis - lastNTPResponse) > 3600000) {
      telnet.println("Sending NTP request ...");
      sendNTPpacket(timeServerIP);               // Send an NTP request
    }

    telnet.print("RX: ");
    Wire.requestFrom(I2C_SLAVE, 6);                   // request 6 bytes from slave device followed by a stop condition

    while (Wire.available()) {                        // slave may send less than requested
      char c = Wire.read();                           // receive a byte as character
      telnet.print(c);                                // print the character
    }

    telnet.println(" RX complete.");


    telnet.print("TX: ");
    // now try writing some data
    Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    Wire.write(0x21);                                 // register address
    Wire.endTransmission(true);                       // end transaction with a stop

    Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    Wire.write(0x32);                                 // register address
    Wire.write("Hello!");                             // send some data
    Wire.endTransmission(true);                       // end transaction with a stop
    
    Wire.beginTransmission(I2C_SLAVE);                // begin transaction with slave address
    Wire.write(0x20);                                 // register address
    Wire.endTransmission(true);                       // end transaction with a stop
    telnet.println("complete.");
  }
}

