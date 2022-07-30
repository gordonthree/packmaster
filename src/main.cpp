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

ESPTelnet telnet;
IPAddress ip;
uint16_t  port = 23;

void onTelnetConnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");
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

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
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
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void loop() {
  telnet.loop();
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);
  ArduinoOTA.handle();
  
  if (nextPing) {
    
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
    //Wire.write(0x00);                               // register address
    Wire.write("1");                                  // send some data
    Wire.write("0");                                  // send some data
    Wire.write("2");                                  // send some data
    Wire.write("4");                                  // send some data
    uint8_t tx = Wire.endTransmission(true);          // end transaction with a stop
    telnet.print("status ");
    telnet.println(tx);

  }
}

