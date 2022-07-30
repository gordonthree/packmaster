#include <Arduino.h>

#include <Wire.h>
#include <PolledTimeout.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define SDA_PIN 4
#define SCL_PIN 5

#define STASSID "Tell my WiFi I love her"
#define STAPSK  "2317239216"

const char* ssid = STASSID;
const char* password = STAPSK;

const int16_t I2C_MASTER = 0x42;
const int16_t I2C_SLAVE = 0x37;

String newHostname = "packmaster";

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

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

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
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);
  ArduinoOTA.handle();
  
  if (nextPing) {
    Wire.requestFrom(I2C_SLAVE, 6);    // request 6 bytes from slave device #8

    while (Wire.available()) { // slave may send less than requested
      char c = Wire.read(); // receive a byte as character
      Serial.print(c);         // print the character
    }

    // now try writing some data
    Wire.beginTransmission(I2C_SLAVE);   // begin transaction with slave address
    //Wire.write(0x00);                    // register address
    Wire.write("1");                     // send some data
    Wire.write("0");                     // send some data
    Wire.write("2");                     // send some data
    Wire.write("4");                     // send some data
    Wire.endTransmission();              // end transaction

  }
}

