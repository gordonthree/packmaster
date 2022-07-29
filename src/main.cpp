#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32DMASPIMaster.h>

ESP32DMASPI::Master master;

const char* ssid = "Tell my WiFi I love her";
const char* password = "2317239216";

static const uint32_t BUFFER_SIZE = 32;

uint8_t* spi_master_tx_buf;
uint8_t* spi_master_rx_buf;

void set_buffer() {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        spi_master_tx_buf[i] = i & 0xFF;
    }
    memset(spi_master_rx_buf, 0, BUFFER_SIZE);
}


#define VSPI_MISO   MISO
#define VSPI_MOSI   MOSI
#define VSPI_SCLK   SCK
#define VSPI_SS     SS

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Booting");

  // to use DMA buffer, use these methods to allocate buffer
  spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
  spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);

  set_buffer();
  Serial.println("SPI Buffers Initialized");

  master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
  master.setFrequency(100000);            // default: 8MHz (too fast for bread board...)
  master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes

  // begin() after setting
  master.begin();  // default: HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12)
  Serial.println("SPI master init complete");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop(void)
{
    // start and wait to complete transaction
    master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);

    // show received data (if needed)
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        Serial.printf("%d\n ", spi_master_rx_buf[i]);
    }

    for (uint8_t x=0; x<200; x++){
      ArduinoOTA.handle();
      delay(1);
    }
}


