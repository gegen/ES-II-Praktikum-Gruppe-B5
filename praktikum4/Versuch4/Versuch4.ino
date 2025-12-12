// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee Color Dimmable light bulb.
 *
 * The example demonstrates how to use Zigbee library to create an end device with
 * color dimmable light end point.
 * The light bulb is a Zigbee end device, which is controlled by a Zigbee coordinator.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 */

#include <Arduino.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE);

#include "esp_mac.h"  // required - exposes esp_mac_type_t values

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "ZigbeeCore.h"
#include "ep/ZigbeeColorDimmableLight.h"

#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN D0

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 10

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

#define BUTTON_PIN D1  // Button on Seeed board

#define ZIGBEE_LIGHT_ENDPOINT 10

ZigbeeColorDimmableLight zbColorLight = ZigbeeColorDimmableLight(ZIGBEE_LIGHT_ENDPOINT);

void colorWipe(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
  }
  strip.show();                                  //  Update strip to match
}

String mac;

void getInterfaceMacAddress(esp_mac_type_t interface) {

  unsigned char mac_base[8] = {0};

  if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
    char buffer[17];  // 8*2 characters for hex + 1 character for null terminator
    sprintf(buffer, "%02x%02x%02x%02x%02x%02x%02x%02x", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5], mac_base[6], mac_base[7]);
    mac = buffer;
  }
}

void pre(void) {
  u8x8.setFont(u8x8_font_artossans8_r);
  u8x8.clear();

  u8x8.inverse();
  u8x8.print("Embedded Systems");
  u8x8.noInverse();
  u8x8.setCursor(0, 1);
  u8x8.print(mac);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0, 2);
}

/********************* RGB LED functions **************************/
void setRGBLight(bool state, uint8_t red, uint8_t green, uint8_t blue, uint8_t level) {
  uint8_t brightness;

  Serial.printf("setREBLight state: %d, level:%u, rgb:%u/%u/%u.\n", state, level, red, green, blue);
  brightness = state ? level : 0;

  pre();
  u8x8.draw2x2String(0, 2, "R");
  u8x8.draw2x2String(2, 2, u8x8_u8toa(red, 3));
  u8x8.draw2x2String(8, 3, "G");
  u8x8.draw2x2String(10, 3, u8x8_u8toa(green, 3));
  u8x8.draw2x2String(0, 5, "B");
  u8x8.draw2x2String(2, 5, u8x8_u8toa(blue, 3));
  u8x8.draw2x2String(8, 6, "I");
  u8x8.draw2x2String(10, 6, u8x8_u8toa(brightness, 3));

  digitalWrite(LED_BUILTIN, !state);
  strip.setBrightness(brightness);
  colorWipe(strip.Color(red, green, blue));
}

// Create a task on identify call to handle the identify function
void identify(uint16_t time) {
  static uint8_t blink = 1;
  log_d("Identify called for %d seconds", time);
  if (time == 0) {
    // If identify time is 0, stop blinking and restore light as it was used for identify
    zbColorLight.restoreLight();
    return;
  }
  digitalWrite(LED_BUILTIN, blink);
  blink = !blink;
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(9600);

  // Set on board LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Init button for factory reset
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Set Neopixel
  strip.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();   // Turn OFF all pixels ASAP
  strip.setBrightness(0);

  // Set callback function for light change
  zbColorLight.onLightChange(setRGBLight);

  // Optional: Set callback function for device identify
  zbColorLight.onIdentify(identify);

  // Optional: Set Zigbee device name and model
  zbColorLight.setManufacturerAndModel("HSD", "ES2 Gruppe B5");

  // Add endpoint to Zigbee Core
  log_d("Adding ZigbeeLight endpoint to Zigbee Core");
  Zigbee.addEndpoint(&zbColorLight);

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }

  Serial.print("Zigbee MAC: ");
  getInterfaceMacAddress(ESP_MAC_IEEE802154);
  Serial.println(mac);

  u8x8.begin();
  pre();
  u8x8.draw2x2String(0, 3, " Zigbee ");
  u8x8.draw2x2String(0, 5, "RGBLight");

  Serial.println("Setup done");
}

void printScannedNetworks(uint16_t networksFound) {
  if (networksFound == 0) {
    Serial.println("No networks found");
  } else {
    zigbee_scan_result_t *scan_result = Zigbee.getScanResult();
    Serial.println("\nScan done");
    Serial.print(networksFound);
    Serial.println(" networks found:");
    Serial.println("Nr | PAN ID | CH | Permit Joining | Router Capacity | End Device Capacity | Extended PAN ID");
    for (int i = 0; i < networksFound; ++i) {
      // Print all available info for each network found
      Serial.printf("%2d", i + 1);
      Serial.print(" | ");
      Serial.printf("0x%04hx", scan_result[i].short_pan_id);
      Serial.print(" | ");
      Serial.printf("%2d", scan_result[i].logic_channel);
      Serial.print(" | ");
      Serial.printf("%-14.14s", scan_result[i].permit_joining ? "Yes" : "No");
      Serial.print(" | ");
      Serial.printf("%-15.15s", scan_result[i].router_capacity ? "Yes" : "No");
      Serial.print(" | ");
      Serial.printf("%-19.19s", scan_result[i].end_device_capacity ? "Yes" : "No");
      Serial.print(" | ");
      Serial.printf(
        "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", scan_result[i].extended_pan_id[7], scan_result[i].extended_pan_id[6], scan_result[i].extended_pan_id[5],
        scan_result[i].extended_pan_id[4], scan_result[i].extended_pan_id[3], scan_result[i].extended_pan_id[2], scan_result[i].extended_pan_id[1],
        scan_result[i].extended_pan_id[0]);
      Serial.println();
      delay(10);
    }
    Serial.println("");
    // Delete the scan result to free memory for code below.
    Zigbee.scanDelete();
  }
}

void loop() {
  // Checking button for factory reset
  if (digitalRead(BUTTON_PIN) == LOW) {  // Push button pressed
    float tsens_value = temperatureRead();
    log_d("Temperature sensor value: %.2f°C", tsens_value);

    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(50);
      if ((millis() - startTime) > 3000) {
        // If key pressed for more than 3secs, factory reset Zigbee and reboot
        Serial.printf("Resetting Zigbee to factory settings, reboot.\n");
        Zigbee.factoryReset();
      }
    }

    // check Zigbee Network Scan process
    int16_t ZigbeeScanStatus = Zigbee.scanComplete();
    if (ZigbeeScanStatus < 0) {  // it is busy scanning or got an error
      if (ZigbeeScanStatus == ZB_SCAN_FAILED) {
        Serial.println("Zigbee scan has failed. Starting again.");
        Zigbee.scanNetworks();
      }
      // other option is status ZB_SCAN_RUNNING - just wait.
    } else {  // Found Zero or more Wireless Networks
      printScannedNetworks(ZigbeeScanStatus);
      Zigbee.scanNetworks();  // start over...
    }
  }
}