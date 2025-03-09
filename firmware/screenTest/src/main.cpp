#include <Arduino.h>
#include <TFT_eSPI.h>
#include "User_Setup.h"

// Other SPI device CS pins
#define TOUCH_CS  15
#define ADC1_CS   14
#define ADC2_CS   21

TFT_eSPI tft = TFT_eSPI();

void setup() {
  // Disable other SPI devices first
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(ADC1_CS, OUTPUT);
  digitalWrite(ADC1_CS, HIGH);
  pinMode(ADC2_CS, OUTPUT);
  digitalWrite(ADC2_CS, HIGH);

  // Initialize display
  tft.init();
  tft.setRotation(1);  // Adjust rotation as needed
  tft.fillScreen(TFT_BLACK);

  // Display test pattern
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("SPI Test", 10, 10, 2);
  
  tft.fillRect(0, 40, tft.width(), 40, TFT_RED);
  tft.fillRect(0, 90, tft.width(), 40, TFT_GREEN);
  tft.fillRect(0, 140, tft.width(), 40, TFT_BLUE);
  
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.drawCentreString("SUCCESS!", tft.width()/2, 190, 2);
}

void loop() {
  // Add touch test logic here if needed
}