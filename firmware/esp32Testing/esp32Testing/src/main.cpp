#include <Arduino.h>

// --- Pin definitions ---
const int buttonPin = 35;  // Button input pin (requires external pull-up)
const int potPin    = 32;  // Potentiometer analog input

// --- Button debounce variables ---
bool stableButtonState = HIGH;     // Last stable button state (assumes pull-up: default HIGH)
bool lastRawButtonState = HIGH;      // Last raw reading
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50 ms debounce

// --- Timing for periodic refresh ---
unsigned long lastPeriodicTime = 0;
const unsigned long periodicInterval = 500;  // 500 ms refresh

// --- For building the status line ---
String currentStatusLine = "";
String buttonEventMsg = "";  // Temporary message for immediate button events

// Utility: compose the current periodic status line
String composeStatusLine(int potValue, bool logicLevel) {
  String line = "[Periodic] [Pot Value] = " + String(potValue) +
                " | [Logic Level] = " + String(logicLevel ? 1 : 0);
  // Append any pending button event message if available
  if (buttonEventMsg.length() > 0) {
    line += " " + buttonEventMsg;
  }
  return line;
}

void setup() {
  Serial.begin(115200);
  // For GPIO 35, no internal pull-up; ensure you have an external resistor.
  pinMode(buttonPin, INPUT);
  
  Serial.println("=== ESP32 GPIO Demo Start ===");
  Serial.println("Reading button on pin 35 and pot on pin 32.");
  // Print an initial status line so that subsequent updates can overwrite it
  int potVal = analogRead(potPin);
  currentStatusLine = composeStatusLine(potVal, digitalRead(buttonPin));
  Serial.println(currentStatusLine);
}

void loop() {
  // --- Read and debounce the button ---
  bool rawState = digitalRead(buttonPin);
  
  if (rawState != lastRawButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (rawState != stableButtonState) {
      stableButtonState = rawState;
      // Build a button event message (without a newline)
      if (stableButtonState == LOW) {
        buttonEventMsg = "[BUTTON EVENT] Pressed (LOW)";
      } else {
        buttonEventMsg = "[BUTTON EVENT] Released (HIGH)";
      }
      
      // Immediately update the status line on the same (last printed) line:
      // Move cursor up one line, then carriage return to start of that line.
      Serial.print("\033[F");  // ANSI code: move cursor up one line
      Serial.print("\r");      // Carriage return
      // Recompose status line with the new button event message and reprint it.
      int potVal = analogRead(potPin);
      currentStatusLine = composeStatusLine(potVal, stableButtonState);
      Serial.println(currentStatusLine);
    }
  }
  
  lastRawButtonState = rawState;
  
  // --- Periodic update every 200 ms ---
  if (millis() - lastPeriodicTime >= periodicInterval) {
    lastPeriodicTime = millis();
    
    // Get current pot reading
    int potVal = analogRead(potPin);
    // Recompose status line. (Clear the temporary button event after using it.)
    currentStatusLine = composeStatusLine(potVal, stableButtonState);
    Serial.println(currentStatusLine);
    // Clear button event message so it doesn't persist past one update.
    buttonEventMsg = "";
  }
}
