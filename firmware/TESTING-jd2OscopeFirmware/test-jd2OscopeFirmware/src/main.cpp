#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "User_Setup.h"
#include "Oscilloscope.h"

// Hardware Definitions
const int screen_CS = 5;
const int screen_DC = 4;
const int touch_CS = 15;
const int touch_IRQ = 25;

// Global Controls
const int triggerBttn = 13;
const int ampScale_wiper = 33;
const int timeScale_wiper = 32;

// Signal 1 Controls
const int signal1_toggleBttn = 16;
const int signal1_ampShiftWiper = 34;
const int signal1_timeShiftWiper = 35;

// ADC1 Configuration
const int ADC1_CS = 14;
const int ADC1_CONVST = 27;
const int ADC1_EOC = 26;

// Signal 2 Controls
const int signal2_toggleBttn = 17;
const int signal2_ampShiftWiper = 36;
const int signal2_timeShiftWiper = 39;

// ADC2 Configuration
const int ADC2_CS = 21;
const int ADC2_CONVST = 22;
const int ADC2_EOC = 12;

// Oscilloscope Parameters
volatile uint16_t adc1_buffer[BUFFER_SIZE];
volatile uint16_t adc2_buffer[BUFFER_SIZE];
volatile uint16_t buffer_index = 0;
volatile bool buffer_ready = false;

// Display Configuration
TFT_eSPI tft = TFT_eSPI();

// Control States
volatile bool ch1_enabled = true, ch2_enabled = true;
volatile uint8_t trigger_mode = 0; // 0=Rising, 1=Falling, 2=Pulse

// Debounce timing (PlatformIO compatible)
volatile unsigned long last_ch1_toggle = 0;
volatile unsigned long last_ch2_toggle = 0;
volatile unsigned long last_trigger_press = 0;

// ================== Interrupt Handlers ==================
void IRAM_ATTR ADC1_ISR() {
  static uint32_t last_conversion = 0;
  if(micros() - last_conversion >= 1) {
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
    digitalWrite(ADC1_CS, LOW);
    adc1_buffer[buffer_index] = SPI.transfer16(0x0000) >> 2;
    digitalWrite(ADC1_CS, HIGH);
    SPI.endTransaction();
    last_conversion = micros();
  }
}

void IRAM_ATTR ADC2_ISR() {
  static uint32_t last_conversion = 0;
  if(micros() - last_conversion >= 1) {
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
    digitalWrite(ADC2_CS, LOW);
    adc2_buffer[buffer_index] = SPI.transfer16(0x0000) >> 2;
    digitalWrite(ADC2_CS, HIGH);
    SPI.endTransaction();
    last_conversion = micros();
  }
}

void IRAM_ATTR handle_ch1_toggle() {
  if((millis() - last_ch1_toggle) > DEBOUNCE_TIME) {
    ch1_enabled = !ch1_enabled;
    last_ch1_toggle = millis();
  }
}

void IRAM_ATTR handle_ch2_toggle() {
  if((millis() - last_ch2_toggle) > DEBOUNCE_TIME) {
    ch2_enabled = !ch2_enabled;
    last_ch2_toggle = millis();
  }
}

void IRAM_ATTR handle_trigger_mode() {
  if((millis() - last_trigger_press) > DEBOUNCE_TIME) {
    trigger_mode = (trigger_mode + 1) % 3;
    last_trigger_press = millis();
  }
}

// ================== Core Functions ==================
void initializeADC() {
  // ADC1 Setup
  pinMode(ADC1_CS, OUTPUT);
  pinMode(ADC1_CONVST, OUTPUT);
  pinMode(ADC1_EOC, INPUT_PULLUP);
  digitalWrite(ADC1_CS, HIGH);

  // ADC2 Setup
  pinMode(ADC2_CS, OUTPUT);
  pinMode(ADC2_CONVST, OUTPUT);
  pinMode(ADC2_EOC, INPUT_PULLUP);
  digitalWrite(ADC2_CS, HIGH);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ADC1_EOC), ADC1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ADC2_EOC), ADC2_ISR, FALLING);
}

bool check_trigger(uint16_t sample) {
  static uint16_t prev_sample = 0;
  bool triggered = false;

  switch(trigger_mode) {
    case 0: // Rising edge
      triggered = (prev_sample < 8192) && (sample >= 8192);
      break;
    case 1: // Falling edge
      triggered = (prev_sample >= 8192) && (sample < 8192);
      break;
    case 2: { // Pulse width
      static bool pulse_state = false;
      static uint32_t pulse_start = 0;
      if(pulse_state && (sample < 8192)) {
        if(micros() - pulse_start > 1000) triggered = true;
        pulse_state = false;
      }
      if(!pulse_state && (sample >= 8192)) {
        pulse_start = micros();
        pulse_state = true;
      }
      break;
    }
  }
  prev_sample = sample;
  return triggered;
}

void handleControls() {
  static float ch1_vert_scale = 1.0, ch1_horiz_scale = 1.0;
  static float ch2_vert_scale = 1.0, ch2_horiz_scale = 1.0;
  
  // Read signal-specific potentiometers
  ch1_vert_scale = analogRead(signal1_ampShiftWiper) / 4095.0 * 2.0;
  ch1_horiz_scale = analogRead(signal1_timeShiftWiper) / 4095.0 * 2.0;
  ch2_vert_scale = analogRead(signal2_ampShiftWiper) / 4095.0 * 2.0;
  ch2_horiz_scale = analogRead(signal2_timeShiftWiper) / 4095.0 * 2.0;

  // Apply scaling
  for(int i=0; i<BUFFER_SIZE; i++) {
    if(ch1_enabled) adc1_buffer[i] = constrain(adc1_buffer[i] * ch1_vert_scale, 0, 16383);
    if(ch2_enabled) adc2_buffer[i] = constrain(adc2_buffer[i] * ch2_vert_scale, 0, 16383);
  }
}

void updateDisplay() {
  static uint32_t last_refresh = 0;
  const uint32_t refresh_interval = 16667;
  
  if(micros() - last_refresh < refresh_interval) return;
  last_refresh = micros();

  // Channel status indicators
  tft.fillRect(0, 0, 30, 20, TFT_BLACK);
  tft.setTextColor(ch1_enabled ? TFT_YELLOW : TFT_DARKGREY);
  tft.drawString("CH1", 5, 0);
  tft.setTextColor(ch2_enabled ? TFT_GREEN : TFT_DARKGREY);
  tft.drawString("CH2", 5, 10);

  // Trigger mode display
  const char* trigger_labels[] = {"Rising", "Falling", "Pulse"};
  tft.fillRect(200, 0, 80, 10, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(trigger_labels[trigger_mode], 200, 0);

  // Waveform drawing
  tft.startWrite();
  for(int x = 0; x < GRAPH_WIDTH; x++) {
    int idx = (buffer_index + x) % BUFFER_SIZE;
    
    if(ch1_enabled) {
      int y1 = map(adc1_buffer[idx], 0, 16383, GRAPH_HEIGHT, 0);
      tft.drawPixel(x + 20, y1 + 20, TFT_YELLOW);
    }
    
    if(ch2_enabled) {
      int y2 = map(adc2_buffer[idx], 0, 16383, GRAPH_HEIGHT, 0);
      tft.drawPixel(x + 20, y2 + 20, TFT_GREEN);
    }
  }
  tft.endWrite();
}

// ================== Main Program ==================
void setup() {
  Serial.begin(115200);
  SPI.begin();

  // Initialize Display
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(19, 19, GRAPH_WIDTH+2, GRAPH_HEIGHT+2, TFT_WHITE);

  // Initialize Controls
  pinMode(signal1_toggleBttn, INPUT_PULLUP);
  pinMode(signal2_toggleBttn, INPUT_PULLUP);
  pinMode(triggerBttn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(signal1_toggleBttn), handle_ch1_toggle, FALLING);
  attachInterrupt(digitalPinToInterrupt(signal2_toggleBttn), handle_ch2_toggle, FALLING);
  attachInterrupt(digitalPinToInterrupt(triggerBttn), handle_trigger_mode, FALLING);

  // Initialize ADCs
  initializeADC();
}

void loop() {
  // Start conversions
  digitalWrite(ADC1_CONVST, LOW);
  digitalWrite(ADC2_CONVST, LOW);
  delayMicroseconds(1);
  digitalWrite(ADC1_CONVST, HIGH);
  digitalWrite(ADC2_CONVST, HIGH);

  // Handle controls and display
  handleControls();
  updateDisplay();

  // Buffer management
  if(buffer_ready) {
    noInterrupts();
    buffer_index = 0;
    buffer_ready = false;
    interrupts();
  }
}