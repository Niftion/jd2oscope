#ifndef OSCILLOSCOPE_H
#define OSCILLOSCOPE_H

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

// Hardware Constants
const uint16_t BUFFER_SIZE = 1024;
const int GRAPH_WIDTH = 290; //px
const int GRAPH_HEIGHT = 190; //px
const uint8_t DEBOUNCE_TIME = 50; // ms
const uint32_t MAX_SAMPLE_RATE = 1000000; //Hz

// Global Variables
extern volatile uint16_t adc1_buffer[BUFFER_SIZE];
extern volatile uint16_t adc2_buffer[BUFFER_SIZE];
extern volatile uint16_t buffer_index;
extern volatile bool buffer_ready;
extern volatile bool ch1_enabled, ch2_enabled;
extern volatile uint8_t trigger_mode;
extern volatile unsigned long last_ch1_toggle;
extern volatile unsigned long last_ch2_toggle;
extern volatile unsigned long last_trigger_press;

// Function Declarations
void initializeADC();
bool check_trigger(uint16_t sample);
void handleControls();
void updateDisplay();
void IRAM_ATTR ADC1_ISR();
void IRAM_ATTR ADC2_ISR();
void IRAM_ATTR handle_ch1_toggle();
void IRAM_ATTR handle_ch2_toggle();
void IRAM_ATTR handle_trigger_mode();

#endif