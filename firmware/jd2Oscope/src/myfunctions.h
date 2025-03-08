//*************************************************
//Function Declarations
//*************************************************

#ifndef MYFUNCTIONS_H
#define MYFUNCTIONS_H

#include <Arduino.h>
#include <TFT_eSPI.h>

// Test and demo functions
unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();

// Utility functions
int findMax(float array[]);
int findMin(float array[]);

// Control and ADC functions
void pause_button();
void trigger_set();
void trigger_function();
void ADC_Set_Default();
int ADC_collect_data();

#endif // MYFUNCTIONS_H
