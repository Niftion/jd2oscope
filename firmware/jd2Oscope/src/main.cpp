#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <esp_timer.h>
#include <esp_err.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

#include <TFT_eSPI.h>  // Bodmer's TFT_eSPI library
#include "User_Setup.h"
#include <TFT_eWidget.h>               // Widget library

TFT_eSPI tft = TFT_eSPI();


GraphWidget gr = GraphWidget(&tft);    // Graph widget gr instance with pointer to tft
TraceWidget tr1 = TraceWidget(&gr);    // Graph trace 1
TraceWidget tr2 = TraceWidget(&gr);    // Graph trace 2

const float gxLow  = 0.0;
const float gxHigh = 100.0;
const float gyLow  = -512.0;
const float gyHigh = 512.0;
float display_data[100];
float real_data[100];
float trigger_data[200];
volatile byte pause_state = LOW;
volatile byte trigger_state = LOW;



/*

*/


//int ADC_val_converter(int example_input);
//static double volts_to_bits(double volt_in);
//static double bits_to_volts(double bit_in);


//Display Screen Functions

unsigned long testFillScreen();
unsigned long testText();
//unsigned long testProportionalText();
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
int findMax(float array[]);
int findMax(float array[]);
void pause_button();
void trigger_set();
void trigger_function();



void setup() {

  //tft.begin();
  // Note: you can now set the SPI speed to any value
  // the default value is 30Mhz, but most ILI9341 displays
  // can handle at least 60Mhz and as much as 100Mhz
  //  tft.setClock(60000000);
    //tft.fillScreen(ILI9341_BLACK);
    //tft.setTextColor(ILI9341_YELLOW);
    //tft.setTextSize(2);
    //tft.println("Waiting for Arduino Serial Monitor...");
  //********************************************************************
  // Graph Setup
  //********************************************************************
  Serial.begin(115200);
  delay(5000);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  // Graph area is 200 pixels wide, 150 high, dark grey background
  gr.createGraph(290, 190, tft.color565(5, 5, 5));
  // x scale units is from 0 to 100, y scale units is -50 to 50
  gr.setGraphScale(0.0, 100.0, -50.0, 50.0);
  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at -50 with lines every 25 y-scale units
  // blue grid
  gr.setGraphGrid(0.0, 10.0, -50.0, 10.0, TFT_BLUE);
  // Draw empty graph, top left corner at 40,10 on TFT
  gr.drawGraph(20, 20);
  // Start a trace with using red and another with green
  tr1.startTrace(TFT_RED);
  tr2.startTrace(TFT_GREEN);
  // Add points on graph to trace 1 using graph scale factors
  tr1.addPoint(0.0, 0.0);
  tr1.addPoint(50.0, 0.0);
  // Add points on graph to trace 2 using graph scale factors
  // Points are off graph so the plotted line is clipped to graph area
  tr2.addPoint(0.0, -50.0);
  tr2.addPoint(50.0, 50.0);
  // Get x,y pixel coordinates of any scaled point on graph
  // and ring that point.
  tft.drawCircle(gr.getPointX(50.0), gr.getPointY(0.0), 5, TFT_MAGENTA);

  // Draw the x axis scale
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(-50.0) + 3);
  tft.drawNumber(50, gr.getPointX(50.0), gr.getPointY(-50.0) + 3);
  tft.drawNumber(100, gr.getPointX(100.0), gr.getPointY(-50.0) + 3);

  // Draw the y axis scale
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  tft.drawNumber(-50, gr.getPointX(0.0), gr.getPointY(-50.0));
  tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(0.0));
  tft.drawNumber(50, gr.getPointX(0.0), gr.getPointY(50.0));

  //attempt to get labels right
  tft.setRotation(2);
  tft.setCursor(100, 5);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Voltage (V)");
  tft.setRotation(3);
  tft.setCursor(150, 225);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Time (sec)");
  tft.setCursor(150, 5);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Trigger: Falling");
  tft.setCursor(50, 5);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Amplitude: ");
  tft.setCursor(275, 5);
  tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(1);
  tft.println("CH1");
  tft.setCursor(300, 5);
  tft.setTextColor(ILI9341_GREEN);  tft.setTextSize(1);
  tft.println("CH2");
 

  // Restart traces with new colours
  tr1.startTrace(TFT_WHITE);
  tr2.startTrace(TFT_YELLOW);
  

  //++++++++++++Interrupts for Pausing, Trigger, and Channel Select
  //int pause_pin
  //int trigger_pin
  //int Ch1_pin
  //int Ch2_pin
  pinMode(26, INPUT_PULLUP);// Using GPIO 6 for button testing

  attachInterrupt(digitalPinToInterrupt(26), trigger_set, FALLING); //was pause_button, now trigger_state
  //attachInterrupt(digitalPinToInterrupt(pause_pin), pause_button, FALLING);
  //attachInterrupt(digitalPinToInterrupt(trigger_pin), trigger_set, FALLING);
}

void loop() {
  
  while(!pause_state){
    tft.fillRect(150, 5, 100, 10, TFT_BLACK);
    tft.setCursor(150, 5);
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);

    if(!trigger_state){
      tft.println("Trigger: Rising");
    }else{
      tft.println("Trigger: Falling");
    }
  trigger_function();
  static uint32_t plotTime = micros(); // used to be = millis(), now using micros
  int Amplitude = (findMax(real_data) - findMin(real_data))/2;
  tft.setCursor(110, 5);
  tft.fillRect(110, 5, 10, 10, TFT_BLACK);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println(Amplitude);
  int y_zoom = analogRead(34) + 1; // GPIO 34
  int x_zoom = analogRead(36) + 1;
  int val = analogRead(35);
  static float gx = 0.0, gy = 0.0;
  static float delta = 7.0;
  int SampleTime = 4095/(x_zoom);
  
  //Time (x) axis zoom
  tft.fillRect(0, gr.getPointY(-50.0) + 3, 320, 10, TFT_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(-50.0) + 3);
  tft.drawNumber((409.5/(.2*x_zoom)), gr.getPointX(50.0), gr.getPointY(-50.0) + 3);
  tft.drawNumber((409.5/(.1*x_zoom)), gr.getPointX(100.0), gr.getPointY(-50.0) + 3);

  //voltage (y) axis zoom
  tft.fillRect(0, 0, 19, 220, TFT_BLACK);
  tft.setRotation(2);
  tft.setCursor(100, 5);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Voltage (V)");
  tft.setRotation(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  tft.drawNumber((((-50)*y_zoom)/4095), gr.getPointX(0.0), gr.getPointY(-50.0));
  tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(0.0));
  tft.drawNumber((((50)*y_zoom)/4095), gr.getPointX(0.0), gr.getPointY(50.0));
  
  if (micros() - plotTime >= SampleTime) { //edited from sampletime
    plotTime = micros();

    // Add a new point on each trace
    //tr1.addPoint(gx, gy);
    display_data[int(gx)] = gy;
    real_data[int(gx)] = display_data[int(gx)]*(y_zoom/4095);
    tr2.addPoint(gx, gy);
    //tr2.addPoint(gx, gy/2.0); // half y amplitude

    // Create next plot point
    gx += 1.0;
    //gy += delta;
    gy = ((val/41)-50)*(4095/y_zoom) ;//+= delta;
    //if (gy >  70.0) { delta = -7.0; gy =  70.0; }
    //if (gy < -70.0) { delta =  7.0; gy = -70.0; }

    // If the end of the graph is reached start 2 new traces
    if (gx > 100) {
      gx = 0.0;
      gy = display_data[99];

      // Draw empty graph at 40,10 on display
      gr.drawGraph(20, 20);
    
      // Start new trace
      //tr1.startTrace(TFT_GREEN);
      tr2.startTrace(TFT_YELLOW);
    }
    
  }
}
}    

//Functions for Display Testing
void trigger_set(){
  trigger_state = !trigger_state;
  return;
}

void trigger_function(){
  static uint32_t plotTime_tgr = micros(); // used to be = millis(), now using micros
  for (int i = 0; i<200;i++){
    if (micros() - plotTime_tgr >= 10) { //edited from sampletime
      plotTime_tgr = micros();
      int val_tgr = analogRead(35);
      int y_zoom_tgr = 4095;//analogRead(36) + 1; // GPIO 36
      static float gy_tgr = ((val_tgr/41)-50)*(4095/y_zoom_tgr);
      trigger_data[i] = gy_tgr;
    }  
  }
  if (trigger_state){                                         // rising edge trigger logic
    for (int i = 0; i<200;i++){
      if (trigger_data[i]<=trigger_data[i+1]){
        if(trigger_data[i+1]<=trigger_data[i+2]){
          if(trigger_data[i+2]<=trigger_data[i+3]){
            if(trigger_data[i+3]<=trigger_data[i+4]){
              for (int j = i; j < 100; j++){
                display_data[i] = trigger_data[i];
              }

            }

          }

        }
      }

    }
  } else {
    for (int i = 0; i<200;i++){
      if (trigger_data[i]>=trigger_data[i+1]){
        if(trigger_data[i+1]>=trigger_data[i+2]){
          if(trigger_data[i+2]>=trigger_data[i+3]){
            if(trigger_data[i+3]>=trigger_data[i+4]){
              for (int j = i; j < 100; j++){
                display_data[i] = trigger_data[i];
              }

            }

          }

        }
      }

    }
  }

  
  return;
}

void pause_button(){
  pause_state = !pause_state;
  return;
}



int findMax(float array[]){
  int max = array[0];
  for (int i=0; i < 100; i++){
    if (array[i] > max){
      max = array[i];
    }

  }
  return max;
}

int findMin(float array[]){
  int min = array[0];
  for (int i=0; i < 100; i++){
    if (array[i] < min){
      min = array[i];
    }

  }
  return min;
}


unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_RED);
  tft.fillScreen(ILI9341_GREEN);
  tft.fillScreen(ILI9341_BLUE);
  tft.fillScreen(ILI9341_BLACK);
  return micros() - start;
}
 
unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}
 
unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();
 
  tft.fillScreen(ILI9341_BLACK);
 
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing
 
  tft.fillScreen(ILI9341_BLACK);
 
  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;
 
  tft.fillScreen(ILI9341_BLACK);
 
  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;
 
  tft.fillScreen(ILI9341_BLACK);
 
  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
 
  return micros() - start;
}
 
unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();
 
  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);
 
  return micros() - start;
}
 
unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;
 
  tft.fillScreen(ILI9341_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }
 
  return micros() - start;
}
 
unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;
 
  tft.fillScreen(ILI9341_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
  }
 
  return t;
}
 
unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;
 
  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
 
  return micros() - start;
}
 
unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;
 
  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }
 
  return micros() - start;
}
 
unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;
 
  tft.fillScreen(ILI9341_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(0, 0, i));
  }
 
  return micros() - start;
}
 
unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;
 
  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i, i));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i, i, 0));
  }
 
  return t;
}
 
unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;
 
  tft.fillScreen(ILI9341_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }
 
  return micros() - start;
}
 
unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;
 
  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
  }
 
  return micros() - start;
}
  

  /*
int ADC_val_converter(int example_input){
  int min = 0b0;
  int max = 0b11111111111111;
  int v_zoom = 1;
  int lcd_height = 1024;
  //int example_input = 0b00001110101011;
  int volt_scale_factor = ((max-min)/lcd_height)*v_zoom;
  int result = (example_input - (volts_to_bits(1.25)))*4;
  return result; 


}
*/

