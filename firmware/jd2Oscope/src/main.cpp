#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <esp_timer.h>
#include <esp_err.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "SPI.h"


#include <TFT_eSPI.h>  // Bodmer's TFT_eSPI library
#include "User_Setup.h"
#include <TFT_eWidget.h>               // Widget library


TFT_eSPI tft = TFT_eSPI();




GraphWidget gr = GraphWidget(&tft);    // Graph widget gr instance with pointer to tft
TraceWidget tr1 = TraceWidget(&gr);    // Graph trace 1
TraceWidget tr2 = TraceWidget(&gr);    // Graph trace 2


//****************************************
//Variable Declarations
//****************************************


const float gxLow  = 0.0;
const float gxHigh = 100.0;
const float gyLow  = -512.0;
const float gyHigh = 512.0;
const int graph_length = 290;
const int graph_height = 190;
float graph_scale_xmin = 0.0;
float graph_scale_xmax = 100.0;
float graph_scale_ymin = -50.0;
float graph_scale_ymax = 50.0;
float graph_scale_xgrid = 10.0;
float graph_scale_ygrid = 10.0;
int graph_top_corner_x = 20;
int graph_top_corner_y = 20;


//Global Vars
int triggerBttn = 13; //GPIO 13
int SDO_pin = 19;
int SDI_pin = 23;
int ampScale_wiper = 33;
int timeScale_wiper = 32;
uint8_t ADC1_valH;
uint8_t ADC1_valL;
uint8_t ADC2_valH;
uint8_t ADC2_valL;
uint16_t ADC_14bit_val=0;

//Screen
int screen_CS = 5;
int screen_DC = 4;

//Signal 1
int signal1_toggleBttn = 16;

int signal1_ampShift_Wiper = 34;
int signal1_timeShiftWiper = 35;

int ADC1_EocInt = 26; // GPIO 26; intended for an interrupt (used to tell us when the ADC is done recording data)
int ADC1_CS_pin = 14; //  GPIO 14, Chip select pin for ADC (Slave select pin for SPI)
int ADC1_CONVST_pin = 27; //GPIO 27 (used to trigger a new conversion; best to drive it from a stable timer or a PWM output from the esp32 to achieve a desired sampling rate)
//int ADC_raw_val;

//Signal 2
int signal2_toggleBttn =  17;

int signal2_ampShiftWiper = 36;
int signal2_timeShiftWiper = 39;

int ADC2_CS_pin = 21;
int ADC2_CONVST_pin = 22;
int ADC2_EocInt = 12;

//int V_dial_pin = 33;
//int T_dial_pin = 32;

//-------------------------------
float display_data[100];
float real_data[100];
float trigger_data[200];
volatile byte pause_state = LOW;
volatile byte trigger_state = LOW;


volatile int spi_max_clk = 50000000;// Max SPI clk frequency is 50MHz.




//int ADC_val_converter(int example_input);
//static double volts_to_bits(double volt_in);
//static double bits_to_volts(double bit_in);




//*************************************************
//Function Declarations
//*************************************************


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
void ADC_collect_data();
void ADC_Set_Default();
void pauseTFTComms();
void startTFTComms();





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
  //SPI Interface Setup
  //********************************************************************
  
  //pinMode(ADC1_CS_pin, OUTPUT);
  //digitalWrite(ADC1_CS_pin, HIGH);
  //SPI.begin();
 




  //********************************************************************
  // Graph Setup
  //********************************************************************
  Serial.begin(115200);
  SPI.begin();//SCK, MISO, MOSI, CS
  delay(5000);

  pinMode(screen_CS, OUTPUT);
  digitalWrite(screen_CS, HIGH);
  tft.begin();
  
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  // Graph area is 200 pixels wide, 150 high, dark grey background
  gr.createGraph(graph_length, graph_height, tft.color565(5, 5, 5));
  // x scale units is from 0 to 100, y scale units is -50 to 50
  gr.setGraphScale(graph_scale_xmin, graph_scale_xmax, graph_scale_ymin, graph_scale_ymax);
  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at -50 with lines every 25 y-scale units
  // blue grid
  gr.setGraphGrid(graph_scale_xmin, graph_scale_xgrid, graph_scale_ymin, graph_scale_ygrid, TFT_BLUE);
  // Draw empty graph, top left corner at 40,10 on TFT
  gr.drawGraph(graph_top_corner_x, graph_top_corner_y);
  // Start a trace with using red and another with green
  //tr1.startTrace(TFT_RED);
  //tr2.startTrace(TFT_GREEN);
  // Add points on graph to trace 1 using graph scale factors
  //tr1.addPoint(0.0, 0.0);
  //tr1.addPoint(50.0, 0.0);
  // Add points on graph to trace 2 using graph scale factors
  // Points are off graph so the plotted line is clipped to graph area
  //tr2.addPoint(0.0, -50.0);
  //tr2.addPoint(50.0, 50.0);
  // Get x,y pixel coordinates of any scaled point on graph
  // and ring that point.
  //tft.drawCircle(gr.getPointX(50.0), gr.getPointY(0.0), 5, TFT_MAGENTA);


  // Draw the x axis scale
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(int(graph_scale_xmin), gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymin) + 3);
  tft.drawNumber(int(graph_scale_xmax - graph_scale_xmin), gr.getPointX(graph_scale_xmax - graph_scale_xmin), gr.getPointY(graph_scale_ymin) + 3);
  tft.drawNumber(int(graph_scale_xmax), gr.getPointX(graph_scale_xmax), gr.getPointY(graph_scale_ymin) + 3);


  // Draw the y axis scale
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  tft.drawNumber(int(graph_scale_ymin), gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymin));
  tft.drawNumber(int(graph_scale_ymax - graph_scale_ymin), gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymax - graph_scale_ymin));
  tft.drawNumber(int(graph_scale_ymax), gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymax));


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
  pinMode(signal1_toggleBttn, INPUT_PULLUP);// Using GPIO 6 for button testing
  pinMode(triggerBttn, INPUT_PULLUP);
  pinMode(ADC1_CS_pin, OUTPUT);
  pinMode(ADC1_EocInt, INPUT_PULLUP);
  pinMode(ADC1_CONVST_pin, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(26), trigger_set, FALLING); //was pause_button, now trigger_state
  attachInterrupt(digitalPinToInterrupt(signal1_toggleBttn), pause_button, FALLING);
  attachInterrupt(digitalPinToInterrupt(triggerBttn), trigger_set, FALLING);
  attachInterrupt(digitalPinToInterrupt(ADC1_EocInt), ADC_collect_data, FALLING);

  //Set ADC to default mode and start converting values


  ADC_Set_Default();
}


void loop() {
  startTFTComms();
 
  while(!pause_state){
    delayMicroseconds(10);
    tft.fillRect(150, 5, 100, 10, TFT_BLACK);
    tft.setCursor(150, 5);
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);


    if(!trigger_state){
      delayMicroseconds(1);
      tft.println(ADC_14bit_val);
      //tft.println("Trigger: Rising");
    }else{
      delayMicroseconds(1);
      //tft.println("Trigger: Falling");
      tft.println(ADC_14bit_val);
    }
  trigger_function();
  static uint32_t plotTime = micros(); // used to be = millis(), now using micros
  int Amplitude = (findMax(real_data) - findMin(real_data))/2;
  tft.setCursor(110, 5);
  tft.fillRect(110, 5, 10, 10, TFT_BLACK);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println(Amplitude);
  int y_zoom = analogRead(ampScale_wiper) + 1; // GPIO 34
  int x_zoom = analogRead(timeScale_wiper) + 1;
  //int val = -1;//analogRead(ADC_out_pin); //fix this when SPI gets updated
  static float gx = 0.0, gy = 0.0;
  //static float delta = 7.0;
  int SampleTime = 4095/(x_zoom);
  
  if ((x_zoom > analogRead(timeScale_wiper)+40) || (x_zoom < analogRead(timeScale_wiper)-40)){
    //Time (x) axis zoom
    tft.fillRect(0, gr.getPointY(graph_scale_ymin) + 3, 320, 10, TFT_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextDatum(TC_DATUM); // Top centre text datum
    tft.drawNumber(0, gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymin) + 3);
    tft.drawNumber((409.5/(.2*x_zoom)), gr.getPointX(graph_scale_xmax - graph_scale_xmin), gr.getPointY(graph_scale_ymin) + 3);
    tft.drawNumber((409.5/(.1*x_zoom)), gr.getPointX(graph_scale_xmax), gr.getPointY(graph_scale_ymin) + 3);
  }

  if ((y_zoom > analogRead(ampScale_wiper)+40) || (y_zoom < analogRead(ampScale_wiper)-40)){
    //voltage (y) axis zoom
    tft.fillRect(0, 0, 19, 220, TFT_BLACK);
    tft.setRotation(2);
    tft.setCursor(100, 5);
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
    tft.println("Voltage (V)");
    tft.setRotation(3);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextDatum(MR_DATUM); // Middle right text datum
    tft.drawNumber((((graph_scale_ymin)*y_zoom)/4095), gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymin));
    tft.drawNumber(0, gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymax - graph_scale_ymin));
    tft.drawNumber((((graph_scale_ymax)*y_zoom)/4095), gr.getPointX(graph_scale_xmin), gr.getPointY(graph_scale_ymax));

  }
 
  if (micros() - plotTime >= 1) { //edited from sampletime
    plotTime = micros();


    // Add a new point on each trace
    //tr1.addPoint(gx, gy);
    real_data[int(gx)] = gy;
    //Serial.println(real_data[int(gx)]);
    display_data[int(gx)] = real_data[int(gx)]*(4095/y_zoom);
    tr2.addPoint(gx, gy);
    //tr2.addPoint(gx, gy/2.0); // half y amplitude


    // Create next plot point
    gx += 1.0;
    //gy += delta;
    ADC_14bit_val = 0;
    ADC_14bit_val = (ADC1_valH<<8 | ADC1_valL)>>4;
    gy = ((((ADC_14bit_val*5)/16383)-2.5)*2) ;//+= delta;
    //if (gy >  70.0) { delta = -7.0; gy =  70.0; }
    //if (gy < -70.0) { delta =  7.0; gy = -70.0; }


    // If the end of the graph is reached start 2 new traces
    if (gx > 100) {
      gx = 0.0;
      gy = display_data[99];


      // Draw empty graph at 40,10 on display
      gr.drawGraph(graph_top_corner_x, graph_top_corner_y);
   
      // Start new trace
      //tr1.startTrace(TFT_GREEN);
      tr2.startTrace(TFT_YELLOW);
    }
   
  }
}
}    


//*****************************************************
// Functions
//*****************************************************


void pauseTFTComms(){
  digitalWrite(screen_CS, HIGH);
  tft.endWrite();
}


void startTFTComms(){
  tft.startWrite();
  digitalWrite(screen_CS, LOW);
}

void ADC_Set_Default(){
  pauseTFTComms();
  delayMicroseconds(1);
  //digitalWrite(screen_CS, HIGH);
  SPI.beginTransaction(SPISettings(spi_max_clk, MSBFIRST, SPI_MODE3));// Mode 3 means clock is idle at HIGH and output edge is falling. Max SPI clk frequency is 50MHz.
  digitalWrite(ADC1_CS_pin, LOW);
  delayMicroseconds(1);
  SPI.transfer(0b1111); //sets converter to default mode
  delayMicroseconds(1);
  digitalWrite(ADC1_CS_pin, HIGH);
  digitalWrite(ADC1_CONVST_pin, HIGH);
  SPI.endTransaction();
  //digitalWrite(screen_CS, LOW);
  startTFTComms();
  return;


}


void ADC_collect_data(){
  pauseTFTComms();
  delayMicroseconds(1);
  //digitalWrite(screen_CS, HIGH);
  SPI.beginTransaction(SPISettings(spi_max_clk, MSBFIRST, SPI_MODE3));// Mode 3 means clock is idle at HIGH and output edge is falling. Max SPI clk frequency is 50MHz.
  digitalWrite(ADC1_CS_pin, LOW);
  delayMicroseconds(1); //delay 1ms
  //do {
    //int i = micros();
  //}while(!ADC1_EocInt);
  //int x = digitalRead(SDO_pin); //communicate with chip to get value
  ADC1_valH = SPI.transfer(0);
  delayMicroseconds(14);
  ADC1_valL = SPI.transfer(0);
  delayMicroseconds(14);
  digitalWrite(ADC1_CS_pin, HIGH);
  digitalWrite(ADC1_CONVST_pin, HIGH);
  SPI.endTransaction();
  //digitalWrite(screen_CS, LOW);
  startTFTComms();
  return ;


}






void trigger_set(){
  trigger_state = !trigger_state;
  return;
}


void trigger_function(){
  bool trigger_met = 0;
  while(!trigger_met){
    static uint32_t plotTime_tgr = micros(); // used to be = millis(), now using micros
    for (int i = 0; i<6;i++){
      if (micros() - plotTime_tgr >= 1) { //edited from sampletime
        plotTime_tgr = micros();
        ADC_14bit_val = (ADC1_valH<<8 | ADC1_valL)>>4;
        int val_tgr = ((((ADC_14bit_val*5)/16383)-2.5)*2) ;
        int y_zoom_tgr = analogRead(ampScale_wiper) + 1;//analogRead(36) + 1; // GPIO 36
        trigger_data[i] = val_tgr;
    }
  
  if (trigger_state){                                         // rising edge trigger logic
      int i = 0;
      if (trigger_data[i]<=trigger_data[i+1]){
        if(trigger_data[i+1]<=trigger_data[i+2]){
          if(trigger_data[i+2]<=trigger_data[i+3]){
            if(trigger_data[i+3]<=trigger_data[i+4]){
                trigger_met=1;


            }


          }


        }
      }


    } else {
      int j = 0;
      if (trigger_data[j]>=trigger_data[j+1]){
        if(trigger_data[j+1]>=trigger_data[j+2]){
          if(trigger_data[j+2]>=trigger_data[j+3]){
            if(trigger_data[j+3]>=trigger_data[j+4]){
              trigger_met = 1;
              }


            }


          }


        }
      }


    }
  
  }
  
  return;

}

  

/*
  static uint32_t plotTime_tgr = micros(); // used to be = millis(), now using micros
  for (int i = 0; i<50;i++){
    if (micros() - plotTime_tgr >= 10) { //edited from sampletime
      plotTime_tgr = micros();
      int val_tgr = analogRead(35);
      int y_zoom_tgr = 4095;//analogRead(36) + 1; // GPIO 36
      static float gy_tgr = ((val_tgr/41)-50)*(4095/y_zoom_tgr);
      trigger_data[i] = gy_tgr;
    }  
  }
  if (trigger_state){                                         // rising edge trigger logic
    for (int i = 0; i<50;i++){
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
    for (int i = 0; i<50;i++){
      if (trigger_data[i]>=trigger_data[i+1]){
        if(trigger_data[i+1]>=trigger_data[i+2]){
          if(trigger_data[i+2]>=trigger_data[i+3]){
            if(trigger_data[i+3]>=trigger_data[i+4]){
              for (int j = i; j < 50; j++){
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
*/

void pause_button(){
  pause_state = !pause_state;
  return;
}






int findMax(float array[]){
  int max = array[0];
  for (int i=1; i < 100; i++){
    if (array[i] > max){
      max = array[i];
    }


  }
  return max;
}


int findMin(float array[]){
  int min = array[0];
  for (int i=1; i < 100; i++){
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





