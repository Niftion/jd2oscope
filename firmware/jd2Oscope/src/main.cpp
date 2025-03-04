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
//#include "font_Arial.h"
//#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library
// #include <Adafruit_GFX.h>       // include Adafruit graphics library
// #include <Adafruit_BusIO_Register.h>


/*
#include <implot.h>
#include <implot_internal.h>
#include <implot.cpp>
#include <implot_items.cpp>
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
#include <math.h>
#include <time.h>
#include <driver/gpio.h>
*/

//char receivedChar;
//char receivedChars[32];
//char endMarker = '.';
//int j =0;
//bool eor = false;
//static void big_timer_callback(void* arg);
//static void small_timer_callback(void* arg);
//int ADC_val_converter(int example_input);
//static double volts_to_bits(double volt_in);
//static double bits_to_volts(double bit_in);
//int button = 23; // io 0 -> leads to interrupt reset? trying io 23 now
//bool Ch1 = 1;
//bool Ch2 = 0;

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


//Setup for Display screen

// initialize ILI9341 TFT library
//TFT_eSPI tft = TFT_eSPI();

// put function declarations here:

void setup() {
  //pinMode(button,INPUT);
  //pinMode(33, OUTPUT); // use IO21 as output
  
  // put your setup code here, to run once:

  //Serial.println("ILI9341 Test!"); 
 
  //tft.begin();
  // Note: you can now set the SPI speed to any value
  // the default value is 30Mhz, but most ILI9341 displays
  // can handle at least 60Mhz and as much as 100Mhz
  //  tft.setClock(60000000);
    //tft.fillScreen(ILI9341_BLACK);
    //tft.setTextColor(ILI9341_YELLOW);
    //tft.setTextSize(2);
    //tft.println("Waiting for Arduino Serial Monitor...");

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
  
 
  //Serial.println("Enter t to test timer, c to test ADC conversions, or g to test graph");

  

}

void loop() {
  static uint32_t plotTime = millis();
  int Zoom = analogRead(36) + 1; // GPIO 36
  static float gx = 0.0, gy = 0.0;
  static float delta = 7.0;
  int SampleTime = 4095/(100*Zoom);
  tft.fillRect(0, gr.getPointY(-50.0) + 3, 320, 10, TFT_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(-50.0) + 3);
  tft.drawNumber((409.5/(.2*Zoom)), gr.getPointX(50.0), gr.getPointY(-50.0) + 3);
  tft.drawNumber((409.5/(.1*Zoom)), gr.getPointX(100.0), gr.getPointY(-50.0) + 3);

  //int Zoom = analogRead(36); // GPIO 36
  // x scale units is from 0 to 100, y scale units is -50 to 50
  //gr.setGraphScale(0.0, (Zoom/40950), -50.0, 50.0);

  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at -50 with lines every 25 y-scale units
  // blue grid
  //gr.setGraphGrid(0.0, (Zoom/409500), -50.0, 10.0, TFT_BLUE);

  //tft.setTextDatum(TC_DATUM); // Top centre text datum
  //tft.drawNumber(0, gr.getPointX(0.0), gr.getPointY(-50.0) + 3);
  //tft.drawNumber((Zoom/(2*40950)), gr.getPointX((Zoom/(2*40950))), gr.getPointY(-50.0) + 3);
  //tft.drawNumber((Zoom/40950), gr.getPointX((Zoom/40950)), gr.getPointY(-50.0) + 3);
  //gr.drawGraph(20, 20);
  // Sample periodically
  if (millis() - plotTime >= SampleTime) {
    plotTime = millis();

    // Add a new point on each trace
    tr1.addPoint(gx, gy);
    tr2.addPoint(gx, gy/2.0); // half y amplitude

    // Create next plot point
    gx += 1.0;
    gy += delta;
    //gy = (val/41)-50 ;//+= delta;
    if (gy >  70.0) { delta = -7.0; gy =  70.0; }
    if (gy < -70.0) { delta =  7.0; gy = -70.0; }

    // If the end of the graph is reached start 2 new traces
    if (gx > 100) {
      gx = 0.0;
      gy = 0.0;

      // Draw empty graph at 40,10 on display
      gr.drawGraph(20, 20);
      // Start new trace
      tr1.startTrace(TFT_GREEN);
      tr2.startTrace(TFT_YELLOW);
    }
  }
  
  // put your main code here, to run repeatedly:
  //Display code:
  //-------Display demo code-------
  //for(uint8_t rotation=0; rotation<4; rotation++) {
    //tft.setRotation(rotation);
    //testText();
    //testFillScreen();
    //testLines(ILI9341_GREEN);
    //delay(500); // 1000 is 1 sec
  //}
  //--------------------------------

  // +++++++++++  This Code below was for Block 1 and 2 Verification. Contents include:
  // +++++++++++  Push button and Potentiometer response
  // +++++++++++  ESPTimer code implementation
  // +++++++++++  Accepting input from the serial monitor.
  // +++++++++++  Pin assignments are oudated.
  /*
  if (Serial.available() >0){
    receivedChar = Serial.read();
  }

  if (receivedChar == 't'){
    //ceivedChar = 'x';
    const esp_timer_create_args_t small_timer_args={
    .callback = small_timer_callback, .name = "small"};

    esp_timer_handle_t small_timer;
    ESP_ERROR_CHECK(esp_timer_create(&small_timer_args, &small_timer));
    // The timer has been created but is not running yet  

    const esp_timer_create_args_t big_timer_args={
    .callback = big_timer_callback, .arg = (void*) small_timer , .name = "big"};

    esp_timer_handle_t big_timer;
    ESP_ERROR_CHECK(esp_timer_create(&big_timer_args, &big_timer));
    // Start the timers 
    Serial.println();
    ESP_ERROR_CHECK(esp_timer_start_periodic(small_timer, 500000));// 500000 -> 0.5s, 0.01s ->10000, 0.000001s -> 1
    ESP_ERROR_CHECK(esp_timer_start_once(big_timer, 5000000)); // 5000000 -> 5s, 1s-> 1000000, 50us->50
    long int x = esp_timer_get_time();
    ///Serial.print("time since boot: ");
    //Serial.print(x);
    //ESP_LOGI("Started timers, time since boot: %lld us", esp_timer_get_time(););

    // Print debugging information about timers to console every 2 seconds 
    for (int i = 0; i < 5; ++i) {
        ESP_ERROR_CHECK(esp_timer_dump(stdout));
        usleep(2000000);
    }
  // Clean up and finish the example 
    ESP_ERROR_CHECK(esp_timer_stop(small_timer));
    ESP_ERROR_CHECK(esp_timer_delete(small_timer));
    ESP_ERROR_CHECK(esp_timer_delete(big_timer));
    //Serial.println("Stopped and deleted timers");
    //ESP_LOGI(TAG, "Stopped and deleted timers");
    receivedChar = 'x';
    
  } else if (receivedChar == 'c'){
    int example_in =volts_to_bits(2.5);
    double example_out = bits_to_volts(example_in);
    
    Serial.println(volts_to_bits(3));
    Serial.println(bits_to_volts(0b11111111111111));
    receivedChar = 'x';

  } else if (receivedChar == 'b'){
    const esp_timer_create_args_t small_timer_args={
    .callback = small_timer_callback, .name = "small"};

    esp_timer_handle_t small_timer;
    ESP_ERROR_CHECK(esp_timer_create(&small_timer_args, &small_timer));
    // The timer has been created but is not running yet 

    const esp_timer_create_args_t big_timer_args={
    .callback = big_timer_callback, .arg = (void*) small_timer , .name = "big"};

    esp_timer_handle_t big_timer;
    ESP_ERROR_CHECK(esp_timer_create(&big_timer_args, &big_timer));
    // Start the timers 
    Serial.println();
    //ESP_ERROR_CHECK(esp_timer_start_periodic(small_timer, 500000));// 500000 -> 0.5s, 0.01s ->10000, 0.000001s -> 1
    //ESP_ERROR_CHECK(esp_timer_start_once(big_timer, 5000000)); // 5000000 -> 5s, 1s-> 1000000, 50us->50
    //long int x = esp_timer_get_time();
    //Serial.print("time since boot: ");
    //Serial.print(x);
    //ESP_LOGI("Started timers, time since boot: %lld us", esp_timer_get_time(););

    //Print debugging information about timers to console every 2 seconds 
    //for (int i = 0; i < 5; ++i) {
        //ESP_ERROR_CHECK(esp_timer_dump(stdout));
        //usleep(2000000);
    
  // Clean up and finish the example 
    //ESP_ERROR_CHECK(esp_timer_stop(small_timer));
    ////ESP_ERROR_CHECK(esp_timer_delete(small_timer));
    //ESP_ERROR_CHECK(esp_timer_delete(big_timer));
    //Serial.println("Stopped and deleted timers");
    //ESP_LOGI(TAG, "Stopped and deleted timers");
      //digitalWrite(33, HIGH);
      if (digitalRead(button)){
          Serial.println("Pressed ");
          Serial.println(esp_timer_get_time());
      } else {
        Serial.println("UnPressed");
        Serial.println(esp_timer_get_time());
      }
  }else if (receivedChar == 'p'){
      int potVal = analogRead(36); // GPIO 36
      Serial.println(potVal);
      Serial.println(esp_timer_get_time());
      Serial.println("");



  }
      //digitalWrite(33, HIGH);
      //if (Ch1){
        //Serial.println("Ch1");
        //}else{
         // Serial.println("Ch2");
         // }

      //if (digitalRead(button) == 1){
        //if (Ch1){
          //Ch1 = 0;
          //Ch2 = 1;
          //Serial.println("Ch2");
        //} else {
          //Ch2 = 0;
          //Ch1 = 1;
          //Serial.println("Ch1");
        //}

        //+++++++++ End of Block code, not including functions below
        */
}
    

//Functions for Display Testing

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
   else if (receivedChar == 'g'){

    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();


  }*/


  
  
//}

// put function definitions here:
/*
static void small_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    Serial.println();
    Serial.print("Small timer called, time since boot: ");
    Serial.print(time_since_boot);
    //ESP_LOGI(TAG, "Small timer called, time since boot: %lld us", time_since_boot);
}*/


/*
static void big_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    Serial.println();
    Serial.print("Big timer called, time since boot: ");
    Serial.print(time_since_boot);
    //ESP_LOGI(TAG, "Big timer called, time since boot: %lld us", time_since_boot);
    esp_timer_handle_t small_timer_handle = (esp_timer_handle_t) arg;
    // To start the timer which is running, need to stop it first 
    ESP_ERROR_CHECK(esp_timer_stop(small_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(small_timer_handle, 1000000));
    time_since_boot = esp_timer_get_time();
    Serial.println();
    Serial.print("Restarted small timer with 1s period, time since boot: ");
    Serial.print(time_since_boot);
    //ESP_LOGI(TAG, "Restarted small timer with 1s period, time since boot: %lld us", time_since_boot);
}

*/





/*
static double bits_to_volts(double bit_in){
  int positive_x_axis = 8191;
  int v_zoom = 1;
  double volts = (bit_in - positive_x_axis)*(5)/(positive_x_axis*v_zoom);
  return volts;
}
*/

/*
static double volts_to_bits(double volt_in){
  int positive_x_axis = 8191;
  int v_zoom = 1;
  double bits = positive_x_axis + (volt_in*positive_x_axis*v_zoom)/5;
  return round(bits);
}
  */

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

/* esp_timer (high resolution timer) example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
