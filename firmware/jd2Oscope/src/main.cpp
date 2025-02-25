#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <esp_timer.h>
#include <esp_err.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
/*
#include <implot.h>
#include <implot_internal.h>
#include <implot.cpp>
#include <implot_items.cpp>
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
//nclude <math.h>
//nclude <time.h>
//#include <driver/gpio.h>
*/
char receivedChar;
char receivedChars[32];
char endMarker = '.';
int j =0;
bool eor = false;
static void big_timer_callback(void* arg);
static void small_timer_callback(void* arg);
int ADC_val_converter(int example_input);
static double volts_to_bits(double volt_in);
static double bits_to_volts(double bit_in);
int button = 0; // io 0
bool Ch1 = 1;
bool Ch2 = 0;
// put function declarations here:

void setup() {
  pinMode(button,INPUT);
  pinMode(33, OUTPUT); // use IO21 as output
  Serial.begin(115200);
  // put your setup code here, to run once:

  
  Serial.println("Enter t to test timer, c to test ADC conversions, or g to test graph");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() >0){
    receivedChar = Serial.read();
  }

  if (receivedChar == 't'){
    //ceivedChar = 'x';
    const esp_timer_create_args_t small_timer_args={
    .callback = small_timer_callback, .name = "small"};

    esp_timer_handle_t small_timer;
    ESP_ERROR_CHECK(esp_timer_create(&small_timer_args, &small_timer));
    /* The timer has been created but is not running yet */ 

    const esp_timer_create_args_t big_timer_args={
    .callback = big_timer_callback, .arg = (void*) small_timer , .name = "big"};

    esp_timer_handle_t big_timer;
    ESP_ERROR_CHECK(esp_timer_create(&big_timer_args, &big_timer));
    /* Start the timers */
    Serial.println();
    ESP_ERROR_CHECK(esp_timer_start_periodic(small_timer, 500000));// 500000 -> 0.5s, 0.01s ->10000, 0.000001s -> 1
    ESP_ERROR_CHECK(esp_timer_start_once(big_timer, 5000000)); // 5000000 -> 5s, 1s-> 1000000, 50us->50
    long int x = esp_timer_get_time();
    ///Serial.print("time since boot: ");
    //Serial.print(x);
    //ESP_LOGI("Started timers, time since boot: %lld us", esp_timer_get_time(););

    /* Print debugging information about timers to console every 2 seconds */
    for (int i = 0; i < 5; ++i) {
        ESP_ERROR_CHECK(esp_timer_dump(stdout));
        usleep(2000000);
    }
  /* Clean up and finish the example */
    ESP_ERROR_CHECK(esp_timer_stop(small_timer));
    ESP_ERROR_CHECK(esp_timer_delete(small_timer));
    ESP_ERROR_CHECK(esp_timer_delete(big_timer));
    //Serial.println("Stopped and deleted timers");
    //ESP_LOGI(TAG, "Stopped and deleted timers");
    receivedChar = 'x';
    
  } else if (receivedChar == 'c'){
    int example_in =volts_to_bits(2.5);
    double example_out = bits_to_volts(example_in);
    //Serial.println(volts_to_bits(1.25));
    //Serial.println(example_out);
    //Serial.println(bits_to_volts(example_in));
    //Serial.println(volts_to_bits(example_out));
    Serial.println(volts_to_bits(3));
    Serial.println(bits_to_volts(0b11111111111111));
    receivedChar = 'x';

  } else if (receivedChar == 'b'){
    const esp_timer_create_args_t small_timer_args={
    .callback = small_timer_callback, .name = "small"};

    esp_timer_handle_t small_timer;
    ESP_ERROR_CHECK(esp_timer_create(&small_timer_args, &small_timer));
    /* The timer has been created but is not running yet */ 

    const esp_timer_create_args_t big_timer_args={
    .callback = big_timer_callback, .arg = (void*) small_timer , .name = "big"};

    esp_timer_handle_t big_timer;
    ESP_ERROR_CHECK(esp_timer_create(&big_timer_args, &big_timer));
    /* Start the timers */
    Serial.println();
    //ESP_ERROR_CHECK(esp_timer_start_periodic(small_timer, 500000));// 500000 -> 0.5s, 0.01s ->10000, 0.000001s -> 1
    //ESP_ERROR_CHECK(esp_timer_start_once(big_timer, 5000000)); // 5000000 -> 5s, 1s-> 1000000, 50us->50
    //long int x = esp_timer_get_time();
    //Serial.print("time since boot: ");
    //Serial.print(x);
    //ESP_LOGI("Started timers, time since boot: %lld us", esp_timer_get_time(););

    /* Print debugging information about timers to console every 2 seconds */
    //for (int i = 0; i < 5; ++i) {
        //ESP_ERROR_CHECK(esp_timer_dump(stdout));
        //usleep(2000000);
    }
  /* Clean up and finish the example */
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
      
      /*digitalWrite(33, HIGH);
      if (Ch1){
        Serial.println("Ch1");
        }else{
          Serial.println("Ch2");
          }

      if (digitalRead(button) == 1){
        if (Ch1){
          Ch1 = 0;
          Ch2 = 1;
          Serial.println("Ch2");
        } else {
          Ch2 = 0;
          Ch1 = 1;
          Serial.println("Ch1");
        }*/
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

static void small_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    Serial.println();
    Serial.print("Small timer called, time since boot: ");
    Serial.print(time_since_boot);
    //ESP_LOGI(TAG, "Small timer called, time since boot: %lld us", time_since_boot);
}

static void big_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    Serial.println();
    Serial.print("Big timer called, time since boot: ");
    Serial.print(time_since_boot);
    //ESP_LOGI(TAG, "Big timer called, time since boot: %lld us", time_since_boot);
    esp_timer_handle_t small_timer_handle = (esp_timer_handle_t) arg;
    /* To start the timer which is running, need to stop it first */
    ESP_ERROR_CHECK(esp_timer_stop(small_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(small_timer_handle, 1000000));
    time_since_boot = esp_timer_get_time();
    Serial.println();
    Serial.print("Restarted small timer with 1s period, time since boot: ");
    Serial.print(time_since_boot);
    //ESP_LOGI(TAG, "Restarted small timer with 1s period, time since boot: %lld us", time_since_boot);
}








static double bits_to_volts(double bit_in){
  int positive_x_axis = 8191;
  int v_zoom = 1;
  double volts = (bit_in - positive_x_axis)*(5)/(positive_x_axis*v_zoom);
  return volts;
}

static double volts_to_bits(double volt_in){
  int positive_x_axis = 8191;
  int v_zoom = 1;
  double bits = positive_x_axis + (volt_in*positive_x_axis*v_zoom)/5;
  return round(bits);
}

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


/* esp_timer (high resolution timer) example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
