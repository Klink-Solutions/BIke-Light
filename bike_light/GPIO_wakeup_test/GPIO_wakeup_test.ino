#include "esp_sleep.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
// define pins
const int led = D10; 
const int hall = D3;

RTC_DATA_ATTR int bootCount = 0;// needs to be keept in RTC to survive sleep

//Accelerometer stuff
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);

const float ACCEL_THRESHOLD = 1.0;// threshold 
//Total accel value x
float readAccelMagnitude() {
  sensors_event_t event;
  accel.getEvent(&event);   // fills event with current x, y, z readings
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  return (x);
}

void setup() {
  Serial.begin(115200);
  delay(200);
 Serial.println("Alive");

  pinMode(led, OUTPUT);   // initialize digital pin led as an output
  pinMode(hall, INPUT_PULLUP);   // initialize digital pin hall as an imput


  //Increment boot number and print it every reboot
  bootCount++;
  Serial.println("Boot number: " + String(bootCount));

  //Accel setup
  if (!accel.begin()) {
    Serial.println("Error");
    while (1);
  }
  Serial.println("ADXL Ready");
  accel.setRange(ADXL345_RANGE_4_G);
 
 while (digitalRead(hall) == LOW) {
    float magnitude = readAccelMagnitude();
    Serial.print("Accel magnitude:");
    Serial.println(magnitude);
    Serial.print(" ");

    if (magnitude > ACCEL_THRESHOLD) {
      digitalWrite(led, HIGH);
      Serial.println("ON");
      delay(100);
      digitalWrite(led, LOW);
      Serial.println("OFF");
      delay(100);
       } 
       else {
      // No motion — LED off
        digitalWrite(led, LOW);
        Serial.println("No Breaking Detected");
       }

  }

  // Magnet was removed — wait 1 second then sleep
  Serial.println("Magnet removed -  Sleeping in 1 second");
  digitalWrite(led, LOW);  // Make sure LED is off
  delay(1000);

  // Setup the wakup trigger
  esp_deep_sleep_enable_gpio_wakeup(BIT(D3), ESP_GPIO_WAKEUP_GPIO_LOW);
  Serial.println("Going to sleep ");
  delay(200);  // Let Serial finish printing before sleep cuts power
  esp_deep_sleep_start();
}

void loop() { }//we kinda dont need this at all i guess
