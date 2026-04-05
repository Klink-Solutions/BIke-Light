#include "esp_sleep.h"
// define led according to pin diagram in article
const int led = D10; 
const int hall = D3;

RTC_DATA_ATTR int bootCount = 0;
void setup() {
  Serial.begin(115200);
  delay(200);
 Serial.println("Alive");

  pinMode(led, OUTPUT);   // initialize digital pin led as an output
  pinMode(hall, INPUT_PULLUP);   // initialize digital pin hall as an imput


  //Increment boot number and print it every reboot
  bootCount++;
  Serial.println("WOKE UP");
  Serial.println("Boot number: " + String(bootCount));

  while (digitalRead(hall) == LOW) {
    digitalWrite(led, HIGH);
    Serial.println("ON");
    delay(1000);
    digitalWrite(led, LOW);
    Serial.println("OFF");
    delay(1000);
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

void loop() {
   
}