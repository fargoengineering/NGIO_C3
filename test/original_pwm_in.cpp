#include "PWM.hpp"
#include <Arduino.h>

PWM ch1(0); // Setup pin 2 for input

void setup() {
    Serial.begin(115200); // Serial for debug
    Serial.println("Setup");
    ch1.begin(true); // ch1 on pin 2 reading PWM HIGH duration
    // pinMode(0,INPUT);
}

void loop() {
    Serial.println(ch1.getValue());
    // Serial.println(digitalRead(0));
    delay(1000);
}