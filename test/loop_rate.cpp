#include <Arduino.h>

unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
int printFreqms = 1000; // 1 second. 
int pwmPinInput = 0;
int loopCount = 0;

void setup(){
    Serial.begin(115200);
    pinMode(1,OUTPUT);
    pinMode(pwmPinInput, INPUT);
}

void loop(){
    digitalWrite(1,!digitalRead(1));
    int val = analogRead(pwmPinInput);
    loopCount++;

    currentMillis = millis();
    if(currentMillis - prevMillis >= printFreqms)
    {
        Serial.print("Analog In value: ");
        Serial.println(val);
        Serial.print("Loop per second: ");
        Serial.println(loopCount);
        loopCount = 0;
        prevMillis = currentMillis;
    }
}