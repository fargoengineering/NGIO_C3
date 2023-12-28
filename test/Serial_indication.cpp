#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10

void setup(){
    Serial.begin(9600);

    RGBled.begin();
    RGBled.setPixelColor(0,RGBled.Color(0,0,255));
    RGBled.show();
    delay(1000);
}

void loop(){
    if (Serial.available()){
        RGBled.setPixelColor(0,RGBled.Color(100,100,0));
        RGBled.show();
        delay(100);
    }

    RGBled.setPixelColor(0,RGBled.Color(0,0,0));
    RGBled.show();

    // int randomNum = random(0,1000);
    // Serial.println(randomNum);
    // delay(50);
}