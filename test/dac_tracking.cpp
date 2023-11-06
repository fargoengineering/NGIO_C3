#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Pin_config_C3.h>

Adafruit_MCP4725 dac;
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); 
int val = 1000;
int input = 0;
int data = 0;
int dac_boost = 0;
int tracking_flag = 1;
int serial_val = 0;
int previous_dac_val = 0;

void setup(){
    Serial.begin(115200);
    Wire.begin(SDA,SCL);
    dac.begin(0x60);
    
    pinMode(SLOT_IO0pin,INPUT);

    RGBled.setPixelColor(0,RGBled.Color(35,75,128));
    RGBled.setBrightness(128);
    RGBled.show();
}

void loop(){
    // this test on slot 8 (output board) does not cause any stacktrace/reboot error?
    // dac.setVoltage(val,false);
    // val+=100;

    if(val>4090){val=0;}

    delay(10);
    // Serial.printf("Val: %d\n",val);

    // input = analogRead(SLOT_IO0pin);
    // Serial.printf("INPUT: %d\n",input);

    ///////////////////////////////////

    // switch case tracking logic: 
    if (Serial.available()){
        val = Serial.parseInt();
        if(val == 1){tracking_flag = 1;}
        if(val == 2){tracking_flag = 0;}
    }

    if (tracking_flag==1){
      // Serial.print("DAC TRACKING: ");
      if (val != previous_dac_val){
        Serial.print("\n\nNEW VAL\n\n");
        previous_dac_val = val;
        dac_boost = 0;
      }
      int final_val = val+dac_boost;
      if(final_val < 0){final_val = 0;}else if(final_val > 4096){final_val = 4095;}
      Serial.printf("Setting DAC to: %d\n",final_val);
      dac.setVoltage(final_val, false);
      data = analogRead(SLOT_IO0pin); // pdo 1 and 2 MSB
      if (data < val){
        Serial.printf("Expected: %d, Actual: %d - RAISING OUTPUT\n",val,data);
        dac_boost++;
      }else if(data > val){
        Serial.printf("Expected: %d, Actual: %d - LOWERING OUTPUT\n",val,data);
        dac_boost--;
      }else{
        Serial.printf("Expected: %d, Actual: %d - OUTPUT ACHEIVED\n",val,data);
      }
    }
    else{
      // Serial.println("NORMAL DAC");
      if(val < 0){val = 0;}else if(val > 4096){val = 4096;}
      dac_boost = 0;
      dac.setVoltage(val,false);
      data = analogRead(SLOT_IO0pin);
      data = val;
    }
}