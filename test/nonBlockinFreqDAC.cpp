/**************************************************************************/
/*!
    @file     sinewave.pde
    @author   Adafruit Industries
    @license  BSD (see license.txt)

    This example will generate a sine wave with the MCP4725 DAC.

    This is an example sketch for the Adafruit MCP4725 breakout board
    ----> http://www.adafruit.com/products/935

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
*/
/**************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

// Set this value to 9, 8, 7, 6 or 5 to adjust the resolution
#define DAC_RESOLUTION (12)

void togglePinNonBlocking(unsigned long frequency, unsigned int dutyCycle) {
    static unsigned long lastToggleTime = 0;  // Keeps track of the last toggle time
    static bool pinState = LOW;               // Current state of the pin

    // Calculate high and low durations based on frequency and duty cycle
    unsigned long period = 1000000UL / frequency;             // Period in microseconds
    unsigned long highTime = period * dutyCycle / 100;        // High duration
    unsigned long lowTime = period - highTime;                // Low duration

    unsigned long currentTime = micros();  // Current time in microseconds

    // Check if it's time to toggle the pin
    if ((pinState == HIGH && currentTime - lastToggleTime >= highTime) ||
        (pinState == LOW && currentTime - lastToggleTime >= lowTime)) {
        pinState = !pinState;               // Toggle the state
        // digitalWrite(pin, pinState);        // Update the pin state
        if(pinState == HIGH){
          dac.setVoltage(4095,false);
        }else{
          dac.setVoltage(0,false);
        }
        lastToggleTime = currentTime;       // Update the last toggle time
    }
}


void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello!");

  Wire.begin(8,7);
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x60);

  Serial.println("Generating a sine wave");
}

unsigned long frequency = 500;
unsigned long duty = 20;
void loop(void) {
  togglePinNonBlocking(3000,20);
  // dac.setVoltage(4095,false);
  // dac.setVoltage(0,false);
}