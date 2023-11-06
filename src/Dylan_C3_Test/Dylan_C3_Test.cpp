#include <Arduino.h>
#include "wiring_private.h"
#include "pins_arduino.h"
#include <hal/cpu_hal.h>
#include "driver/gpio.h"

int pwmChannel = 0;   // Selects channel 0
int frequence = 1000; // PWM frequency of 1 KHz
int resolution = 8;   // 8-bit resolution, 256 possible values
int pwmPin = 0;

int pwmPinInput = 0;
int threashold = 1000;

int dutyCycle = 0;
int frequenceInp = 0;
unsigned long high = 0;
unsigned long low = 0;
unsigned long microsecForsec = 1000000;

unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
int printFreqms = 3000; // 1 second.

// #define GET_DIGITAL_STATE_FROM_ANALOG_FEI(pin, limit) \
//     return analogRead(pin) >= limit; \

bool get_digital_from_analog(int pin, uint16_t limit)
{
    if (analogRead(pin) >= limit)
    {
        return true;
    }
    return false;
}

#define WAIT_FOR_PIN_STATE_FEI(state, pin, limit)                           \
    while (get_digital_from_analog(pin, limit) != (state))                  \
    {                                                                       \
        if (cpu_hal_get_cycle_count() - start_cycle_count > timeout_cycles) \
        {                                                                   \
            return 0;                                                       \
        }                                                                   \
    }

// max timeout is 27 seconds at 160MHz clock and 54 seconds at 80MHz clock
unsigned long pulseIn_FEI(uint8_t pin, uint8_t state, unsigned long timeout, int analogLimit)
{
    const uint32_t max_timeout_us = clockCyclesToMicroseconds(UINT_MAX);
    if (timeout > max_timeout_us)
    {
        timeout = max_timeout_us;
    }
    const uint32_t timeout_cycles = microsecondsToClockCycles(timeout);
    const uint32_t start_cycle_count = cpu_hal_get_cycle_count();
    WAIT_FOR_PIN_STATE_FEI(!state, pin, analogLimit);
    WAIT_FOR_PIN_STATE_FEI(state, pin, analogLimit);
    const uint32_t pulse_start_cycle_count = cpu_hal_get_cycle_count();
    WAIT_FOR_PIN_STATE_FEI(!state, pin, analogLimit);
    return clockCyclesToMicroseconds(cpu_hal_get_cycle_count() - pulse_start_cycle_count);
}

void GetPWMDetails_FEI(byte pin)
{
    // Let this function be included in the C3 Sketch
      unsigned long highTime = pulseIn_FEI(pin, HIGH, 50000UL, threashold);  // 50 millisecond timeout
      unsigned long lowTime = pulseIn_FEI(pin, LOW, 50000UL, threashold);  // 50 millisecond timeout

    // unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 millisecond timeout
    // unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout
    // pulseIn() returns zero on timeout
    if (highTime == 0 || lowTime == 0)
        dutyCycle = get_digital_from_analog(pin,threashold) ? 100 : 0; // HIGH == 100%,  LOW = 0%
      

    dutyCycle = (100 * highTime) / (highTime + lowTime); // highTime as percentage of total cycle time
    frequenceInp = (1 * microsecForsec) / (highTime + lowTime);
    high = highTime;
    low = lowTime;
}

void GetPWMDetails(byte pin)
{
    // Let this function be included in the C3 Sketch
    //   unsigned long highTime = pulseIn_FEI(pin, HIGH, 50000UL, threashold);  // 50 millisecond timeout
    //   unsigned long lowTime = pulseIn_FEI(pin, LOW, 50000UL, threashold);  // 50 millisecond timeout

    unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 millisecond timeout
    unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout
    // pulseIn() returns zero on timeout
    if (highTime == 0 || lowTime == 0)
        dutyCycle = digitalRead(pin) ? 100 : 0; // HIGH == 100%,  LOW = 0%
      

    dutyCycle = (100 * highTime) / (highTime + lowTime); // highTime as percentage of total cycle time
    frequenceInp = (1 * microsecForsec) / (highTime + lowTime);
    high = highTime;
    low = lowTime;
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("Initializing ...");

    // Configuration of channel 0 with the chosen frequency and resolution
    // ledcSetup(pwmChannel, frequence, resolution);

    // // Assigns the PWM channel to pin 23
    // ledcAttachPin(pwmPin, pwmChannel);

    // // Create the selected output voltage
    // ledcWrite(pwmChannel, 100); // 1.65 V ~ 127
    // pinMode(1,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(pwmPinInput, INPUT);

    digitalWrite(19,LOW);
}

void loop()
{
    GetPWMDetails_FEI(pwmPinInput);
    // digitalWrite(1,!digitalRead(1));
    int val = analogRead(pwmPinInput);
    // int val = digitalRead(pwmPinInput);
    // Serial.println(val);

    currentMillis = millis();

    // if (currentMillis - prevMillis >= printFreqms)
    // {
    // Serial.print("Analog In value: ");
    Serial.print("Digital In Value: ");
    Serial.print(val);        
    Serial.print(" Freq: ");
    Serial.print(frequenceInp);
    Serial.print(" DC: ");
    Serial.print(dutyCycle);
    Serial.print(" High: ");
    Serial.print(high);
    Serial.print(" Low: ");
    Serial.println(low);

    prevMillis = currentMillis;
    // }
}