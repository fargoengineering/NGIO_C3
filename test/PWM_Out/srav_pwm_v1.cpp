// TEST: PWM input based on analogRead() - 10-26-2023
#include <Arduino.h>

int pwmPin = 0;

unsigned long high1 = 0;
unsigned long high2 = 0;
unsigned long low = 0;
unsigned long highdiff = 0;

// unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
int printFreqms = 1000; // 1 second.

// float dutyCycle = 0.0;
// float frequency = 0.0;
unsigned int dutyCycle = 0;
unsigned int frequency = 0;

int analogval = 0;
int analoglimit = 800;
bool ishigh = false;

void setup()
{
}

void loop()
{
    analogval = analogRead(pwmPin);
    ishigh = (analogval > analoglimit);

    if (ishigh)
    {
        high2 = micros();
        if (high1 > 0 and low > 0)
        {
            highdiff = high2 - high1;
            frequency = (1000000 / highdiff); // Converging Micro seconds to frequency per second.
            dutyCycle = ((low - high1) * 100) / highdiff;
        }
        low = 0;
        high1 = high2;
    }
    // low
    else
    {
        low = micros();
    }
    currentMillis = millis();
    if (currentMillis - prevMillis >= printFreqms)
    {
        prevMillis = currentMillis;
        data = frequency;
        data2 = dutyCycle;
    }
}