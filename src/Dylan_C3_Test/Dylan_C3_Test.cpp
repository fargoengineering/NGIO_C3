
#include <Arduino.h>

int pwmPin = 0; 

unsigned long high1 = 0;
unsigned long high2 = 0;
unsigned long low = 0;
unsigned long highdiff = 0;
int loop_count = 0;

unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
int printFreqms = 1000; // 1 second. 

// float dutyCycle = 0.0;
// float frequency = 0.0;

unsigned int dutyCycle = 0;
unsigned int frequency = 0;

int analogval = 0;
int analoglimit = 1100;
bool ishigh = false;

void setup() {
    Serial.begin(115200); // Serial for debug
    Serial.println("Seting up");
    
    pinMode(pwmPin,INPUT);

    Serial.println("Set up done.");
}

void loop() {
    analogval = analogRead(pwmPin);

    ishigh = (analogval > analoglimit);

    if(ishigh)
    {
        high2 = micros();
        if(high1 > 0 and low > 0)
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

    if(currentMillis - prevMillis >= printFreqms)
    {
        Serial.print("Freq: ");
        Serial.println(frequency);
        Serial.print("DC: ");
        Serial.println(dutyCycle);
        Serial.print("Analog Value: ");
        Serial.println(analogval);
        prevMillis = currentMillis; 
        // Serial.print("Loop count: ");
        // Serial.println(loop_count);
        // loop_count = 0;
    }
    // Serial.println(analogval);
    // loop_count++;
}