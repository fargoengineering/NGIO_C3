#include <Arduino.h>

// Define the loop rate
const int LOOP_RATE = 100000; // 100us

// Function to calculate the PWM input
void calculatePWMInput(int &highCounts, int &lowCounts)
{
    // Initialize the counters
    highCounts = 0;
    lowCounts = 0;

    // Set the high flag to false
    bool highFlag = false;

    // Loop until we see two highs
    while (!highFlag || highCounts < 2)
    {
        // Read the analog input value
        int analogInputValue = analogRead(0);

        // If the analog input value is greater than 1000, set the high flag
        if (analogInputValue > 1000)
        {
            highFlag = true;
        }

        // Increment the high counter
        highCounts++;

        // Wait for the loop rate
        delayMicroseconds(LOOP_RATE);
    }

    // Reset the high flag
    highFlag = false;

    // Loop until we see a low
    while (!highFlag)
    {
        // Read the analog input value
        int analogInputValue = analogRead(0);

        // If the analog input value is less than 1000, set the high flag
        if (analogInputValue < 1000)
        {
            highFlag = true;
        }

        // Increment the low counter
        lowCounts++;

        // Wait for the loop rate
        delayMicroseconds(LOOP_RATE);
    }
}

// Setup function
void setup()
{
    pinMode(0,INPUT);
    // Start the serial port
    Serial.begin(9600);
}

// Loop function
void loop()
{
    // Calculate the PWM input
    int highCounts, lowCounts;
    calculatePWMInput(highCounts, lowCounts);

    // Calculate the cycle time
    int cycleTime = highCounts + lowCounts;

    // Calculate the on time percentage
    float onTimePercentage = (float)highCounts / (float)cycleTime * 100.0f;

    // Calculate the off time percentage
    float offTimePercentage = (float)lowCounts / (float)cycleTime * 100.0f;

    // Calculate the frequency
    float frequency = (float)LOOP_RATE / (float)cycleTime;

    // Print the results to the serial port
    Serial.print("High counts: ");
    Serial.println(highCounts);
    Serial.print("Low counts: ");
    Serial.println(lowCounts);
    Serial.print("Cycle time: ");
    Serial.println(cycleTime);
    Serial.print("On time percentage: ");
    Serial.println(onTimePercentage);
    Serial.print("Off time percentage: ");
    Serial.println(offTimePercentage);
    Serial.print("Frequency: ");
    Serial.println(frequency);

    // Delay for 1 second
    delay(1000);
}
