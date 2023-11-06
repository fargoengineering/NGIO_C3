#include <Arduino.h>
#include "wiring_private.h"
#include "pins_arduino.h"
#include <hal/cpu_hal.h>
#include <driver/adc.h>

// int _pwmChannel = 0; // Selects channel 0
// int _frequence = 1000; // PWM frequency of 1 KHz
// int _resolution = 8; // 8-bit resolution, 256 possible values
// int _pwmPin = 0;

// int _pwmPinInput = 0;
// int _threashold = 500;

// int _dutyCycle = 0;
// int _frequenceInp = 0;
// unsigned long _high = 0;
// unsigned long _low = 0;
// unsigned long _microsecForsec = 1000000;

// unsigned long _currentMillis = 0;
// unsigned long _prevMillis = 0;
// int _printFreqms = 3000; // 1 second. 


bool get_digital_from_analog(int pin, uint16_t limit)
{
    if(adc1_get_raw(ADC1_CHANNEL_0) >= limit)
    {
        return true;
    }
    return false;
}

#define WAIT_FOR_PIN_STATE_FEI(state, pin, limit) \
    while (get_digital_from_analog(pin, limit) != (state)) { \
        if (cpu_hal_get_cycle_count() - start_cycle_count > timeout_cycles) { \
            return 0; \
        } \
    }

// max timeout is 27 seconds at 160MHz clock and 54 seconds at 80MHz clock
unsigned long pulseIn_FEI(uint8_t pin, uint8_t state, unsigned long timeout, int analogLimit)
{
    const uint32_t max_timeout_us = clockCyclesToMicroseconds(UINT_MAX);
    if (timeout > max_timeout_us) {
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