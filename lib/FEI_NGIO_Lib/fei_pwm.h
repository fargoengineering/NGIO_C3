#pragma once

#include <Arduino.h>
// #include <fei_pwm.cpp>

// Function to read a digital value from an analog pin
bool get_digital_from_analog(int pin, uint16_t limit);

// Function to wait for a pin to reach a given state
#define WAIT_FOR_PIN_STATE_FEI(state, pin, limit) \
  while (get_digital_from_analog(pin, limit) != (state)) { \
    if (cpu_hal_get_cycle_count() - start_cycle_count > timeout_cycles) { \
      return 0; \
    } \
  }

// Function to measure the pulse width of a PWM signal in microseconds
unsigned long pulseIn_FEI(uint8_t pin, uint8_t state, unsigned long timeout, int analogLimit);
