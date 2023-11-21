#include <ESP32SPISlave.h>
#include <Adafruit_BusIO_Register.h>
#include <atomic>
#include <Adafruit_NeoPixel.h>
#include <CRC.h>
#include <Pin_config_C3.h>
#include <Reset_Reason_C3.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include <PWM.hpp>
#include <ble_ota.h>
#include <WiFi.h>
#include <fei_pwm.h>
#include <driver/adc.h>

int loopCount = 0;

const char VERSION[] = "V4_C3";
const char *filename = "/config.txt";
ESP32SPISlave slave;
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10
byte newLEDdata[3];
boolean newDataAvail = false;
bool ble_enabled = false;
bool slot_type_error = false;
bool first_run = false;
bool blink = false;
static constexpr uint32_t BUFFER_SIZE{32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b

// Global atomic variables SLOT TYPE and DATA OUT
std::atomic<int> data_out_atom(0);    // Data OUT
std::atomic<int> data2_out_atom(0);   // Data OUT
std::atomic<int> slot_type_atom(0);   // Slot Type Byte (Aux)
std::atomic<int> ble_state_atom(0);   // BLE State Byte (Aux)
std::atomic<int> pdo1(0);             // PDO 1 Output
std::atomic<int> pdo2(0);             // PDO 2 Output
std::atomic<int> pdo3(0);             // PDO 3 Output
std::atomic<int> pdo4(0);             // PDO 4 Output
std::atomic<int> relay_state_atom(0); // Holds state of the relays
std::atomic<bool> first_run_atom(0);  // Marks run setup flag
std::atomic<int> HW_Version_atom(0);  // Hardware Version
std::atomic<int> SW_Version_atom(0);  // Software Version

CRC crc;
PWM my_pwm(SLOT_IO0pin);

const int pwmResolution = 10;
int freq_value = 0;
int tracking_flag = 0;
int freq_value_last = 0;
int pwm_value = 0;
int pwm_value_last = 0;
bool low_speed_freq = false;
int pwmState = LOW;
int freq = 50;
const int ledChannel = 0;
const int resolution = 12;
int previous_dac_val = 0;

unsigned long previousMillis = 0;

///////
// PWM INPUT VARIABLES
int pwmPin = SLOT_IO0pin;
unsigned long high = 0;
unsigned long low = 0;
unsigned long microsecForsec = 1000000;

unsigned int dutyCycle = 0;
unsigned int frequency = 0;

int analogval = 0;
int analoglimit = 800;
bool ishigh = false;
/////////////
int upper_value = 0;
int lower_value = 0;
int slot_number = 0;
int slot_type = 0;
int ble_state = 0;
int val = 0;
int dac_boost = 0;
int input_w_gain = 0;
bool dac_result = 0;
uint32_t duty = 0;

constexpr uint8_t CORE_TASK_SPI_SLAVE{0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER{0};
static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

struct Config
{
  // int slot_number_json;
  int slot_type_json;
  int HW_Version_json;
  int SW_Version_json;
};

Config config;

uint32_t primaryColors[10] = {
    // SWAP RED AND GREEN (G,R,B) for slot boards V1
    RGBled.Color(0, 0, 0),       // 0 off
    RGBled.Color(0, 255, 0),     // 1 Red
    RGBled.Color(255, 255, 255), // 2 White
    RGBled.Color(0, 128, 128),   // 3 Purple
    RGBled.Color(140, 255, 0),   // Orange
    RGBled.Color(0, 0, 255),     // Blue
    RGBled.Color(255, 0, 0),     // Green
    RGBled.Color(105, 255, 180), // Pink
    RGBled.Color(255, 255, 0),   // Yellow
    RGBled.Color(128, 0, 128)    // Brown

    // With correct order (R,G,B): 
    // Seems to be correct with V2 Slot boards...
    // RGBled.Color(0, 0, 0),       // 0 off
    // RGBled.Color(255, 0, 0),     // 1 Red
    // RGBled.Color(255, 255, 255), // 2 White
    // RGBled.Color(128, 0, 128),   // 3 Purple
    // RGBled.Color(255, 140, 0),   // Orange
    // RGBled.Color(0, 0, 255),     // Blue
    // RGBled.Color(0, 255, 0),     // Green
    // RGBled.Color(255, 105, 180), // Pink
    // RGBled.Color(255, 255, 0),   // Yellow
    // RGBled.Color(0, 128, 128)    // Brown
};

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

void loop(void) {
  togglePinNonBlocking(100,20);
  // dac.setVoltage(4095,false);
  // dac.setVoltage(0,false);
}