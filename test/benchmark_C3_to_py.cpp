#include <ESP32SPISlave.h>
#include <Arduino.h>
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
std::atomic<int> data_out_atom(0);
std::atomic<int> data2_out_atom(0);
std::atomic<int> slot_type_atom(0);
std::atomic<int> ble_state_atom(0);
std::atomic<int> pdo1(0);
std::atomic<int> pdo2(0);
std::atomic<int> pdo3(0);
std::atomic<int> pdo4(0);
std::atomic<int> relay_state_atom(0);
std::atomic<bool> first_run_atom(0);

Adafruit_MCP4725 dac;
#define DAC_RESOLUTION (12)

CRC crc;
PWM my_pwm(SLOT_IO0pin);

const int pwmResolution = 10;
int freq_value = 0;
;
int freq_value_last = 0;
int pwm_value = 0;
int pwm_value_last = 0;
bool low_speed_freq = false;
int pwmState = LOW;
int freq = 50;
const int ledChannel = 0;
const int resolution = 12;

unsigned long previousMillis = 0;

///////
// PWM INPUT VARIABLES
int pwmPin = SLOT_IO0pin;

unsigned long high1 = 0;
unsigned long high2 = 0;
unsigned long low = 0;
unsigned long highdiff = 0;

unsigned long high = 0;
unsigned long microsecForsec = 1000000;
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
/////////////

int upper_value = 0;
int lower_value = 0;
int slot_number;
int slot_type;
int ble_state;
int val;
uint32_t duty = 0;

constexpr uint8_t CORE_TASK_SPI_SLAVE{0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER{0};
static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

struct Config
{
    // int slot_number_json;
    int slot_type_json;
};

Config config;

uint32_t primaryColors[10] = {
    // SWAP RED AND GREEN (G,R,B)
    RGBled.Color(0, 0, 0),       // 0 off
    RGBled.Color(0, 255, 0),     // 1 Red
    RGBled.Color(255, 255, 255), // 2 White
    RGBled.Color(0, 128, 128),   // 3 Purple
    RGBled.Color(140, 255, 0),   // Orange
    RGBled.Color(0, 0, 255),     // Blue
    RGBled.Color(255, 0, 0),     // Green
    RGBled.Color(255, 255, 0),   // Yellow
    RGBled.Color(105, 255, 180), // Pink
    RGBled.Color(128, 0, 128)    // Brown
};


void loadConfiguration(const char *filename, Config &config)
{
    Serial.println(F("Checking Config File"));
    if (SPIFFS.begin(true))
    {
        File file = SPIFFS.open(filename, "r");
        StaticJsonDocument<2000> doc;
        DeserializationError error = deserializeJson(doc, file);
        if (error)
        {
            Serial.println(F("Failed to read file..."));
        }
        // config.slot_number_json = doc["slot_number_json"];
        config.slot_type_json = doc["slot_type_json"];
        Serial.printf("Retrieved slot type {%d} from config file\n", config.slot_type_json);
    }
    else
    {
        Serial.println(F("SPIFFS FAULT"));
    }
}

void saveConfiguration(const char *filename, const Config &config)
{
    Serial.println(F("saving config file..."));
    if (SPIFFS.begin(true))
    {
        File file = SPIFFS.open(filename, "w");
        if (!file)
        {
            Serial.println(F("Failed to save config file..."));
            return;
        }
        StaticJsonDocument<2000> doc;
        // doc["slot_number_json"] = config.slot_number_json;
        doc["slot_type_json"] = config.slot_type_json;
        if (serializeJson(doc, file) == 0)
        {
            Serial.println(F("Failed to save config file..."));
        }
        file.close();
    }
}

void set_buffer()
{
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
        spi_slave_tx_buf[i] = 0xFF;
    }
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

void task_wait_spi(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // block until the transaction comes from master
        slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

        xTaskNotifyGive(task_handle_process_buffer);
    }
}

void task_process_buffer(void *pvParameters)
{
    unsigned long count = 0;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t checkSumFromS3 = spi_slave_rx_buf[6] + (spi_slave_rx_buf[7] << 8);
        uint16_t checkSumOfS3 = crc.checksumCalculator(spi_slave_rx_buf, 6);
        uint16_t data = data_out_atom.load();
        uint16_t data2 = data2_out_atom.load();
        int command = 0;
        bool valid_data = false;

        if (checkSumOfS3 == checkSumFromS3 && checkSumFromS3 != 0)
        {
            count++;
            command = spi_slave_rx_buf[0];
            spi_slave_tx_buf[0] = spi_slave_rx_buf[0];
            spi_slave_tx_buf[1] = data;
            spi_slave_tx_buf[2] = data >> 8;
            spi_slave_tx_buf[3] = count;
            spi_slave_tx_buf[4] = count >> 8;
            spi_slave_tx_buf[5] = count; // data3

            uint16_t checkSumOfC3 = crc.checksumCalculator(spi_slave_tx_buf, 6);
            spi_slave_tx_buf[6] = checkSumOfC3;
            spi_slave_tx_buf[7] = checkSumOfC3 >> 8;

            valid_data = true;
        }

        if (command == 5 && (slot_type_atom.load() != spi_slave_rx_buf[5]))
        {
            first_run_atom.store(1);
            slot_type_atom.store(spi_slave_rx_buf[5]);
        }
        else if (command == 6)
        {
            ble_state_atom.store(spi_slave_rx_buf[5]);
        }
        else if (command == 7)
        {
            relay_state_atom.store(spi_slave_rx_buf[5]);
        }

        if (valid_data)
        {
            // Serial.printf("pdo1: %d\n",spi_slave_rx_buf[1]);
            pdo1.store(spi_slave_rx_buf[1]);

            // Serial.printf("pdo2: %d\n",spi_slave_rx_buf[2]);
            pdo2.store(spi_slave_rx_buf[2]);

            // Serial.printf("pdo3: %d\n",spi_slave_rx_buf[3]);
            pdo3.store(spi_slave_rx_buf[3]);

            // Serial.printf("pdo4: %d\n",spi_slave_rx_buf[4]);
            pdo4.store(spi_slave_rx_buf[4]);
        }

        slave.pop();
        xTaskNotifyGive(task_handle_wait_spi);
    }
}

void setup()
{
    Serial.begin(115200);
    // TODO: Implement JSON to remember previous slot type...
    loadConfiguration(filename, config);

    slot_type = config.slot_type_json;
    slot_type_atom.store(slot_type);
    // slot_number = config.slot_number_json;
    // if (0 < slot_type < 12)
    // {
    //   Serial.printf("Setting LED to color #%d\n",slot_type);
    //   RGBled.setPixelColor(0, primaryColors[slot_type]);
    //   RGBled.setBrightness(128);
    //   RGBled.show();
    // }

    // Pin Configuration
    pinMode(SLOT_TP1pin, OUTPUT);
    pinMode(BYPASSpin, OUTPUT);
    pinMode(RELAYpin, OUTPUT);

    // SPI Slave setup
    slave.setDataMode(SPI_MODE0);
    gpio_set_drive_capability((gpio_num_t)ESP_D5, GPIO_DRIVE_CAP_1);
    slave.begin(SPI2_HOST, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
    set_buffer();

    RGBled.setPixelColor(0, primaryColors[slot_type]);
    RGBled.setBrightness(128);
    RGBled.show();

    // Create background tasks/threads (SPI R/W)
    xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
    xTaskNotifyGive(task_handle_wait_spi);
    xTaskCreatePinnedToCore(task_process_buffer, "task_process_buffer", 2048, NULL, 2, &task_handle_process_buffer, CORE_TASK_PROCESS_BUFFER);
}

void loop()
{
    // perform atomic operations
    digitalWrite(SLOT_TP1pin, !digitalRead(SLOT_TP1pin));
}