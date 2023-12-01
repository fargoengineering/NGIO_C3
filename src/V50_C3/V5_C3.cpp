// Use flash size as 4mb
#include <Arduino.h>
#include <ESP32SPISlave.h>
#include <Adafruit_BusIO_Register.h>
#include <atomic>
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
#include "RS-FEC.h"  



ESP32SPISlave slave;
const int msglen = 8;
const uint8_t ECC_LENGTH = 4;
RS::ReedSolomon<msglen, ECC_LENGTH> rs; // leng ECC leng
uint8_t dataDecoded[msglen];
// uint8_t dataDecoded[msglen];
uint8_t dataEncoded[ECC_LENGTH + msglen];
uint8_t dataAckRaw[msglen];
uint8_t dataAckEncoded[ECC_LENGTH + msglen];

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10
volatile uint8_t newLEDdata[3];
volatile bool newDataAvail = false;
static constexpr uint32_t BUFFER_SIZE{64};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

int slot_type = 0;

const char VERSION[] = "V4_C3";
const char *filename = "/config.txt";

const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b
const int DIGOUTpin = 19;

bool first_run = false;
// atomics
std::atomic<int> data_out_atom(0);    // Data OUT
std::atomic<int> data2_out_atom(0);   // Data OUT
std::atomic<int> slot_type_atom(0);   // Slot Type Byte (Aux)
std::atomic<int> pdo1(0);             // PDO 1 Output
std::atomic<int> pdo2(0);             // PDO 2 Output
std::atomic<int> pdo3(0);             // PDO 3 Output
std::atomic<int> pdo4(0);             // PDO 4 Output
std::atomic<int> relay_state_atom(0); // Holds state of the relays
std::atomic<int> command_atom(0);     // Holds Command Byte when applicable
std::atomic<bool> first_run_atom(0);  // Marks run setup flag
std::atomic<int> HW_Version_atom(0);  // Hardware Version
std::atomic<int> SW_Version_atom(0);  // Software Version
// BLE OTA
std::atomic<int> ble_state_atom(0);   // BLE State Byte (Aux)
bool ble_enabled = false; // FEI
int ble_state = 0;

struct Config
{
    // int slot_number_json;
    int slot_type_json;
    int HW_Version_json;
    int SW_Version_json;
};

Config config;

uint16_t checksumCalculator(uint8_t *data, uint16_t length)
{
    uint16_t curr_crc = 0x0000;
    uint8_t sum1 = (uint8_t)curr_crc;
    uint8_t sum2 = (uint8_t)(curr_crc >> 8);
    int index;
    for (index = 0; index < length; index = index + 1)
    {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

void loadConfiguration(const char *filename, Config &config)
{
    // Serial.println(F("Checking Config File"));
    if (SPIFFS.begin(true))
    {
        File file = SPIFFS.open(filename, "r");
        StaticJsonDocument<2000> doc;
        DeserializationError error = deserializeJson(doc, file);
        if (error)
        {
            // Serial.println(F("Failed to read file..."));
        }
        // config.slot_number_json = doc["slot_number_json"];
        config.slot_type_json = doc["slot_type_json"];
        config.HW_Version_json = doc["HW_Version_json"];
        config.SW_Version_json = doc["SW_Version_json"];
        // Serial.printf("Retrieved slot type {%d} from config file\n", config.slot_type_json);
    }
    else
    {
        // Serial.println(F("SPIFFS FAULT"));
    }
}

void saveConfiguration(const char *filename, const Config &config)
{
    // Serial.println(F("saving config file..."));
    if (SPIFFS.begin(true))
    {
        File file = SPIFFS.open(filename, "w");
        if (!file)
        {
            // Serial.println(F("Failed to save config file..."));
            return;
        }
        StaticJsonDocument<2000> doc;
        // doc["slot_number_json"] = config.slot_number_json;
        doc["slot_type_json"] = config.slot_type_json;
        doc["SW_Version_json"] = config.SW_Version_json;
        doc["HW_Version_json"] = config.HW_Version_json;
        if (serializeJson(doc, file) == 0)
        {
            // Serial.println(F("Failed to save config file..."));
        }
        file.close();
    }
}

void set_buffer()
{
    // for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
    //   spi_slave_tx_buf[i] = (0xFF - i) & 0xFF;
    // }
    memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

constexpr uint8_t CORE_TASK_SPI_SLAVE{0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER{0};

static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

void task_wait_spi(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // block until the transaction comes from master
        // memset(spi_slave_tx_buf,0x00,BUFFER_SIZE);
        // memcpy(spi_slave_tx_buf, dataAckEncoded, ECC_LENGTH + msglen);  //copy into encoded variable
        slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

        xTaskNotifyGive(task_handle_process_buffer);
    }
}

void task_process_buffer(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        memcpy(dataEncoded, spi_slave_rx_buf, ECC_LENGTH + msglen);

        // //decode the entire buffer
        memset(dataDecoded, 0x00, msglen);
        rs.Decode(dataEncoded, dataDecoded); // src dest
        bool dataWasCorrected = false;
        // printf("RAW DATA DECODED ->");
        for (size_t i = 0; i < msglen; ++i)
        {
            // printf("%02x ", dataDecoded[i]);
            if (dataDecoded[i] != dataEncoded[i])
                dataWasCorrected = true;
        }
        // printf("\n");
        if (dataWasCorrected && dataDecoded[0] != 0)
            Serial.println("!!!DATA WAS CORRECTED!!!!");

        if (dataDecoded[0] == '$' && dataDecoded[5] == '\n')
        {                                                           // let first process
            uint16_t checkSum = checksumCalculator(dataDecoded, 6); // only load checksum if LED data
            uint16_t checksumFromS3 = dataDecoded[6] + (dataDecoded[7] << 8);
            // memset(spi_slave_tx_buf, 0x00, BUFFER_SIZE);
            //  memcpy(spi_slave_tx_buf, dataAckEncoded, ECC_LENGTH + msglen);  //copy into encoded variable

            if (checksumFromS3 == checkSum)
            { // this is only if data is truly good
                // dataAckRaw is the tx_buf
                uint16_t data = data_out_atom.load();
                uint16_t data2 = data2_out_atom.load();
                newDataAvail = true;
                memset(dataAckRaw, 0x00, msglen);
                dataAckRaw[0] = dataDecoded[0];
                dataAckRaw[1] = data;
                dataAckRaw[2] = data>>8;
                dataAckRaw[3] = data2;
                dataAckRaw[4] = data2>>8;
                dataAckRaw[5] = dataDecoded[5];
                dataAckRaw[6] = checkSum;
                dataAckRaw[7] = checkSum >> 8;

                memset(dataAckEncoded, 0x00, ECC_LENGTH + msglen);
                rs.Encode(dataAckRaw, dataAckEncoded);
                memcpy(spi_slave_tx_buf, dataAckEncoded, ECC_LENGTH + msglen); // ONLY DO THIS HERE!!!! Don't touch tx buff

                newLEDdata[1] = dataDecoded[2]; // g  we flip for V2, RGB format
                newLEDdata[0] = dataDecoded[3]; // r
                newLEDdata[2] = dataDecoded[4]; // b
                slot_type_atom.store(newLEDdata[2]);
            }
            else
            { // red if check fails on way there
                // Serial.println("FAIL-CHECK");
                //  newLEDdata[0] = 200;
                //  newLEDdata[1] = 0;
                //  newLEDdata[2] = 0;
                // Serial.println("failed after");
            }
        }
        else if (dataDecoded[0] == 'b')
        { // send a 'b' instead of a '$' from S3 to enable BLE
            ble_state_atom.store(1);
        }
        else if(dataDecoded[0] == 'p'){
            ble_state_atom.store(0);
        }

        slave.pop();
        xTaskNotifyGive(task_handle_wait_spi);
    }
}

void setup()
{
    Serial.begin(230400);
    pinMode(DIGOUTpin, OUTPUT);
    digitalWrite(DIGOUTpin, LOW);

    RGBled.begin();
    RGBled.setPixelColor(0, RGBled.Color(100, 0, 0));
    RGBled.show();
    delay(100);
    RGBled.setPixelColor(0, RGBled.Color(0, 100, 0));
    RGBled.show();
    delay(100);
    RGBled.setPixelColor(0, RGBled.Color(0, 0, 100));
    RGBled.show();
    delay(100);
    RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
    RGBled.show();
    delay(200);

    // Pin Configuration
    pinMode(SLOT_TP1pin, OUTPUT);
    pinMode(BYPASSpin, OUTPUT);
    pinMode(RELAYpin, OUTPUT);
    pinMode(DIGI_OUTpin, OUTPUT);
    
    // I2C setup
    Wire.begin(SDA, SCL);

    // Set led to green on setup
    RGBled.setPixelColor(0, RGBled.Color(100,0,100));
    RGBled.setBrightness(128);
    RGBled.show();
    slave.setDataMode(SPI_MODE0);
    
    slave.begin(SPI2_HOST, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
    set_buffer();

    xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
    xTaskNotifyGive(task_handle_wait_spi);

    xTaskCreatePinnedToCore(
        task_process_buffer,
        "task_process_buffer",
        2048,
        NULL,
        2,
        &task_handle_process_buffer,
        CORE_TASK_PROCESS_BUFFER);

    first_run_atom.store(1);
}

unsigned long previousMillis=0;
bool blink = false;

void loop()
{
    first_run = first_run_atom.load();
    ble_state = ble_state_atom.load();
    slot_type = slot_type_atom.load();
    
    first_run_atom.store(0);

    unsigned long currentMillis=millis();
    if(ble_enabled){
        if(currentMillis-previousMillis >=500){
            previousMillis = currentMillis;
            if(blink){
                RGBled.setPixelColor(0,RGBled.Color(0,0,200));
                RGBled.setBrightness(128);
                RGBled.show();
                blink=false;
            }
            else{
                RGBled.setPixelColor(0,RGBled.Color(0,0,0));
                RGBled.setBrightness(128);
                RGBled.show();
                blink=true;
            }
        }
    }

    if(ble_state == 1 && ble_enabled == false){
        String slot = "SLOT_";
        String mac = WiFi.macAddress();
        ble_enabled = true;
        ota_dfu_ble.begin(slot+mac);
        delay(500);
    }else if((ble_state == 0) && (ble_enabled == true)){
        ESP.restart();   // seems to cause issues? But no other way to stop ble...
    }


    if (newDataAvail)
    {
        newDataAvail = false;
        RGBled.setPixelColor(0, RGBled.Color(newLEDdata[0], newLEDdata[1], newLEDdata[2]));
        RGBled.show();
    }
}