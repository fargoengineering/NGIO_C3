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
//PWM INPUT VARIABLES
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

void setRelays()
{
  int relay_state = relay_state_atom.load();
  switch (relay_state)
  {
  case 0:
    digitalWrite(RELAYpin, LOW);
    digitalWrite(BYPASSpin, LOW);
    break;
  case 1:
    digitalWrite(RELAYpin, HIGH);
    digitalWrite(BYPASSpin, LOW);
    break;
  case 2:
    digitalWrite(RELAYpin, LOW);
    digitalWrite(BYPASSpin, HIGH);
    break;
  case 3:
    digitalWrite(RELAYpin, HIGH);
    digitalWrite(BYPASSpin, HIGH);
    break;
  default:
    break;
  }
}

void GetPWMDetails(byte pin,int threshold)
{
  // Let this function be included in the C3 Sketch
  unsigned long highTime = pulseIn_FEI(pin, HIGH, 50000UL, threshold);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn_FEI(pin, LOW, 50000UL, threshold);  // 50 millisecond timeout

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    dutyCycle = get_digital_from_analog(pin, threshold) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  dutyCycle = (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
  frequency = (1 * microsecForsec) / (highTime + lowTime);
  high = highTime;
  low = lowTime;
}

void pwm_bit_bang_micro(int freq, int pin, int duty_cycle) {
  // Calculate the period and high time of the PWM signal.
  int period = 1000000 / freq;
  int high_time = period * duty_cycle / 100;
  int low_time = period - high_time;

  // Start a timer to track the elapsed time.
  unsigned long start_time = micros();

  // If the high time has elapsed, set the pin low.
  if (micros() - start_time >= high_time) {
    digitalWrite(pin, LOW);
  }

  // If the low time has elapsed, set the pin high.
  if (micros() - start_time >= period) {
    digitalWrite(pin, HIGH);
    start_time = micros();
  }  
}

void pwm_bit_bang_millis(int freq, int pin, int duty_cycle) {
  // Calculate the period and high time of the PWM signal in milliseconds.
  int period = 1000 / freq;
  int high_time = period * duty_cycle / 100;
  int low_time = period - high_time;

  // Start a timer to track the elapsed time in milliseconds.
  unsigned long start_time = millis();

  // If the high time has elapsed, set the pin low.
  if (millis() - start_time >= high_time) {
    digitalWrite(pin, LOW);
  }

  // If the low time has elapsed, set the pin high.
  if (millis() - start_time >= period) {
    digitalWrite(pin, HIGH);
    start_time = millis();
  }
}


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
    Serial.printf("Retrieved slot type {%d} from config file\n",config.slot_type_json);
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
      command = spi_slave_rx_buf[0];
      spi_slave_tx_buf[0] = spi_slave_rx_buf[0];
      spi_slave_tx_buf[1] = data;
      spi_slave_tx_buf[2] = data >> 8;
      spi_slave_tx_buf[3] = data2;
      spi_slave_tx_buf[4] = data2 >> 8;
      spi_slave_tx_buf[5] = spi_slave_rx_buf[5]; // data3

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

  // Create background tasks/threads (SPI R/W)
  xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
  xTaskNotifyGive(task_handle_wait_spi);
  xTaskCreatePinnedToCore(task_process_buffer, "task_process_buffer", 2048, NULL, 2, &task_handle_process_buffer, CORE_TASK_PROCESS_BUFFER);
}

void loop()
{
  while(1)
  {
    // perform atomic operations
    digitalWrite(SLOT_TP1pin, !digitalRead(SLOT_TP1pin));
    first_run = first_run_atom.load();
    slot_type = slot_type_atom.load();
    ble_state = ble_state_atom.load();

    // handle LED status
    unsigned long currentMillis = millis();

    if (ble_enabled)
    {
      if (currentMillis - previousMillis >= 500)
      {
        previousMillis = currentMillis;
        if (blink)
        {
          RGBled.setPixelColor(0, RGBled.Color(0, 0, 200));
          RGBled.setBrightness(128);
          RGBled.show();
          blink = false;
        }
        else
        {
          RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
          RGBled.setBrightness(128);
          RGBled.show();
          blink = true;
        }
      }
    }
    else
    {
      RGBled.setPixelColor(0, primaryColors[slot_type]);
      RGBled.setBrightness(128);
      RGBled.show();
    }
    unsigned int data;
    unsigned int data2;

    // Will hit if slot type is updated
    if (first_run == 1)
    {
      // Store Slot type 
      config.slot_type_json = slot_type;
      saveConfiguration(filename,config);

      RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
      RGBled.setBrightness(0);
      RGBled.show();
      // this switch case was originally in setup(), but first_run allows us to reconfigure without power cycling
      switch (slot_type)
      {
      case 1: // Digital Output
        pinMode(DIGI_OUTpin, OUTPUT);
        break;
      case 2: // Digital Input
        pinMode(SLOT_IO0pin, INPUT);
        break;
      case 3: // Analog Input
        pinMode(SLOT_IO0pin, INPUT);
        break;
      case 4: // Analog Output / DAC
        Wire.begin(SDA, SCL);
        dac.begin(0x60);
        break;
      case 5: // PWM (input)
        my_pwm.begin(true); // method 1 (digital)
        pinMode(SLOT_IO0pin,INPUT);   // method 2 (analog)
        break;
      case 6: // Frequency (output)
        pinMode(DIGI_OUTpin, OUTPUT);
        ledcAttachPin(DIGI_OUTpin, ledChannel);
        break;
      case 7: // n/a
        break;
      case 8: // n/a
        break;
      case 9: // test bit bang output
        // pinMode(DIGI_OUTpin, OUTPUT);
        pinMode(pwmPin,INPUT);
        break;
      case 10: // ?
        Wire.begin(SDA, SCL);
        dac.begin(0x60);
        pinMode(DIGI_OUTpin, OUTPUT);
        break;
      default:
        break;
      }
      // Mark First run as complete
      first_run_atom.store(0);
    }

    switch (slot_type)
    {
    case 1:
      val = pdo2.load();
      digitalWrite(DIGI_OUTpin, val);
      data = digitalRead(DIGI_OUTpin);
      break;
    case 2:
      // Do an analog read, set threshold on pi master. if read above threshold, digital high, else low (handled on py side)
      data = analogRead(SLOT_IO0pin); // pdo 2 output
      break;
    case 3:
      data = analogRead(SLOT_IO0pin); // pdo 1 and 2 MSB
      // Serial.printf("a2d count: %d\n",data);
      break;
    case 4:
      // data = 4000;
      upper_value = pdo1.load();
      lower_value = pdo2.load();
      val = (upper_value << 8) | lower_value;
      dac.setVoltage(val, false);
      data = analogRead(SLOT_IO0pin); // pdo 1 and 2 MSB
      break;
    case 5:
      // digital PWM
      // data = my_pwm.getValue(); // to pdo1 and 2, good
      upper_value = pdo1.load();
      lower_value = pdo2.load();
      analoglimit = (upper_value << 8) | lower_value;
      GetPWMDetails(SLOT_IO0pin,analoglimit);
      // Serial.printf("threshold: %d\n",analoglimit);
      // Serial.printf("duty: %d\n",dutyCycle);
      // Serial.printf("frequency: %d\n",frequency);
      data = dutyCycle;
      data2 = frequency;
      break;
    case 6:
      // freq value pdo1 and pdo2
      // pdo 3 = freq_value
      // pdo 4 = duty_cycle
      upper_value = pdo1.load();
      lower_value = pdo2.load();
      freq_value = (upper_value << 8) | lower_value;
      upper_value = pdo3.load();
      lower_value = pdo4.load();
      duty = (upper_value << 8) | lower_value;
      data = freq_value;
      data2 = duty;
      // Freq output
      if (freq_value != freq_value_last)
      {
        // If Frequency has been changed!
        freq_value_last = freq_value;
        ledcDetachPin(DIGI_OUTpin);
        ledcSetup(ledChannel, freq_value, resolution);
        ledcAttachPin(DIGI_OUTpin, ledChannel);
      }
      if (freq_value >= 200)
      {
        // High Speed Freq
        ledcWrite(ledChannel, duty); // PDO byte 3 4 buffer will set PWM here.
        // Serial.println("Frequency greater than 200");
      }
      else if (freq_value < 200)
      { 
        // Low Speed Freq
        pwm_bit_bang_millis(freq_value,DIGI_OUTpin,duty);
      }
      break;
    case 7:
      data = 7000;
      break;
    case 8:
      data = 8000;
      data2 = 9000;
      // config.slot_type_json = slot_type;
      // saveConfiguration(filename, config);
      // first_run = true;
      delay(500);
      break;
    case 9:
      // TEST: PWM input based on analogRead() - 10-26-2023
      upper_value = pdo1.load();
      lower_value = pdo2.load();
      analoglimit = (upper_value << 8) | lower_value;
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
          prevMillis = currentMillis; 
          data = frequency;
          data2 = dutyCycle;
      }
      break;
    case 10:
      digitalWrite(DIGI_OUTpin, HIGH);
      delay(20);
      digitalWrite(DIGI_OUTpin, LOW);
      delay(20);
      break;
    default:
      data = 0;
      break;
    }

    // Set PDO Output data
    data_out_atom.store(data);
    data2_out_atom.store(data2);

    // BLE OTA check
    if (ble_state == 1 && ble_enabled == false)
    {
      // BLE Configuration
      String slot = "SLOT_";
      String mac = WiFi.macAddress();
      ble_enabled = true;
      RGBled.setPixelColor(0, primaryColors[5]);
      RGBled.setBrightness(128);
      ota_dfu_ble.begin(slot + mac);
      delay(500);
    }
    else if ((ble_state == 0) && (ble_enabled == true))
    {
      RGBled.setPixelColor(0, primaryColors[6]);
      RGBled.setBrightness(128);
      ESP.restart(); // if ble is on and received message to turn off, reboot ESP
    }
    else if (ble_state == 2)
    {
      ESP.restart(); // if ble is on and auxdata = 2, reboot ESP
    }

    // handle relays
    setRelays();
  } 
}