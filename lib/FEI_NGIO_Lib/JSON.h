#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

class JSON
{
    public:

        void loadConfiguration(const char *filename, Config &config);
        void saveConfiguration(const char *filename, Config &config);
};