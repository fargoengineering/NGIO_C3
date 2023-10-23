#include<Arduino.h>

class CRC
{
    public:
        
        uint16_t checksumCalculator_Original(uint8_t * data, uint16_t length);
        
        uint16_t checksumCalculator(uint8_t * data, uint16_t length);
        uint16_t checksumCalculator_FEI(uint8_t * data, uint16_t length);
};