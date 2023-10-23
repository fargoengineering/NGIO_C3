#include<Arduino.h>
#include "CRC.h"

uint16_t CRC::checksumCalculator_Original(uint8_t * data, uint16_t length)
{
   uint16_t curr_crc = 0x0000;
   uint8_t sum1 = (uint8_t) curr_crc;
   uint8_t sum2 = (uint8_t) (curr_crc >> 8);
   int index;
   for(index = 0; index < length; index = index+1)
   {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

uint16_t CRC::checksumCalculator(uint8_t * data, uint16_t length)
{
    if(data[0] == 0)
    {
        return 0;
    }
    uint16_t curr_crc = 0x0000;
    uint8_t sum1 = (uint8_t) curr_crc;
    uint8_t sum2 = (uint8_t) (curr_crc >> 8);
    int index;
    for(index = 0; index < length; index = index+1)
    {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

uint16_t CRC::checksumCalculator_FEI(uint8_t * data, uint16_t length)
{
    if(data[0] == 0)
    {
        return 0;
    }
    uint16_t curr_crc = 0x0000;
    int index;
    for(index = 0; index < length; index = index+1)
    {
        curr_crc = curr_crc + data[index];
    }
    return curr_crc;
}