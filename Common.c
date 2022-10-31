#include "Common.h"

void swapBytes(void* number, int bytes_num)
{
    uint8_t* ptr = number;
    int i = 0;
    for ( i = 0; i<bytes_num/2; i++) {
        uint8_t tmp = ptr[i];
        ptr[i] = ptr[bytes_num-1-i];
        ptr[bytes_num-1-i] = tmp;
    }
}