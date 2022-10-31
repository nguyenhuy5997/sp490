//-------------------------------------------------------------------------------------------------
//  File name:      simple_ecc_lib.h
//  Description:    Simple_ecc
//  Author:         phuocnt6
//-------------------------------------------------------------------------------------------------

//#include <stdio.h>
//#include <stdint.h>

#include "Hamming.h"

//----- simple xor 1 byte - 8bit
uint8_t inter_byte_xor(uint8_t inData)
{
    uint8_t result;
    result = 0;

    for(uint32_t n = 0; n < 8; n++)
    {
        result = result ^ (inData & 0x01);
        inData = inData >> 1;
    }
    return result;
}

//----- simple hamming (8,4) encoder
uint8_t hamming_8_4_enc(uint8_t inData)
{
    uint8_t result;

    if (inData > 15)
        return 0x01;

    switch(inData)
    {
        case 0: result = 0x00; break;
        case 1: result = 0xD2; break;
        case 2: result = 0x55; break;
        case 3: result = 0x87; break;
        case 4: result = 0x99; break;
        case 5: result = 0x4B; break;
        case 6: result = 0xCC; break;
        case 7: result = 0x1E; break;
        case 8: result = 0xE1; break;
        case 9: result = 0x33; break;
        case 10: result = 0xB4; break;
        case 11: result = 0x66; break;
        case 12: result = 0x78; break;
        case 13: result = 0xAA; break;
        case 14: result = 0x2D; break;
        default: result = 0xFF;
    }
    return result;
}

//----- simple hamming (8,4) decoder
uint8_t hamming_8_4_dec(uint8_t inData)
{
    uint8_t result;
    uint8_t c1r, c2r, c3r, c4r;
    uint8_t cr;

    c1r = inter_byte_xor(inData & 0xAA);
    c2r = inter_byte_xor(inData & 0x66);
    c3r = inter_byte_xor(inData & 0x1E);
    c4r = inter_byte_xor(inData & 0xFF);

    result = ((inData & 0x0E) >> 1) + ((inData & 0x20) >> 2);
    cr = (c1r << 3) + (c2r << 2) + (c3r << 1) + c4r;

    switch(cr)
    {
        //  there is 1 wrong bit
        case 0x00:  break;

        //  there is not 1 wrong data bit but there is 1 bit parity bit
        case 0x09:  break;
        case 0x05:  break;
        case 0x03:  break;
        case 0x01:  break;

        //  there is 1 wrong data bit
        case 0x0F:  result = result ^ 0x01; break;
        case 0x07:  result = result ^ 0x02; break;
        case 0x0B:  result = result ^ 0x04; break;
        case 0x0D:  result = result ^ 0x08; break;

        //  there is 2 wrong data bit
//        case 0x08:  result = result + 0x10; break;
//        case 0x0C:  result = result + 0x10; break;
//        case 0x0A:  result = result + 0x10; break;
//        case 0x02:  result = result + 0x10; break;

        default: result = result + 0x10; break;
    }

    return result;
}

//----- simple hamming (8,4) encoder for byte array.
//  Important note: the length of output array must be twice of the length of input array
uint32_t hamming_8_4_array_enc(uint8_t *inBytesP, uint32_t inBytesLength, uint8_t *outBytesP)
{
    uint32_t outByteCount;
    uint8_t inData;

    outByteCount = 0;
    while(1)
    {
        if((outByteCount >> 1) >= inBytesLength)
            break;

        if(outByteCount % 2 == 0)
            inData = *(inBytesP + (outByteCount >> 1)) & 0x0F;
        else
        {
            inData = *(inBytesP + (outByteCount >> 1)) & 0xF0;
            inData = inData >> 4;
        }

        *(outBytesP + outByteCount) = hamming_8_4_enc(inData);

        outByteCount++;
    }

    return 0;
}

//----- simple hamming (8,4) encoder for byte array.
//  Important note: the length of input array must be twice of the length of output array
uint32_t hamming_8_4_array_dec(uint8_t *inBytesP, uint32_t inByteLength, uint8_t *outBytesP)
{
    uint32_t inByteCount;
    uint8_t encData;
    uint8_t decData;
    uint8_t sOutByte;

    //  Detected: the input length is wrong. No even
    if(inByteLength % 2 == 1)
        return 2;

    //  Detected: the input length is wrong. Size > 1MB = 2^20 bytes
    if(inByteLength >= (1 << 20))
        return 3;

    inByteCount = 0;
    while(1)
    {
        if(inByteCount >= inByteLength)
            break;

        encData = *(inBytesP + inByteCount);
        decData = hamming_8_4_dec(encData);
        sOutByte = *(outBytesP + (inByteCount >> 1));
        sOutByte = sOutByte & 0x0F;

        //  Detected the error bit
        if(decData > 15)
            return ((1 << 20) + inByteCount);

        if(inByteCount % 2 == 1)
        {
            decData = (decData << 4) + sOutByte;
        }

        *(outBytesP + (inByteCount >> 1)) = decData;

        inByteCount++;
    }

    return 0;
}
