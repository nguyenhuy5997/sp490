//-------------------------------------------------------------------------------------------------
//  File name:      simple_ecc_lib.h
//  Description:    Header of simple_ecc
//  Author:         phuocnt6
//-------------------------------------------------------------------------------------------------

#ifndef SIMPLE_ECC_LIB_H
#define SIMPLE_ECC_LIB_H

#include <stdint.h>

//----- simple xor 1 byte - 8bit
uint8_t inter_byte_xor(uint8_t inData);

//----- simple hammming (8,4) encoder
uint8_t hamming_8_4_enc(uint8_t inData);

//----- simple hammming (8,4) decoder
uint8_t hamming_8_4_dec(uint8_t inData);

//----- simple hamming (8,4) encoder for byte array.
//  Important note: the length of output array must be twice of the length of input array
uint32_t hamming_8_4_array_enc(uint8_t *inBytesP, uint32_t inBytesLength, uint8_t *outBytesP);

//----- simple hamming (8,4) encoder for byte array.
//  Important note: the length of input array must be twice of the length of output array
uint32_t hamming_8_4_array_dec(uint8_t *inBytesP, uint32_t inByteLength, uint8_t *outBytesP);

#endif