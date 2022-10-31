/**********************************************************************************
*
*	Purpose:			Header-file for SP49 device-library
*
*	Target:				SP49 Bxx
*
*	Toolchain:			Keil uVision5 V5.26.2.0
*
***********************************************************************************
*	Copyright (C) Infineon Technologies 2019.  All rights reserved.
***********************************************************************************
*	This SOFTWARE is Provided "AS IS" Without ANY WARRANTIES OF ANY KIND, WHETHER
*	EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
*	MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE or warranties of
*	non-infringement of THIRD PARTY intellectual property rights. infineon
*	disclaims all liability regardless of the cause in law, in particular, but
*	not limited to, the liability for indirect or consequential damages arising
*	from interrupted operation, loss of profits, loss of information and data,
*	unless in cases of gross negligence, intent, lack of assured characteristics
*	or in any cases where liability is mandatory at law.
***********************************************************************************/
#include "SP49.h"
#include <stdio.h>

/**********************************************************************************
Defines and types
***********************************************************************************/
#define UART_CONFIG_REFCLK_HPRC				(0x00)
#define UART_CONFIG_REFCLK_XTAL				(0x01)
#define UART_CONFIG_REFCLK_MASK				(0x01)

#define UART_SENDHEX_PREFIX_DISABLED	(0x00)
#define UART_SENDHEX_PREFIX_ENABLED		(0x80)
#define UART_SENDHEX_PREFIX_MASK			(0x80)

#define UART_STATUS_OK					(0x00)
#define UART_STATUS_TIMEOUT			(0x01)
#define UART_STATUS_OUTOFRANGE	(0x02)

enum BaudrateSel_Options
{
	BR_19k2 = 0x01,
	BR_57k6 = 0x02,
	BR_96k = 0x03
};


/**********************************************************************************
Delay1ms - Delays multiple of 1ms in run-sate and permanently resets the WDOG during that

Prerequisite: -
Input: 	uint32_t NumMilliseconds: How many milliseconds shall be delayed
Return: -
***********************************************************************************/
void Delay1ms(uint32_t NumMilliseconds);

/**********************************************************************************
Delay1ms - Delays multiple of 10us in run-sate

Prerequisite: -
Input: 	uint32_t Num10Microseconds: How many of 10microseconds shall be delayed
Return: -
***********************************************************************************/
void Delay10us(uint32_t Num10Microseconds);

/**********************************************************************************
Uart_Init - Sets PP0 as Uart-Rx and PP1 as Uart-Tx and configures baudrate- and timeout-
						timer values for typical condition.
Prerequisite: -
Input: 	uint8_t Config: Configuration-Byte
												Config[0]: 0 = Use 12Mhz for baudrate-generation, 1 = Use Xtal for baudrate-generation
				enum BaudrateSel_Option: Desired Uart baudrate. For possible values see: enum BaudrateSel_Options. 
Return: -
***********************************************************************************/
void Uart_Init(uint8_t Config, enum BaudrateSel_Options BaudrateSel_Option);

/**********************************************************************************
Uart_Baudrate_Calib - If HPRC is used as baudrate-clock, this function might be required to be called for precise baudrate.
											E.g. when device is at extreme temperatures, the HPRC might deviate too much from ideal frequency for exact baudrate.

Prerequisite: Uart must be initialized via Uart_Init. Xtal must have been started.
Input: 	-
Return: uint8_t Status: 0 = okay, 1 = timeout, 2 = out of range
***********************************************************************************/
uint8_t Uart_Baudrate_Calib(void);

/**********************************************************************************
Uart_Putchar - Transmits one byte using PP0 as Uart-Rx and PP1 as Uart-Tx at a fixed baudrate of 19.2kBaud

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint8_t Byte: The byte that shall be transmitted
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_Putchar(uint8_t Byte);

/**********************************************************************************
fputc - Overload function required for printf, sending one character

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	int ch: The character to be transmitted
				FILE *f: Not used
Return: int ch: the character which was sent out
***********************************************************************************/
int fputc(int ch, FILE *f);

/**********************************************************************************
Uart_SendString - Transmits a given string via Uart.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	sint8_t* String: Pointer to the string that shall be transmitted
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendString(uint8_t* String);

/**********************************************************************************
Uart_SendHex32 - Transmits a given 32bit-value in ASCII-format as hex-value. The '0x' prefix can be added on demand.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in hex-format
				uint8_t Config: Configuration-byte
												Config[7]: 0 = Do not send '0x' prefix, 1 = Send '0x' prefix
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendHex32(uint32_t Value, uint8_t Config);

/**********************************************************************************
Uart_SendHex16 - Transmits a given 16bit-value in ASCII-format as hex-value. The '0x' prefix can be added on demand.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in hex-format
				uint8_t Config: Configuration-byte
												Config[7]: 0 = Do not send '0x' prefix, 1 = Send '0x' prefix
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendHex16(uint16_t Value, uint8_t Config);

/**********************************************************************************
Uart_SendHex8 - Transmits a given 8bit-value in ASCII-format as hex-value. The '0x' prefix can be added on demand.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in hex-format
				uint8_t Config: Configuration-byte
												Config[7]: 0 = Do not send '0x' prefix, 1 = Send '0x' prefix
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendHex8(uint8_t Value, uint8_t Config);

/**********************************************************************************
Uart_SendDecimalSigned - Transmits a given signed 32bit-value in ASCII-format as decimal-value using fixed baudrate of 19.2kBaud.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case Config[0] is 1 (use Xtal), Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in decimal-format
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendDecimalSigned(int32_t Value);

/**********************************************************************************
Uart_SendDecimalUnsigned - Transmits a given unsigned 32bit-value in ASCII-format as decimal-value using fixed baudrate of 19.2kBaud.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case Config[0] is 1 (use Xtal), Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in decimal-format
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendDecimalUnsigned(uint32_t Value);
