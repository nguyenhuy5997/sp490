/**********************************************************************************
*
*	Purpose:			Source-file for SP49 device-library
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
#include "DevLib.h"

static uint8_t Uart_Config_Loc;
static uint8_t Uart_BRTimer_ReloadValue_Loc;
static uint16_t Uart_TimeoutTimer_ReloadValue_Loc;
static enum BaudrateSel_Options Uart_BaudrateSel_Option_Loc;

/**********************************************************************************
Delay1ms - Delays multiple of 1ms in run-sate and permanently resets the WDOG during that

Prerequisite: -
Input: 	uint32_t NumMilliseconds: How many milliseconds shall be delayed
Return: -
***********************************************************************************/
// Do optimize this function always with level-0 in order to keep timing. push and pop save and restore the actual optimization level
#if defined ( __CC_ARM )
	#pragma push
	#pragma O0
#elif defined ( __GNUC__ )
	#pragma GCC push_options
	__attribute__ ((optimize(0)))
#endif
void Delay1ms(uint32_t NumMilliseconds)
{
	for(int i = NumMilliseconds; i > 0; i--)
	{
		// Delay 1ms
		for(int j = (1 * (12000000 / 2 / 1000) / 2); j > 0; j--);
		
		// Watchdog reset
		Corelogic->TIMERCFG01_b.WDRES = 1;
	}
}
#if defined ( __CC_ARM )
	#pragma pop
#elif defined ( __GNUC__ )
	#pragma GCC pop_options
#endif

/**********************************************************************************
Delay1ms - Delays multiple of 10us in run-sate

Prerequisite: -
Input: 	uint32_t Num10Microseconds: How many of 10microseconds shall be delayed
Return: -
***********************************************************************************/
// Do optimize this function always with level-0 in order to keep timing. push and pop save and restore the actual optimization level
#if defined ( __CC_ARM )
	#pragma push
	#pragma O0
#elif defined ( __GNUC__ )
	#pragma GCC push_options
	__attribute__ ((optimize(0)))
#endif
void Delay10us(uint32_t Num10Microseconds)
{
	// Make first 10µs exactly timed
	for(int i = 0; i < 21; i++)
		__NOP();
	Num10Microseconds--;
	
	//Add further 10µs if desired, numbers found by measurements
	for(;Num10Microseconds > 0; Num10Microseconds--)
	{
		for(int i = 0; i < 10; i++)
		{
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		}
	}
}
#if defined ( __CC_ARM )
	#pragma pop
#elif defined ( __GNUC__ )
	#pragma GCC pop_options
#endif

/**********************************************************************************
Uart_Init - Sets PP0 as Uart-Rx and PP1 as Uart-Tx and configures baudrate- and timeout-
						timer values for typical condition.
Prerequisite: -
Input: 	uint8_t Config: Configuration-Byte
												Config[0]: 0 = Use 12Mhz for baudrate-generation, 1 = Use Xtal for baudrate-generation
				enum BaudrateSel_Option: Desired Uart baudrate. For possible values see: enum BaudrateSel_Options. 
Return: -
***********************************************************************************/
void Uart_Init(uint8_t Config, enum BaudrateSel_Options BaudrateSel_Option)
{
	// Configure Pins
	Wakeup_Controller->GPIO_b.PPD0 = 1;	// PP0 is input
	Wakeup_Controller->GPIO_b.PPD1 = 0;	// PP1 is output
	Wakeup_Controller->GPIO_b.PPO0 = 0;	// No pull-up resistor on PP0
	Wakeup_Controller->GPIO_b.PPO1 = 1;	// Output high on PP1
	
	// Save Config and set correct timer-value for chosen baudrate
	Uart_Config_Loc = Config;
	Uart_BaudrateSel_Option_Loc = BaudrateSel_Option;
	if((Uart_Config_Loc & UART_CONFIG_REFCLK_MASK) == UART_CONFIG_REFCLK_HPRC)	// HPRC/4 will be used for BR-clock, HPRC/4 will be used for timeout
	{
		switch(BaudrateSel_Option)
		{
			case BR_19k2:
				Uart_BRTimer_ReloadValue_Loc = (uint8_t)((float)(12000000.0 / 4.0 / 14400.0) + 0.5);
				Uart_TimeoutTimer_ReloadValue_Loc = (uint16_t)(((float)(12000000.0 / 4.0 / 14400.0 * 12) + 0.5));
				break;
			case BR_57k6:
				Uart_BRTimer_ReloadValue_Loc = (uint8_t)((float)(12000000.0 / 4.0 / 57600.0) + 0.5);
				Uart_TimeoutTimer_ReloadValue_Loc = (uint16_t)(((float)(12000000.0 / 4.0 / 57600.0 * 12) + 0.5));
				break;
			case BR_96k:
				Uart_BRTimer_ReloadValue_Loc = (uint8_t)((float)(12000000.0 / 4.0 / 96000.0) + 0.5);
				Uart_TimeoutTimer_ReloadValue_Loc = (uint16_t)(((float)(12000000.0 / 4.0 / 96000.0 * 12) + 0.5));
				break;
			default:
				Uart_BRTimer_ReloadValue_Loc = 0;
				Uart_TimeoutTimer_ReloadValue_Loc = 0;
		}
	}
	else if((Uart_Config_Loc & UART_CONFIG_REFCLK_MASK) == UART_CONFIG_REFCLK_XTAL)	// XTAL/8 will be used for BR-clock, HPRC/4 will be used for timeout
	{
		switch(BaudrateSel_Option)
		{
			case BR_19k2:
				Uart_BRTimer_ReloadValue_Loc = (uint8_t)((float)(26000000.0 / 8.0 / 14400.0) + 0.5);
				Uart_TimeoutTimer_ReloadValue_Loc = (uint16_t)(((float)(12000000.0 / 4.0 / 14400.0 * 12) + 0.5));
				break;
			case BR_57k6:
				Uart_BRTimer_ReloadValue_Loc = (uint8_t)((float)(26000000.0 / 8.0 / 57600.0) + 0.5);
				Uart_TimeoutTimer_ReloadValue_Loc = (uint16_t)(((float)(12000000.0 / 4.0 / 57600.0 * 12) + 0.5));
				break;
			case BR_96k:
				Uart_BRTimer_ReloadValue_Loc = (uint8_t)((float)(26000000.0 / 8.0 / 96000.0) + 0.5);
				Uart_TimeoutTimer_ReloadValue_Loc = (uint16_t)(((float)(12000000.0 / 4.0 / 96000.0 * 12) + 0.5));
				break;
			default:
				Uart_BRTimer_ReloadValue_Loc = 0;
				Uart_TimeoutTimer_ReloadValue_Loc = 0;
		}
	}
}

/**********************************************************************************
Uart_Baudrate_Calib - If HPRC is used as baudrate-clock, this function might be required to be called for precise baudrate.
											E.g. when device is at extreme temperatures, the HPRC might deviate too much from ideal frequency for exact baudrate.

Prerequisite: Uart must be initialized via Uart_Init. Xtal must have been started.
Input: 	-
Return: uint8_t Status: 0 = okay, 1 = timeout, 2 = out of range
***********************************************************************************/
uint8_t Uart_Baudrate_Calib(void)
{
	uint8_t status = UART_STATUS_OK;
	uint8_t RESUMEMASK_bak = System_Controller->RESUMEMASK;
	
	// Calibration only needed when HPRC shall be used as BR-clock
	if((Uart_Config_Loc & UART_CONFIG_REFCLK_MASK) == UART_CONFIG_REFCLK_HPRC)
	{
		// User Timer0/1 in mode 7 for calibration. T0CLK = HPRC/4, T0DIVSEL = 0 (divide by 1). T1CLK = XTAL/8, T1DIVSEL = 0 (divide by 1)
		Corelogic->TIMERCFG01 = 0;
		Corelogic->TIMERCFG01 |= Corelogic_TIMERCFG01_T01EN_Msk;		
		Corelogic->TIMERCFG01_b.T01MODE = 7;
		Corelogic->TIMERCFG01_b.T0CLKSEL = 4;
		Corelogic->TIMERCFG01_b.T0DIVSEL = 0;
		Corelogic->TIMERCFG01_b.T1CLKSEL = 1;
		Corelogic->TIMERCFG01_b.T1DIVSEL = 0;
		// Preload timer1 with 0xFFFF, preload timer0 with 128 (--> Count XTAL/8 during 128 HPRC/4-periods)
		Corelogic->TIMER01_b.T0H = 0;
		Corelogic->TIMER01_b.T0L = 128-1;
		Corelogic->TIMER01_b.T1H = 0xFF;
		Corelogic->TIMER01_b.T1L = 0xFF;
		// Run timers
		Corelogic->TIMERCFG01 |= (Corelogic_TIMERCFG01_T0RUN_Msk | Corelogic_TIMERCFG01_T1RUN_Msk);
		
		// Enable resume by timer0, go to IDLE and wait for TIMER0 to resume
		System_Controller->RESUMEMASK = ~System_Controller_RESUMEMASK_RET0M_Msk;
		System_Controller->RESUMEFLAGS = 0xFFFFFFFF;
		Wakeup_Controller->DEVSTATUS = 0xFFFFFFFF;
		System_Controller->SYSCCTRL_b.IDLE = 1;
		while(System_Controller->RESUMEFLAGS_b.RET0 == 0);
		
		// Stop timers, read reasult from T1-value
		Corelogic->TIMERCFG01 &= ~(Corelogic_TIMERCFG01_T0RUN_Msk | Corelogic_TIMERCFG01_T1RUN_Msk);
		uint16_t result = ((uint16_t)0xFFFF - (uint16_t)((Corelogic->TIMER01 & (Corelogic_TIMER01_T1H_Msk | Corelogic_TIMER01_T1L_Msk)) >> Corelogic_TIMER01_T1L_Pos));
	
		// result should be roughly within [110 ... 185]
		if(result < 110 || result > 185)
			status = UART_STATUS_OUTOFRANGE;
		
		if(status == UART_STATUS_OK)
		{
			// result = 128/f_HPRC*4*f_XTAL/8 
			// --> f_HPRC = 128*4*f_XTAL/8/result
			// BRTimer_value = f_HPRC / 4.0 / f_Baudrate
			// BRTimer_value = 128*f_XTAL/8/result/f_Baudrate
			// BRTimer_value = 416000000/(result*f_Baudrate)
			uint32_t Divider = result;
			switch(Uart_BaudrateSel_Option_Loc)
			{
				case BR_19k2:
					Divider *= 14400;
					break;
				case BR_57k6:
					Divider *= 57600;
					break;
				case BR_96k:
					Divider *= 96000;
					break;
				default:
					status = UART_STATUS_OUTOFRANGE;
			}
			Uart_BRTimer_ReloadValue_Loc = (416000000 + (Divider>>1))/Divider;
		}
	}
	
	// Restore RESUMEMASK, stop and disable timers and return status
	System_Controller->RESUMEMASK = RESUMEMASK_bak;
	Corelogic->TIMERCFG01 = 0;
	return status;
}

/**********************************************************************************
Uart_Putchar - Transmits one byte using PP0 as Uart-Rx and PP1 as Uart-Tx at a fixed baudrate of 19.2kBaud

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint8_t Byte: The byte that shall be transmitted
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
#pragma push
#pragma O3
uint8_t Uart_Putchar(uint8_t Byte)
{
	uint8_t status = UART_STATUS_OK;
	uint8_t RESUMEMASK_bak = System_Controller->RESUMEMASK;
	
	// HPRC/4 will be used for BR-clock, HPRC/4 will be used for timeout
	if((Uart_Config_Loc & UART_CONFIG_REFCLK_MASK) == UART_CONFIG_REFCLK_HPRC)	
	{
		//Select Timer-Mode5 (8-bit timer with reload and 16-bit timer without reload), Timer0 is used for baudrate and uses clksel = HPRC / 4. Timer1 is used for timeout and uses also HPRC / 4
		Corelogic->TIMERCFG01 = ((0x4 << Corelogic_TIMERCFG01_T0CLKSEL_Pos) | (0x0 << Corelogic_TIMERCFG01_T0DIVSEL_Pos) | (0x4 << Corelogic_TIMERCFG01_T1CLKSEL_Pos) | (0x0 << Corelogic_TIMERCFG01_T1DIVSEL_Pos) | (0x5 << Corelogic_TIMERCFG01_T01MODE_Pos) | (Corelogic_TIMERCFG01_T01EN_Msk));
	}
	// XTAL/8 will be used for BR-clock, HPRC/4 will be used for timeout
	else if((Uart_Config_Loc & UART_CONFIG_REFCLK_MASK) == UART_CONFIG_REFCLK_XTAL)	
	{
		//Select Timer-Mode5 (8-bit timer with reload and 16-bit timer without reload), Timer0 is used for baudrate and uses clksel = XTAL / 8. Timer1 is used for timeout and uses HPRC / 4
		Corelogic->TIMERCFG01 = ((0x1 << Corelogic_TIMERCFG01_T0CLKSEL_Pos) | (0x0 << Corelogic_TIMERCFG01_T0DIVSEL_Pos) | (0x4 << Corelogic_TIMERCFG01_T1CLKSEL_Pos) | (0x0 << Corelogic_TIMERCFG01_T1DIVSEL_Pos) | (0x5 << Corelogic_TIMERCFG01_T01MODE_Pos) | (Corelogic_TIMERCFG01_T01EN_Msk));
	}
	//Load Timer-values appropriate for chosen baudrate into T0. Load timeout-value (12 * tBit) into T1
	Corelogic->TIMER01 = (((uint32_t)(Uart_BRTimer_ReloadValue_Loc) << Corelogic_TIMER01_T0L_Pos) | ((uint32_t)(Uart_BRTimer_ReloadValue_Loc) << Corelogic_TIMER01_T0H_Pos));
	Corelogic->TIMER01 |= ((uint32_t)(Uart_TimeoutTimer_ReloadValue_Loc) << Corelogic_TIMER01_T1L_Pos);
	
	// Save GPIO-value
	uint32_t GPIO_value = Wakeup_Controller->GPIO;
	uint32_t GPIO_value_PP1_Low = GPIO_value & ~(Wakeup_Controller_GPIO_PPO1_Msk);
	uint32_t GPIO_value_PP1_High = GPIO_value | Wakeup_Controller_GPIO_PPO1_Msk;
	
	// START Bit
	System_Controller->RESUMEMASK = ~(System_Controller_RESUMEMASK_RET0M_Msk | System_Controller_RESUMEMASK_RET1M_Msk);	// Enable resume from timers
	Wakeup_Controller->GPIO = GPIO_value_PP1_Low; // Set Tx-Pin to 0
	Corelogic->TIMERCFG01_b.T1RUN = 1;			// Start Timer1 (Timeout-timer)
	Corelogic->TIMERCFG01_b.T0RUN = 1;			// Start Timer0 (Baudrate-timer)
	System_Controller->RESUMEFLAGS = 0x0F;	// Write-clear all resume-flags
	Corelogic->COLFLAGS = (Corelogic_COLFLAGS_T0FULL_Msk | Corelogic_COLFLAGS_T1FULL_Msk);	// Write-clear T0FUL- and T1FUL-Bit
	System_Controller->SYSCCTRL_b.IDLE = 1;	// Wait in IDLE until timer elapsed
	while((Corelogic->COLFLAGS & (Corelogic_COLFLAGS_T0FULL_Msk | Corelogic_COLFLAGS_T1FULL_Msk)) == 0);	// Ensure that really T0 or T1 resumed from IDLE
	
	// DATA Bit [7:0]			   
	for(uint8_t i = 8; i > 0; i--)
	{
		if(Byte & 0x01)
			Wakeup_Controller->GPIO = GPIO_value_PP1_High;	// Set Tx-Pin to 1
		else
			Wakeup_Controller->GPIO = GPIO_value_PP1_Low;		// Set Tx-Pin to 0
		Byte >>= 1;															// Shift Result one bit left to prepare next bit
		System_Controller->RESUMEFLAGS = 0x0F;	// Write-clear all resume-flags
		Corelogic->COLFLAGS = (Corelogic_COLFLAGS_T0FULL_Msk);	// Write-clear T0FULL flag
		System_Controller->SYSCCTRL_b.IDLE = 1;	// Wait in IDLE until timer elapsed
		while((Corelogic->COLFLAGS & (Corelogic_COLFLAGS_T0FULL_Msk | Corelogic_COLFLAGS_T1FULL_Msk)) == 0);	// Ensure that really T0 or T1 resumed from IDLE
	}
	
	// STOP Bit
	Wakeup_Controller->GPIO = GPIO_value_PP1_High;		// Set Tx-Pin to 1
	System_Controller->RESUMEFLAGS = 0x0F;	// Write-clear all resume-flags
	Corelogic->COLFLAGS = (Corelogic_COLFLAGS_T0FULL_Msk);	// Write-clear T0FULL flag
	System_Controller->SYSCCTRL_b.IDLE = 1;	// Wait in IDLE until timer elapsed
	while((Corelogic->COLFLAGS & (Corelogic_COLFLAGS_T0FULL_Msk | Corelogic_COLFLAGS_T1FULL_Msk)) == 0);	// Ensure that really T0 or T1 resumed from IDLE
	
	// Timeout occured
	if(Corelogic->COLFLAGS_b.T1FULL != 0)
		status = UART_STATUS_TIMEOUT;
	
	//Stop timers, write-clear timer-full flags and then return status
	Corelogic->TIMERCFG01 = 0;
	Corelogic->COLFLAGS = (Corelogic_COLFLAGS_T0FULL_Msk | Corelogic_COLFLAGS_T1FULL_Msk);
	System_Controller->RESUMEMASK = RESUMEMASK_bak;	// Restore RESUMEMASK
	return status;
}
#pragma pop

/**********************************************************************************
fputc - Overload function required for printf, sending one character

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	int ch: The character to be transmitted
				FILE *f: Not used
Return: int ch: the character which was sent out
***********************************************************************************/
int fputc(int ch, FILE *f)
{
  Uart_Putchar(ch);
  return(ch);
}

/**********************************************************************************
Uart_SendString - Transmits a given string via Uart.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	sint8_t* String: Pointer to the string that shall be transmitted
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendString(uint8_t* String)
{
	uint8_t status = 0;
	while(*String != 0x00)
	{
		status = Uart_Putchar(*String);
		String++;
		if(status != 0)
			break;
	}
	return status;
}

/**********************************************************************************
Uart_SendHex32 - Transmits a given 32bit-value in ASCII-format as hex-value. The '0x' prefix can be added on demand.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in hex-format
				uint8_t Config: Configuration-byte
												Config[7]: 0 = Do not send '0x' prefix, 1 = Send '0x' prefix
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendHex32(uint32_t Value, uint8_t Config)
{
	uint8_t status = 0;
	uint8_t Nibble;
	
	//Send '0x' prefix if desired
	if((Config & UART_SENDHEX_PREFIX_MASK) == UART_SENDHEX_PREFIX_ENABLED)
		status = Uart_SendString((uint8_t *)"0x");
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the first nibble
	Nibble = (uint8_t)((Value >> 28) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 24) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 20) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 16) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 12) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 8) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 4) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the last nibble
	Nibble = (uint8_t)((Value) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	
	return status;
}

/**********************************************************************************
Uart_SendHex16 - Transmits a given 16bit-value in ASCII-format as hex-value. The '0x' prefix can be added on demand.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in hex-format
				uint8_t Config: Configuration-byte
												Config[7]: 0 = Do not send '0x' prefix, 1 = Send '0x' prefix
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendHex16(uint16_t Value, uint8_t Config)
{
	uint8_t status = 0;
	uint8_t Nibble;
	
	//Send '0x' prefix if desired
	if((Config & UART_SENDHEX_PREFIX_MASK) == UART_SENDHEX_PREFIX_ENABLED)
		status = Uart_SendString((uint8_t *)"0x");
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 12) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 8) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 4) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the last nibble
	Nibble = (uint8_t)((Value) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	
	return status;
}

/**********************************************************************************
Uart_SendHex8 - Transmits a given 8bit-value in ASCII-format as hex-value. The '0x' prefix can be added on demand.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in hex-format
				uint8_t Config: Configuration-byte
												Config[7]: 0 = Do not send '0x' prefix, 1 = Send '0x' prefix
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendHex8(uint8_t Value, uint8_t Config)
{
	uint8_t status = 0;
	uint8_t Nibble;
	
	//Send '0x' prefix if desired
	if((Config & UART_SENDHEX_PREFIX_MASK) == UART_SENDHEX_PREFIX_ENABLED)
		status = Uart_SendString((uint8_t *)"0x");
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the next nibble
	Nibble = (uint8_t)((Value >> 4) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	if(status != UART_STATUS_OK)
		return status;
	
	//Convert and send the last nibble
	Nibble = (uint8_t)((Value) & 0xF) + 0x30;
	if (Nibble > 0x39)
		Nibble += 7;
	status = Uart_Putchar(Nibble);
	
	return status;
}

/**********************************************************************************
Uart_SendDecimalSigned - Transmits a given signed 32bit-value in ASCII-format as decimal-value using fixed baudrate of 19.2kBaud.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case XTAL is chosen as ref-clock, Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in decimal-format
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendDecimalSigned(int32_t Value)
{
	uint32_t Divider = 1000000000;
	uint8_t AlreadyWritten = 0;
	uint8_t Status = UART_STATUS_OK;
	
	if(Value < 0)
	{
		Uart_Putchar('-');
		Value *= -1;
	}

	for(; Divider > 0; Divider /= 10)
	{
		uint32_t Tmp = Value / Divider;
		if (Tmp > 0 || AlreadyWritten == 1 || Divider == 1)
		{ 
			Status = Uart_Putchar(Tmp+48);
			if(Status != UART_STATUS_OK)
				return Status;
			AlreadyWritten = 1;
		}
		Value -= Tmp*Divider;
	}
	
	return Status;
} 

/**********************************************************************************
Uart_SendDecimalUnsigned - Transmits a given unsigned 32bit-value in ASCII-format as decimal-value using fixed baudrate of 19.2kBaud.

Prerequisite: GPIOs have been initialized via Uart_Init(). In case Config[0] is 1 (use Xtal), Xtal must have been started.
Input: 	uint32_t Value: Value which shall be ASCII-formatted and transmitted in decimal-format
Return: uint8_t Status: 0 = okay, 1 = timeout
***********************************************************************************/
uint8_t Uart_SendDecimalUnsigned(uint32_t Value)
{
	uint32_t Divider = 1000000000;
	uint8_t AlreadyWritten = 0;
	uint8_t Status = UART_STATUS_OK;

	for(; Divider > 0; Divider /= 10)
	{
		uint32_t Tmp = Value / Divider;
		if (Tmp > 0 || AlreadyWritten == 1 || Divider == 1)
		{ 
			Status = Uart_Putchar(Tmp+48);
			if(Status != UART_STATUS_OK)
				return Status;
			AlreadyWritten = 1;
		}
		Value -= Tmp*Divider;
	}
	
	return Status;
} 


