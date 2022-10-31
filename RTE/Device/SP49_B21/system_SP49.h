/*********************************************************************************************************************
 * @file     system_SP49.h
 * @brief    Device specific initialization for the SP49 according to CMSIS
 * @version  V2.0
 * @date     04 August 2020
 *
 * @cond
 *********************************************************************************************************************
 * Copyright (c) 2012-2020, Infineon Technologies AG
 * All rights reserved.                        
 *                                             
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the 
 * following conditions are met:   
 *                                                                              
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following 
 * disclaimer.                        
 * 
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following 
 * disclaimer in the documentation and/or other materials provided with the distribution.                       
 * 
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote 
 * products derived from this software without specific prior written permission.                                           
 *                                                                              
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR  
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                                                                                                                                                   
 *********************************************************************************************************************
 *
 **************************** Change history *********************************
 *
 *****************************************************************************
 * @endcond 
 */

#ifndef SYSTEM_SP49_H
#define SYSTEM_SP49_H

/*******************************************************************************
 * HEADER FILES
 *******************************************************************************/
#include <stdint.h>

typedef signed char 	sint8_t;
typedef signed short int 	sint16_t;
typedef signed int 	sint32_t;

#define LIB_ROM_H		//Avoid that Lib_ROM.h is included again and does again define types and instructions

/*******************************************************************************
 * PREPROCESSOR defines
 *******************************************************************************/
// The following lines are used to automatically calculate the CRC16-CCITT for the flash-configuration line -> do not change! 
#define CRCStep1(StartVal, InByte) (uint8_t)((StartVal >> 8) ^ InByte)
#define CRCStep2(StartVal, InByte) (uint8_t)(CRCStep1(StartVal, InByte) ^ (CRCStep1(StartVal, InByte) >> 4))
#define CRC(StartVal, InByte) (uint16_t)((StartVal << 8) ^ (((uint16_t)CRCStep2(StartVal, InByte)) << 12)^(((uint16_t)CRCStep2(StartVal, InByte)) << 5)^((uint16_t)CRCStep2(StartVal, InByte)))
#define CRC_BootSect CRC(0xFFFF, (uint8_t)(FL_CONFIG_BOOT_SECT_START | (FL_CONFIG_BOOT_SECT_END << 4)))
#define CRC_UCSSect CRC(CRC_BootSect, (uint8_t)(FL_CONFIG_UCS_SECT_START | (FL_CONFIG_UCS_SECT_END << 4)))
#define CRC_GPIOConf CRC(CRC_UCSSect, (uint8_t)((PP0_Ref | PP1_Ref | PP2_Ref | PP3_Ref) | ((PP0_Mask | PP1_Mask | PP2_Mask | PP3_Mask) << 4)))
#define CRC_FLASHCONFIG CRC(CRC_GPIOConf, 0x00)


#define GPIOConf_Ref_PP0High			(0x1)
#define GPIOConf_Ref_PP0Low				(0x0)
#define GPIOConf_Ref_PP1High			(0x2)
#define GPIOConf_Ref_PP1Low				(0x0)
#define GPIOConf_Ref_PP2High			(0x4)
#define GPIOConf_Ref_PP2Low				(0x0)
#define GPIOConf_Ref_PP3High			(0x8)
#define GPIOConf_Ref_PP3Low				(0x0)
#define GPIOConf_Mask_PP0Active		(0x1)
#define GPIOConf_Mask_PP0Inactive	(0x0)
#define GPIOConf_Mask_PP1Active		(0x2)
#define GPIOConf_Mask_PP1Inactive	(0x0)
#define GPIOConf_Mask_PP2Active		(0x4)
#define GPIOConf_Mask_PP2Inactive	(0x0)
#define GPIOConf_Mask_PP3Active		(0x8)
#define GPIOConf_Mask_PP3Inactive	(0x0)

/*******************************************************************************
 * GLOBAL VARIABLES
 *******************************************************************************/
typedef struct 
{ 
	struct 
	{
		unsigned char Start  : 4;			// Describes at which logical sector the boot-sector starts. Must always be 2
		unsigned char End  : 4;				// Describes at which logical sector the boot-sector ends. Can range from BootSect.Start ... 8
	}BootSect; 
	
	struct 
	{
		unsigned char Start : 4;			// Describes at which logical sector the UCS-sector(s) start(s). Can be 0 or range from (BootSect.End+1) ... 8
		unsigned char End : 4;				// Describes at which logical sector the UCS-sector(s) end(s)s. Can range from UCSSect.Start ... 8
	}UCSSect;
	
	struct 
	{
		unsigned char Ref : 4;				// Describes which levels shall be at the respective pins during boot for the start-up boost feature
		unsigned char Mask : 4;				// Describes which pins shall be checked during boot for the start-up boost feature
	}GPIOConf;
	
	unsigned char NotUsed;					// Must be 0x00
	
	unsigned char CRC16_MSB;				// CRC16 must hold the valid CRC16(CCITT) over [BootSect, UCSSect, GPIOConf, NotUsed]
	unsigned char CRC16_LSB;
	
	unsigned char NotUsed2[24];
	
	unsigned short Protection;			// Defines whether the device is locked (0x6969) or not
}SEC_CONF_CUST_t;

/*******************************************************************************
 * API PROTOTYPES
 *******************************************************************************/

#endif
