/*********************************************************************************************************************
 * @file     system_SP49.c
 * @brief    Device specific initialization for SP49 according to CMSIS
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
 * *************************** Change history ********************************
 * 
 * @endcond 
 */

/*******************************************************************************
 * HEADER FILES
 *******************************************************************************/
#include <SP49.h>
#include "system_SP49.h"
#include "FlConfig.h"

/**********************************************************************************
 LOCKBYTES
***********************************************************************************/
//Reservation of 16 Bits for lockbyte in each logical sector except boot-sector
#if (FL_CONFIG_NUM_UCS_SEC > 0)
uint16_t CRCLockbyteUCS1[2] __attribute__((section("UCS1_CRCLB"), used)) = {0x0000, 0x0000};		//{CRC, Lockbyte} -> Write Lockbyte to 0x6969 to lock UCS1
#endif
#if (FL_CONFIG_NUM_UCS_SEC > 1)
uint16_t CRCLockbyteUCS2[2] __attribute__((section("UCS2_CRCLB"), used)) = {0x0000, 0x0000};		//{CRC, Lockbyte} -> Write Lockbyte to 0x6969 to lock UCS2
#endif
#if (FL_CONFIG_NUM_UCS_SEC > 2)
uint16_t CRCLockbyteUCS3[2] __attribute__((section("UCS3_CRCLB"), used)) = {0x0000, 0x0000};		//{CRC, Lockbyte} -> Write Lockbyte to 0x6969 to lock UCS3
#endif
#if (FL_CONFIG_NUM_UCS_SEC > 3)
uint16_t CRCLockbyteUCS4[2] __attribute__((section("UCS4_CRCLB"), used)) = {0x0000, 0x0000};		//{CRC, Lockbyte} -> Write Lockbyte to 0x6969 to lock UCS4
#endif
#if (FL_CONFIG_NUM_UCS_SEC > 4)
uint16_t CRCLockbyteUCS5[2] __attribute__((section("UCS5_CRCLB"), used)) = {0x0000, 0x0000};		//{CRC, Lockbyte} -> Write Lockbyte to 0x6969 to lock UCS5
#endif
#if (FL_CONFIG_NUM_UCS_SEC > 5)
uint16_t CRCLockbyteUCS6[2] __attribute__((section("UCS6_CRCLB"), used)) = {0x0000, 0x0000};		//{CRC, Lockbyte} -> Write Lockbyte to 0x6969 to lock UCS6
#endif
uint16_t CRCLockbyteCode[2] __attribute__((section("CODE_CRCLB"), used)) = {0x0000, 0x0000};		//{CRC, Lockbyte} -> Write Lockbyte to 0x6969 to lock code sector

/*******************************************************************************
 * FLASH CONFIGURATION
 *******************************************************************************/
// Configure here the start-up boost behaviour: Select which pin(s) shall be checked for which level each in order to skip mode-selection
#define PP0_Mask GPIOConf_Mask_PP0Inactive
#define PP1_Mask GPIOConf_Mask_PP1Inactive
#define PP2_Mask GPIOConf_Mask_PP2Inactive
#define PP3_Mask GPIOConf_Mask_PP3Inactive
#define PP0_Ref GPIOConf_Ref_PP0Low
#define PP1_Ref GPIOConf_Ref_PP1Low
#define PP2_Ref GPIOConf_Ref_PP2Low
#define PP3_Ref GPIOConf_Ref_PP3Low

// This is the declaration of the flash-configuration-line -> don not change something here except the Protection!
const SEC_CONF_CUST_t CurrentConfiguration __attribute__((section("FLASH_CONFIG"), used)) = 
{
	.BootSect =												// Do not change here, change in FlConfig.h instead!
	{
		.Start = FL_CONFIG_BOOT_SECT_START,
		.End = FL_CONFIG_BOOT_SECT_END,
	},
	.UCSSect =												// Do not change here, change in FlConfig.h instead!
	{
		.Start = FL_CONFIG_UCS_SECT_START,
		.End = FL_CONFIG_UCS_SECT_END,
	},
	.GPIOConf =												// Define above, do not change here!
	{
		.Ref = (PP0_Ref | PP1_Ref | PP2_Ref | PP3_Ref),
		.Mask = (PP0_Mask | PP1_Mask | PP2_Mask | PP3_Mask),
	},
	.NotUsed = 0x00,								// Must be 0x00, do not change!
	.CRC16_MSB = (uint8_t)((CRC_FLASHCONFIG >> 8) & 0xFF),	// Automatically calculated, do not change!
	.CRC16_LSB = (uint8_t)(CRC_FLASHCONFIG & 0xFF),					// Automatically calculated, do not change!
	.Protection = 0x0000,		// Set to 0x0000 to deactivate device-protection, set to 0x6969 to activate device-protection
};
