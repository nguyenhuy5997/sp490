/**********************************************************************************
*
*	Purpose:			Sensor Measurements
*
*	Target:				SP49 B21
*
*	Toolchain:		Keil uVision5 V5.26.2.0
*
***********************************************************************************
*	Copyright (C) Infineon Technologies 2020.  All rights reserved.
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

// *** IMPORTANT INFORMATION ***//
// For configuration of flash-sectorization, check files FlConfig.h and system_SP49.c
// For lockbytes in each logical sector, check system_SP49.c

/**********************************************************************************
 Includes
***********************************************************************************/
// Must-to-have includes in any source-file
#include "SP49.h"

// Optional includes for modules, only if respective module is used
#include "Com_Fl_ext.h"
#include "Com_Math_ext.h"
#include "Com_MEIF_ext.h"
#include "Com_Misc_ext.h"
#include "Lib_Adv.h"
#include "Lib_Calib.h"
#include "Lib_Comp.h"
#include "Lib_Comp_SM.h"
#include "Lib_Diag.h"
#include "Lib_Diag_SM.h"
#include "Lib_Fl.h"
#include "Lib_LF.h"
#include "Lib_Math.h"
#include "Lib_Math_SM.h"
#include "Lib_Meas.h"
#include "Lib_Meas_SM.h"
#include "Lib_RF.h"
#include "Lib_Serv.h"
#include "Lib_State.h"
#include "Lib_State_SM.h"

// Other includes
#include "DevLib.h"
#include "Hamming.h"
#include "Common.h"
#include "math.h"
/**********************************************************************************
 Type definitions

***********************************************************************************/
// ---------------------
// Do not change!
// ---------------------
#define _ThresholdMethod_Relative			0
#define _ThresholdMethod_Absolute			1
#define _Pset_SET0								0
#define _Pset_SET1								(COM_MEIF_EXT_CFG_PSET)
#define _Pset_Auto								(COM_MEIF_EXT_CFG_AUTO_PRESS_RANGE)

#define _Acc_Direction_Positive		(0)
#define _Acc_Direction_Negative		(COM_MEIF_EXT_CFG_ACC_POS)

#define _Acc_Range_Low						(0)
#define _Acc_Range_High						(COM_MEIF_EXT_CFG_ACC_RANGE)

#define _Acc_Offset_Manual				(0)
#define _Acc_Offset_Auto					(COM_MEIF_EXT_CFG_AUTO_ACC_OFFSET)

#define _CRC_Check_Disabled				(0)
#define _CRC_Check_Enabled				(COM_MEIF_EXT_CFG_CRC_CHK)

#define _WBC_Check_Disabled				(0)
#define _WBC_Check_Enabled				(COM_MEIF_EXT_CFG_WBC)

#define _RD_Check_Disabled				(0)
#define _RD_Check_Enabled					(COM_MEIF_EXT_CFG_RD)

#define _AIN_Pin_PP0							(0)
#define _AIN_Pin_PP3							(2)

#define BITSIZE(n) ((n) >= 128 ? 7 \
                  : (n) >=  64 ? 6 \
                  : (n) >=  32 ? 5 \
                  : (n) >=  16 ? 4 \
                  : (n) >=   8 ? 3 \
                  : (n) >=   4 ? 2 \
                  : (n) >=   2 ? 1 \
                  : 0)								

// -------------------------------------
// User configurable meas-comp-settings
// -------------------------------------
#define MeasComp_Temperature_CRCCheckEnable	_CRC_Check_Enabled			// Enable or Disable the CRC-check for temperature-MeasComp

#define Meas_RawPressure_NumSamples					(4)											// Define here how many RAW-samples shall be averaged for pressure
#define Meas_RawPressure_CRCCheckEnable			_CRC_Check_Enabled			// Enable or Disable the CRC-check for pressure-measurement
#define Meas_RawPressure_WBCCheckEnable			_WBC_Check_Enabled			// Enable or Disable the WBC-check for pressure-measurement
#define Comp_Pressure_DesiredPSet						_Pset_Auto							// Choose here either _Pset_Auto, or any of the _Pset_SET0 ... _Pset_SET3
#define Comp_Pressure_CRCCheckEnable				_CRC_Check_Enabled			// Enable or Disable the CRC-check for pressure-compensation

#define Meas_RawAcceleration_NumSamples			(16)										// Define here how many RAW-samples shall be averaged for acceleration
#define Meas_RawAcceleration_CRCCheckEnable	_CRC_Check_Enabled			// Enable or Disable the CRC-check for acceleration-measurement
#define Meas_RawAcceleration_WBCCheckEnable	_WBC_Check_Enabled			// Enable or Disable the WBC-check for acceleration-measurement
#define Meas_RawAcceleration_RDCheckEnable	_RD_Check_Enabled				// Enable or Disable the RD-check for acceleration-measurement
#define MeasComp_Acceleration_Direction			_Acc_Direction_Positive	// Choose here either positive or negative direction for acceleration sensor
#define MeasComp_Acceleration_Range					_Acc_Range_Low					// Choose here either low or high range for acceleration sensor
#define Comp_Acceleration_CRCCheckEnable		_CRC_Check_Enabled			// Enable or Disable the CRC-check for acceleration-compensation
#define Comp_Acceleration_Offset_Source			_Acc_Offset_Auto				// Choose here either manual (via argument) or automatic offser for Acc-compensation (recommended)
#define Comp_Acceleration_Offset_Arg				(0)											// Enter here the offset (only valid for _Acc_Offset_Manual) in compensated LSB [1/16g]

#define MeasComp_SupplyV_CRCCheckEnable			(_CRC_Check_Enabled)		// Enable or Disable the CRC-check for supply-voltage-MeasComp

#define MeasComp_AIN_CRCCheckEnable					(_CRC_Check_Enabled)		// Enable or Disable the CRC-check for supply-voltage-MeasComp
#define MeasComp_AIN_Pin										(_AIN_Pin_PP0)					// Choose here either PP0 or PP3 as input for AIN-MeasComp

#define DesiredAutoAccOffsetBlocksize (50)													// Define the number of samples for the auto-acc-offset function for offset calculation
#define DesiredAutoAccOffsetMovingThreshold_LSBsqr	(4)							// Define the threshold for moving/drive decision for the auto-acc-offset function in [LSB^2]
#define StartXtal_Delay100us		(5)		// Configure here the delay when starting the crystal, in [100us]
// -------------------------------
// User configurable LPM-settings
// -------------------------------
// Configure (LPM-)timings here
#define DesiredPeriod100ms_IntervalWakup	(2999)
#define DesiredPeriod100ms_LPMPMeas				(119)
#define DesiredPeriod100ms_LPMAMeas				(120)
#define DesiredPeriod100ms_LPMTMeas				(121)

// Configure LPM-priorities here
#define DesiredPriority_LPMPMeas					(0)				// 0 = highest, 1 = medium, 2 = lowest priority. Each priority may only be defined once!
#define DesiredPriority_LPMAMeas					(1)				// 0 = highest, 1 = medium, 2 = lowest priority. Each priority may only be defined once!
#define DesiredPriority_LPMTMeas					(2)				// 0 = highest, 1 = medium, 2 = lowest priority. Each priority may only be defined once!

#if (DesiredPriority_LPMPMeas == DesiredPriority_LPMAMeas) || (DesiredPriority_LPMPMeas == DesiredPriority_LPMTMeas) || (DesiredPriority_LPMAMeas == DesiredPriority_LPMTMeas)
	#error "Each priority may only be defined once!"
#endif

// Configure thresholds here
#define DesiredThresholdMethod_LPMPMeas		_ThresholdMethod_Absolute		// Choose here the threshold method for LPM-pressure (relative or absolute)
#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Relative)
	#define DesiredThreshold_LPMPMeas_Positive		(5*2)		// In case of relative threshold, the threshold is relative to last measured pressure, in [LSB] (RAW-P makes ~2LSB/kPa). E.g.: Threshold = 5*2 --> ~5kPa relative to measured-pressure after reset
	#define DesiredThreshold_LPMPMeas_Negative		(-5*2)		
#endif
#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Absolute)
	#define DesiredThreshold_LPMPMeas_Positive		(105)		// In case of absolute threshold, the threshold here is in [kPa]
	#define DesiredThreshold_LPMPMeas_Negative		(95)		
#endif

#define DesiredThresholdMethod_LPMAMeas		_ThresholdMethod_Absolute		// Choose here the threshold method for LPM-acceleration (relative or absolute)
#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Relative)
	#define DesiredThreshold_LPMAMeas_Positive		(1*13)		// In case of relative threshold, the threshold is relative to last measured acceleration, in [LSB] (RAW-A makes ~13LSB/g). E.g.: Threshold = 2*13 --> ~2g relative to measured-acceleration after reset
	#define DesiredThreshold_LPMAMeas_Negative		(-1*13)		
#endif
#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Absolute)
	#define DesiredThreshold_LPMAMeas_Positive		(1)		// In case of absolute threshold, the threshold here is in [g]
	#define DesiredThreshold_LPMAMeas_Negative		(-1)		
#endif

#define DesiredThreshold_LPMTMeas_Positive		(1*128)		// The threshold for temperature is always in [LSB] relative to last measured temperature. RAW-T makes 128LSB/°C). E.g.: Threshold = 2*128 --> 2°C relative to last measured temperature
#define DesiredThreshold_LPMTMeas_Negative		(-1*128)
// ---------------------
// Error numbers
// ---------------------
#define ErrorNumber_CrystalNotStarted		1
#define ErrorNumber_RCOscCheckFailed		2
#define ErrorNumber_CrystalNotStopped		3
#define ErrorNumber_UnknownReset				4
#define ErrorNumber_UartError						4
#define ErrorNumber_UnknownWakeup 			5
#define ErrorNumber_TMeasCompError			6
#define ErrorNumber_PMeasError					7
#define ErrorNumber_PCompError					8
#define ErrorNumber_AMeasError					9
#define ErrorNumber_ACompError					10
#define ErrorNumber_VMeasCompError			11
#define ErrorNumber_LibCalibPrecountFailed 	12
#define ErrorNumber_LibServInitITFailed			13
#define ErrorNumber_AinMeasError						14
#define ErrorNumber_PinConfigError					15

#define HEX_STR_FORMAT		1
#define	WU_IT			 				1			// wake up source is interval timer
#define	WU_EXT		 				0			// wake up source is PP2 (pull down)
#define MAX_VOL_PWR_SUP				3300			

#if 0
#define		Uart_SendString(a)  __NOP
#define		Uart_SendHex32(a,b) __NOP
#define		Uart_SendHex16(a,b) __NOP
#define		Uart_SendHex8(a,b) __NOP
#define		Uart_SendDecimalSigned(a) __NOP
#define		Uart_SendDecimalUnsigned(a) __NOP
#endif
/**********************************************************************************
 Global Variables
 
***********************************************************************************/

uint32_t frame_cnt __attribute__((section("RETRAM"), zero_init));	
/**********************************************************************************
 Function Prototypes
***********************************************************************************/
void ErrorHandling(int ErrorNumber);
void MeasCompPrint_Temperature(Com_MEIF_Ext_Meas_t* MeasResults, VAL_FRAME_t* SensorValuePacked);
void MeasCompPrint_Pressure(Com_MEIF_Ext_Meas_t* MeasResults, uint8_t UseTempMeasFromExternal, VAL_FRAME_t* SensorValuePacked);
void MeasCompPrint_Acceleration(Com_MEIF_Ext_Meas_t* MeasResults, uint8_t UseTempMeasFromExternal, VAL_FRAME_t* SensorValuePacked);
void MeasCompPrint_SupplyVoltage(Com_MEIF_Ext_Meas_t* MeasResults, VAL_FRAME_t* SensorValuePacked);
void MeasCompPrint_AIN(Com_MEIF_Ext_Meas_t* MeasResults, VAL_FRAME_t* SensorValuePacked);
void EarlyRollingMode_GenerateAPSTrial_SendRF(Com_MEIF_Ext_Meas_t* MeasResults, uint8_t UseTempMeasFromExternal);
void InitI2C();
/**********************************************************************************
 Function Implementations
***********************************************************************************/
volatile uint32_t triggered_wakeup_mask;
volatile uint32_t triggered_wakeup;
int main(void)
{		
	int Retry_I2C = 0;
	VAL_FRAME_t SensorValuePacked;
	Wakeup_Controller->GPIO_b.PPD2 = 0;
	Wakeup_Controller->GPIO_b.PPO2 = 0;
	Com_MEIF_Ext_Meas_t MeasResults;
	// A wakeup from powerdown occured
	if(Wakeup_Controller->DEVSTATUS_b.WUPDWN != 0)
	{
		// Check the actual wakeup-source. Wakeup-flags can be set, but only if they are not masked, they can generate a wakeup.
		//	For this purpose we calculate a mask to be applied to the flags in DEVSTATUS:
		//			-All mask-bits from EXT_MASK (Bit 9) to CD_MASK (Bit0) --> invert and use Bits9 ... Bit0 (0x03FF) in DEVCTRL
		//			-ITIM cannot be masked in normal mode --> OR with ITIM_MASK_Msk
		//			-LPM-TRIGGER flags are only set, if their mask-bit in DEVCTRL is 0 --> No need to consider DEVCTRL.LPMx_MASK, just OR with DEVSTATUS.LPMx_FLAG_Msk
		triggered_wakeup_mask = ((~(Wakeup_Controller->DEVCTRL) & 0x03FF) | Wakeup_Controller_DEVCTRL_ITIM_MASK_Msk | Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk | Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk | Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk);
		triggered_wakeup = (Wakeup_Controller->DEVSTATUS & (triggered_wakeup_mask));
		// LF-Carrier was detected
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_CD_FLAG_Msk)
		{
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_CD_FLAG_Msk;	// Write-clear CD_FLAG
		}
		// LF-Telegram(-start) was detected
		if(triggered_wakeup & (Wakeup_Controller_DEVSTATUS_EOM_FLAG_Msk | Wakeup_Controller_DEVSTATUS_BF_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM3_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM2_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM1_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM0_FLAG_Msk | Wakeup_Controller_DEVSTATUS_SYNC_FLAG_Msk))
		{
			// LF-EOM was detected
			if(Wakeup_Controller->DEVSTATUS_b.EOM_FLAG != 0)
			{
			}
			// LF-Buffer is full
			if(Wakeup_Controller->DEVSTATUS_b.BF_FLAG != 0)
			{
			}
			// LF-Pattern-Match0 detected
			if(Wakeup_Controller->DEVSTATUS_b.PM0_FLAG != 0)
			{
			}
			// LF-Pattern-Match1 detected
			if(Wakeup_Controller->DEVSTATUS_b.PM1_FLAG != 0)
			{
			}
			// LF-Pattern-Match2 detected
			if(Wakeup_Controller->DEVSTATUS_b.PM2_FLAG != 0)
			{
			}
			// LF-Pattern-Match3 detected
			if(Wakeup_Controller->DEVSTATUS_b.PM3_FLAG != 0)
			{
			}
			// LF-Sync detected
			if(Wakeup_Controller->DEVSTATUS_b.SYNC_FLAG != 0)
			{	
			}
			// Write-clear EOM_FLAG, BF_FLAG, PMx_FLAG and SYNC_FLAG	
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_EOM_FLAG_Msk | Wakeup_Controller_DEVSTATUS_BF_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM3_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM2_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM1_FLAG_Msk | Wakeup_Controller_DEVSTATUS_PM0_FLAG_Msk | Wakeup_Controller_DEVSTATUS_SYNC_FLAG_Msk);	
		}
		// Wakeup by Interval-timer
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_ITIM_FLAG_Msk)
		{		
			
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_ITIM_FLAG_Msk;
			
			// Initialize Uart-Pins
			Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);
			uint32_t status;
			uint8_t UartStatus;
			status = Lib_Serv_Start_Xtal_Osc(COM_MISC_EXT_OSC_CFG_XTAL_BOOST, 20);
			if(status != 0)
					ErrorHandling(ErrorNumber_CrystalNotStarted);
			UartStatus = Uart_Baudrate_Calib();
			if(UartStatus != 0)
					ErrorHandling(ErrorNumber_UartError);
			status = Lib_Serv_Stop_Xtal_Osc();
			if(status != 0)
					ErrorHandling(ErrorNumber_CrystalNotStopped);
			// Send start-message via Uart
//			printf("Measurement results:\r\n");
			
			// Measure, compensate and send temperature via Uart
			MeasCompPrint_Temperature(&MeasResults, &SensorValuePacked);
			if(Wakeup_Controller->DEVSTATUS_b.LPMT_FLAG)
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk;		// Clear LPM-T pending-flag, because measurement was done above
			// Measure, compensate and send Pressue via Uart
			MeasCompPrint_Pressure(&MeasResults, 1, &SensorValuePacked);
			if(Wakeup_Controller->DEVSTATUS_b.LPMP_FLAG)
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk;		// Clear LPM-P pending-flag, because measurement was done above
			
			// Measure, compensate and send Acceleration via Uart
			MeasCompPrint_Acceleration(&MeasResults, 1, &SensorValuePacked);
			if(Wakeup_Controller->DEVSTATUS_b.LPMA_FLAG)
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk;		// Clear LPM-A pending-flag, because measurement was done above
			
			// Measure, compensate and send Voltage via Uart
			MeasCompPrint_SupplyVoltage(&MeasResults, &SensorValuePacked);
		}
		// Wakeup by external
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_EXT_FLAG_Msk)
		{
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_EXT_FLAG_Msk;	// Write-clear EXT_FLAG
		}
		// Wakeup by LPM-P -> Pressure outside thresholds
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk)
		{
						#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Relative)
				uint16_t LowLimit, HighLimit;
			#endif
			
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk);	// Write-clear LPMP_FLAG
			 
			// Send message
			Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);
			printf("-->Wakeup by pressure out of limits\r\n");
			MeasCompPrint_Temperature(&MeasResults, &SensorValuePacked);
			// Measure and print current pressure
			MeasCompPrint_Pressure(&MeasResults, 1, &SensorValuePacked);
			
			// Adjust pressure-limits in case of relative
			#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Relative)
				LowLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMPMeas_Negative);
				HighLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMPMeas_Positive);
				Lib_State_LPM_Config((uint16_t)((0 << LIB_STATE_LPM_CFG_SRC_POS) | (DesiredPriority_LPMPMeas << LIB_STATE_LPM_CFG_PRIO_POS) | (1 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMPMeas, (sint16_t)(HighLimit), (sint16_t)(LowLimit));
				Uart_SendString((uint8_t *)"    Pressure-thresholds adjusted:\r\n");	
				Uart_SendString((uint8_t *)"     LPMP-Low-Limit: ");
				Uart_SendHex16(LowLimit, UART_SENDHEX_PREFIX_ENABLED);
				Uart_SendString((uint8_t *)"\r\n     LPMP-High-Limit: ");
				Uart_SendHex16(HighLimit, UART_SENDHEX_PREFIX_ENABLED);
				Uart_SendString((uint8_t *)"\r\n");
			#endif
			// Measure, compensate and send Acceleration via Uart
				MeasCompPrint_Acceleration(&MeasResults, 1, &SensorValuePacked);
			// Measure, compensate and send Voltage via Uart
				MeasCompPrint_SupplyVoltage(&MeasResults, &SensorValuePacked);
				Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk);	// Write-clear LPMP_FLAG
		}
		// Wakeup by LPM-T -> Temperature outside thresholds
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk)
		{
						uint16_t LowLimit, HighLimit;
			
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk);	// Write-clear LPMT_FLAG
			
			// Send message
			Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);
			printf("-->Wakeup by temperature out of limits\r\n");
			
			// Measure and print current temperature, adjust temperature limits
			MeasCompPrint_Temperature(&MeasResults, &SensorValuePacked);
			LowLimit = MeasResults.Scaled_Temp_Value + DesiredThreshold_LPMTMeas_Negative;
			HighLimit = MeasResults.Scaled_Temp_Value + DesiredThreshold_LPMTMeas_Positive;
			Lib_State_LPM_Config((uint16_t)((2 << LIB_STATE_LPM_CFG_SRC_POS) | (DesiredPriority_LPMTMeas << LIB_STATE_LPM_CFG_PRIO_POS) | (0 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMTMeas, (sint16_t)(HighLimit), (sint16_t)(LowLimit));		
			printf("Temperature-thresholds adjusted:\r\n");
			printf("     LPMT-Low-Limit: 0x%x\r\n", LowLimit);
			printf("     LPMT-High-Limit: 0x%x\r\n", HighLimit);
			
			// Adjust pressure-limits
			MeasCompPrint_Pressure(&MeasResults, 1, &SensorValuePacked);
			#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Relative)
				LowLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMPMeas_Negative);
				HighLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMPMeas_Positive);
			#endif
			#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Absolute)			
				MeasResults.Comp_Value = DesiredThreshold_LPMPMeas_Negative*16;
				Lib_Comp_Reverse_Pressure((uint32_t)(Comp_Pressure_CRCCheckEnable | Comp_Pressure_DesiredPSet | COM_MEIF_EXT_CFG_TEMP_MEAS), &MeasResults);	// temperature from external
				LowLimit = MeasResults.RAW_Value;
				MeasResults.Comp_Value = DesiredThreshold_LPMPMeas_Positive*16;
				Lib_Comp_Reverse_Pressure((uint32_t)(Comp_Pressure_CRCCheckEnable | Comp_Pressure_DesiredPSet | COM_MEIF_EXT_CFG_TEMP_MEAS), &MeasResults);	// temperature from external
				HighLimit = MeasResults.RAW_Value;
			#endif
			Lib_State_LPM_Config((uint16_t)((0 << LIB_STATE_LPM_CFG_SRC_POS) | (DesiredPriority_LPMPMeas << LIB_STATE_LPM_CFG_PRIO_POS) | (1 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMPMeas, (sint16_t)(HighLimit) , (sint16_t)(LowLimit));
			printf("Pressure-thresholds adjusted:\r\n");	
			printf("     LPMP-Low-Limit: 0x%x\r\n", LowLimit);
			printf("     LPMP-High-Limit: 0x%x\r\n",HighLimit);
			if(Wakeup_Controller->DEVSTATUS_b.LPMP_FLAG)
				printf("     LPMP-PEND flag was set, clear it now\r\n");
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk;		// Clear LPM-P pending-flag, because measurement was done above
			
			// Adjust acceleration-limits
			MeasCompPrint_Acceleration(&MeasResults, 1, &SensorValuePacked);
			#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Relative)
				LowLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMAMeas_Negative);
				HighLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMAMeas_Positive);
			#endif
			#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Absolute)			
				MeasResults.Comp_Value = DesiredThreshold_LPMAMeas_Negative*16;
				Lib_Comp_Reverse_Acceleration((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range | Comp_Acceleration_Offset_Source | COM_MEIF_EXT_CFG_TEMP_MEAS), 0, &MeasResults);	// temperature from external
				LowLimit = MeasResults.RAW_Value;
				MeasResults.Comp_Value = DesiredThreshold_LPMAMeas_Positive*16;
				Lib_Comp_Reverse_Acceleration((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range | Comp_Acceleration_Offset_Source | COM_MEIF_EXT_CFG_TEMP_MEAS), 0, &MeasResults);	// temperature from external
				HighLimit = MeasResults.RAW_Value;
			#endif
			Lib_State_LPM_Config((uint16_t)((1 << LIB_STATE_LPM_CFG_SRC_POS) | (2 << LIB_STATE_LPM_CFG_PRIO_POS) | (4 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMAMeas, (sint16_t)(HighLimit), (sint16_t)(LowLimit));
			printf("Acceleration-thresholds adjusted:\r\n");	
			printf("     LPMA-Low-Limit: 0x%x\r\n", LowLimit);
			printf("     LPMA-High-Limit: 0x%x\r\n",HighLimit);
			if(Wakeup_Controller->DEVSTATUS_b.LPMA_FLAG)
				printf("     LPMA-PEND flag was set, clear it now\r\n");
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk;		// Clear LPM-A pending-flag, because measurement was done above
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk);	// Write-clear LPMT_FLAG
		}	
		// Wakeup by LPM-A -> Acceleration outside thresholds
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk)
		{
			#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Relative)
				uint16_t LowLimit, HighLimit;
			#endif
			
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk);	// Write-clear LPMA_FLAG
			
			// Send message
			Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);
			printf("-->Wakeup by acceleration out of limits\r\n");
			// Measure, compensate and send temperature via Uart
			MeasCompPrint_Temperature(&MeasResults, &SensorValuePacked);
			// Measure, compensate and send Pressue via Uart
			MeasCompPrint_Pressure(&MeasResults, 1, &SensorValuePacked);
			// Measure and print current acceleration
			MeasCompPrint_Acceleration(&MeasResults, 1, &SensorValuePacked);
			// Adjust acceleration-limits in case of relative
			#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Relative)
				LowLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMAMeas_Negative);
				HighLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMAMeas_Positive);
				Lib_State_LPM_Config((uint16_t)((1 << LIB_STATE_LPM_CFG_SRC_POS) | (2 << LIB_STATE_LPM_CFG_PRIO_POS) | (4 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMAMeas, (sint16_t)(HighLimit), (sint16_t)(LowLimit));
				Uart_SendString((uint8_t *)"    Acceleration-thresholds adjusted:\r\n");
				Uart_SendString((uint8_t *)"     LPMA-Low-Limit: ");
				Uart_SendHex16(LowLimit, UART_SENDHEX_PREFIX_ENABLED);
				Uart_SendString((uint8_t *)"\r\n     LPMA-High-Limit: ");
				Uart_SendHex16(HighLimit, UART_SENDHEX_PREFIX_ENABLED);
				Uart_SendString((uint8_t *)"\r\n");
			#endif
			// Measure, compensate and send Voltage via Uart
			MeasCompPrint_SupplyVoltage(&MeasResults, &SensorValuePacked);
		}				
		// Any LPM measurement is still pending. Pending-flags can be set, although their mask is set. Therefore
		//		apply the DEVCTRL.LPMx_MASK bits to the DEVSTATUS.LPMx_FLAG bits (Note: order and position of P/T/A is the same in both SFRs)
		uint32_t pending_LPM_flags = (Wakeup_Controller->DEVSTATUS & ((~(Wakeup_Controller->DEVCTRL)) & (Wakeup_Controller_DEVCTRL_LPMP_MASK_Msk | Wakeup_Controller_DEVCTRL_LPMT_MASK_Msk | Wakeup_Controller_DEVCTRL_LPMA_MASK_Msk)));
		if(pending_LPM_flags != 0)
		{
			// Pressure measurement is pending
			if(pending_LPM_flags & Wakeup_Controller_DEVSTATUS_LPMP_PEND_Msk)
			{
				Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMP_PEND_Msk;	// Write-clear the pending-flag
			}
			// Temperature measurement is pending
			if(pending_LPM_flags & Wakeup_Controller_DEVSTATUS_LPMT_PEND_Msk)
			{
				Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMT_PEND_Msk;	// Write-clear the pending-flag
			}
			// Acceleration measurement is pending
			if(pending_LPM_flags & Wakeup_Controller_DEVSTATUS_LPMA_PEND_Msk)
			{
				Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_LPMA_PEND_Msk;	// Write-clear the pending-flag
			}
		}
	}
	// A reset occured
	else
	{
		frame_cnt = 0;
		uint16_t LowLimit, HighLimit;
		// Enable retention for ret. RAM
		Wakeup_Controller->DEVCTRL_b.RETMEM0 = 1;
		Wakeup_Controller->DEVCTRL_b.RETMEM1 = 1;
		Wakeup_Controller->DEVCTRL_b.RETMEM2 = 1;
		Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);
		uint32_t status;
		uint8_t UartStatus;
		status = Lib_Serv_Start_Xtal_Osc(COM_MISC_EXT_OSC_CFG_XTAL_BOOST, 20);
		if(status != 0)
					ErrorHandling(ErrorNumber_CrystalNotStarted);
		UartStatus = Uart_Baudrate_Calib();
		if(UartStatus != 0)
					ErrorHandling(ErrorNumber_UartError);
		status = Lib_Serv_Stop_Xtal_Osc();
		if(status != 0)
					ErrorHandling(ErrorNumber_CrystalNotStopped);
		// Check of RC-oscillators
		status = Lib_Serv_Start_Xtal_Osc(COM_MISC_EXT_OSC_CFG_XTAL_BOOST, 20);
		if(status != 0)
			ErrorHandling(ErrorNumber_CrystalNotStarted);
		status = Lib_Diag_RC_Osc_Chk((uint32_t)(LIB_DIAG_OSC_CHK_LP_RC | LIB_DIAG_OSC_CHK_MP_RC | LIB_DIAG_OSC_CHK_HP_RC), 0);
		if(status != 0)
			ErrorHandling(ErrorNumber_RCOscCheckFailed);		
		status = Lib_Serv_Stop_Xtal_Osc();
		if(status != 0)
			ErrorHandling(ErrorNumber_CrystalNotStopped);
#if WU_IT
		// Set interval-timer to 1s
		status = Lib_Calib_Precounter(100);											// Set TIMEBASE for 100ms
		if(status != 0)
			ErrorHandling(ErrorNumber_LibCalibPrecountFailed);
		status = Lib_Serv_Init_Interval_Timer(DesiredPeriod100ms_IntervalWakup);								// Set postcounter to 9 -> 1s interval-timing
		if(status != 0)
			ErrorHandling(ErrorNumber_LibServInitITFailed);
#endif
			//Enable wake up by external by poll PP2 to GND
#if WU_EXT
			Wakeup_Controller->GPIO_b.PPIEN2 = 1;
			Wakeup_Controller->GPIO_b.PPD2 = 1;
			Wakeup_Controller->GPIO_b.PPO2 = 1;
			Wakeup_Controller->GPIO_b.PPULL2 = 0;
			Wakeup_Controller->DEVCTRL_b.EXT_MASK = 0;
			Wakeup_Controller->DEVCTRL_b.WUP_PIN_SEL = 0;
#endif
//			printf("Measurement results:\r\n");
			
			// Measure, compensate and send temperature via Uart
			MeasCompPrint_Temperature(&MeasResults, &SensorValuePacked);
			LowLimit = MeasResults.Scaled_Temp_Value + DesiredThreshold_LPMTMeas_Negative;
			HighLimit = MeasResults.Scaled_Temp_Value + DesiredThreshold_LPMTMeas_Positive;
			Lib_State_LPM_Config((uint16_t)((2 << LIB_STATE_LPM_CFG_SRC_POS) | (DesiredPriority_LPMTMeas << LIB_STATE_LPM_CFG_PRIO_POS) | (0 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMTMeas, (sint16_t)(HighLimit), (sint16_t)(LowLimit));		
			Uart_SendString((uint8_t *)"     LPMT-Low-Limit: ");
			printf("     LPMT-Low-Limit: 0x%x\r\n", LowLimit);
			printf("     LPMT-High-Limit: 0x%x\r\n",HighLimit);
			// Measure, compensate and send Pressue via Uart
			MeasCompPrint_Pressure(&MeasResults, 1, &SensorValuePacked);
			#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Relative)
				LowLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMPMeas_Negative);
				HighLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMPMeas_Positive);
			#endif
			#if (DesiredThresholdMethod_LPMPMeas == _ThresholdMethod_Absolute)			
				MeasResults.Comp_Value = DesiredThreshold_LPMPMeas_Negative*16;
				Lib_Comp_Reverse_Pressure((uint32_t)(Comp_Pressure_CRCCheckEnable | Comp_Pressure_DesiredPSet | COM_MEIF_EXT_CFG_TEMP_MEAS), &MeasResults);	// temperature from external
				LowLimit = MeasResults.RAW_Value;
				MeasResults.Comp_Value = DesiredThreshold_LPMPMeas_Positive*16;
				Lib_Comp_Reverse_Pressure((uint32_t)(Comp_Pressure_CRCCheckEnable | Comp_Pressure_DesiredPSet | COM_MEIF_EXT_CFG_TEMP_MEAS), &MeasResults);	// temperature from external
				HighLimit = MeasResults.RAW_Value;
			#endif
			Lib_State_LPM_Config((uint16_t)((0 << LIB_STATE_LPM_CFG_SRC_POS) | (DesiredPriority_LPMPMeas << LIB_STATE_LPM_CFG_PRIO_POS) | (1 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMPMeas, (sint16_t)(HighLimit), (sint16_t)(LowLimit));
			Uart_SendString((uint8_t *)"     LPMP-Low-Limit: ");
			printf("     LPMP-Low-Limit: 0x%x\r\n", LowLimit);
			printf("     LPMP-High-Limit: 0x%x\r\n",HighLimit);
			// Measure, compensate and send Acceleration via Uart
			MeasCompPrint_Acceleration(&MeasResults, 1, &SensorValuePacked);
			#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Relative)
				LowLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMAMeas_Negative);
				HighLimit = (MeasResults.RAW_Value + DesiredThreshold_LPMAMeas_Positive);
			#endif
			#if (DesiredThresholdMethod_LPMAMeas == _ThresholdMethod_Absolute)			
				MeasResults.Comp_Value = DesiredThreshold_LPMAMeas_Negative*16;
				Lib_Comp_Reverse_Acceleration((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range | Comp_Acceleration_Offset_Source | COM_MEIF_EXT_CFG_TEMP_MEAS), 0, &MeasResults);	// temperature from external
				LowLimit = MeasResults.RAW_Value;
				MeasResults.Comp_Value = DesiredThreshold_LPMAMeas_Positive*16;
				Lib_Comp_Reverse_Acceleration((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range | Comp_Acceleration_Offset_Source | COM_MEIF_EXT_CFG_TEMP_MEAS), 0, &MeasResults);	// temperature from external
				HighLimit = MeasResults.RAW_Value;
			#endif
			Lib_State_LPM_Config((uint16_t)((1 << LIB_STATE_LPM_CFG_SRC_POS) | (2 << LIB_STATE_LPM_CFG_PRIO_POS) | (4 << LIB_STATE_LPM_CFG_NRS_POS)), (uint16_t)DesiredPeriod100ms_LPMAMeas, (sint16_t)(HighLimit), (sint16_t)(LowLimit));
			Uart_SendString((uint8_t *)"     LPMA-Low-Limit: ");
			printf("     LPMA-Low-Limit: 0x%x\r\n", LowLimit);
			printf("     LPMA-High-Limit: 0x%x\r\n",HighLimit);
			// Measure, compensate and send Voltage via Uart
			MeasCompPrint_SupplyVoltage(&MeasResults, &SensorValuePacked);
			// Enable wakeup by LPM-sources, clear all flags in DEVSTATUS and start LPM
			Wakeup_Controller->DEVCTRL &= ~(Wakeup_Controller_DEVCTRL_LPMP_MASK_Msk | Wakeup_Controller_DEVCTRL_LPMA_MASK_Msk | Wakeup_Controller_DEVCTRL_LPMT_MASK_Msk);		// Enable wakeup by any LPM-source			
			Wakeup_Controller->DEVSTATUS = 0xFFFFFFFF;		// Write-clear all flags in DEVSTATUS				
			Lib_State_LPM_Start(LIB_STATE_LPM_SS_START_ITIMER | LIB_STATE_LPM_SS_START_PRESS | LIB_STATE_LPM_SS_START_ACC | LIB_STATE_LPM_SS_START_TEMP);	// Start and sync all timers		
	}
	frame_cnt = frame_cnt + 1;
	SensorValuePacked.TX_buf_b.START = 0x22;
	SensorValuePacked.TX_buf_b.FRAME_CNT = frame_cnt;
	SensorValuePacked.TX_buf_b.RFU = 0x00;
	SensorValuePacked.TX_buf_b.END = 0x03;
	swapBytes(&SensorValuePacked.TX_buf_b.START, 			sizeof(SensorValuePacked.TX_buf_b.START));
	swapBytes(&SensorValuePacked.TX_buf_b.FRAME_CNT,  sizeof(SensorValuePacked.TX_buf_b.FRAME_CNT));
	swapBytes(&SensorValuePacked.TX_buf_b.TEMP, 			sizeof(SensorValuePacked.TX_buf_b.TEMP));
	swapBytes(&SensorValuePacked.TX_buf_b.PRES, 			sizeof(SensorValuePacked.TX_buf_b.PRES));
	swapBytes(&SensorValuePacked.TX_buf_b.VEL, 				sizeof(SensorValuePacked.TX_buf_b.VEL));
	swapBytes(&SensorValuePacked.TX_buf_b.BAT, 				sizeof(SensorValuePacked.TX_buf_b.BAT));
	swapBytes(&SensorValuePacked.TX_buf_b.RFU, 				sizeof(SensorValuePacked.TX_buf_b.RFU));
	swapBytes(&SensorValuePacked.TX_buf_b.END, 				sizeof(SensorValuePacked.TX_buf_b.END));
	uint8_t UartFrameEndcode[32];
	printf("Raw frame: ");
	for(int i = 0; i < 16; i++)
	{
		printf("%02x", SensorValuePacked.TX_buf[i]);
	}
	printf("\r\n");
	printf("Endcoded frame: ");
	hamming_8_4_array_enc(SensorValuePacked.TX_buf, 16, UartFrameEndcode);
	for(int i = 0; i < 32; i++)
	{
#if HEX_STR_FORMAT
	//	Uart_SendHex8(UartFrameEndcode[i], 0);
#else
		Uart_Putchar(UartFrameEndcode[i]);
#endif
	}
	printf("\r\n");
REI2C:
	Retry_I2C++;
	Wakeup_Controller->GPIO_b.PPD2 = 0;
	Wakeup_Controller->GPIO_b.PPO2 = 1;
	InitI2C();
	Lib_Serv_I2C_SendReceive_t User_I2C;
	User_I2C.Send_Buffer_Size = 32;
	User_I2C.Received_bytes = 100;
	User_I2C.Send_Buffer = UartFrameEndcode;
	//User_I2C.Received_Buffer = Rec_buf;
	uint8_t Send_ERR_Byte= 0xff;
	uint32_t I2C_error = 0;
	// This syntax sets single bits via read-modify-write
	Corelogic->TIMERCFG01_b.WDRES = 1;
	I2C_error = Lib_Serv_I2C_Slave_Send_Receive(&User_I2C, Send_ERR_Byte, 10);
	Wakeup_Controller->GPIO_b.PPD2 = 0;
	Wakeup_Controller->GPIO_b.PPO2 = 0;
	if(!I2C_error) 
	{
		printf("I2C success \r\n ");
		if(User_I2C.Received_bytes)
		{
			//printf("Rec: ");
			for(int i = 0; i < User_I2C.Received_bytes; i++)
			{
				//printf("%x ", User_I2C.Received_bytes[i]);
			}
			printf("\r\n");
		}
	}
	else
	{
		printf("I2C error: ");
		printf("%x", I2C_error);
		printf("\r\n");
		if (Retry_I2C < 2) goto REI2C;
	}
	// All wakeup- and reset-flags should be cleared here from branches above. Go to PDWN.
//	Wakeup_Controller->GPIO_b.PPD0 = 0;
//	Wakeup_Controller->GPIO_b.PPO0 = 0;
//	Wakeup_Controller->GPIO_b.PPD1 = 0;
//	Wakeup_Controller->GPIO_b.PPO1 = 0;
	Wakeup_Controller->GPIO_b.PPD2 = 0;
	Wakeup_Controller->GPIO_b.PPO2 = 0;
//	Wakeup_Controller->GPIO_b.PPD3 = 0;
//	Wakeup_Controller->GPIO_b.PPO3 = 0;
	Lib_State_Low_Power(COM_MISC_EXT_LP_PWD);
}

void MeasCompPrint_Temperature(Com_MEIF_Ext_Meas_t* MeasResults, VAL_FRAME_t* SensorValuePacked)
{
	uint32_t status;
	
	// Measure, compensate and send temperature via Uart
	status = Lib_Meas_Comp_Temp((uint32_t)(MeasComp_Temperature_CRCCheckEnable /*| (COM_MEIF_EXT_CFG_SKIP_TEMP_COMP)*/), MeasResults);	// do NOT skip T-compensation
	if(status != COM_MEIF_EXT_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_TMeasCompError);
		printf("     ->Error during Lib_Meas_Comp_Temp, status = ");
		Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
		printf("\r\n");
	}
	SensorValuePacked->TX_buf_b.TEMP = MeasResults->Comp_Value / 128;
	printf("    Temperature: ");
	printf("%d", MeasResults->Comp_Value / 128);
	printf(" degC, ");
	printf("%x", MeasResults->Scaled_Temp_Value);
	printf(" LSB (scaled Temp)\r\n");
}

void MeasCompPrint_Pressure(Com_MEIF_Ext_Meas_t* MeasResults, uint8_t UseTempMeasFromExternal, VAL_FRAME_t* SensorValuePacked)
{
	uint32_t status;
	
	// Measure, compensate and send Pressue via Uart
	status = Lib_Meas_Raw_Pressure((uint32_t)(BITSIZE(Meas_RawPressure_NumSamples) | Meas_RawPressure_WBCCheckEnable | Meas_RawPressure_CRCCheckEnable), MeasResults);
	if(status != COM_MEIF_EXT_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_PMeasError);
		printf("     ->Error during Lib_Meas_Raw_Pressure, status = ");
		printf("%x", status);
		printf("\r\n");
	}
	if(UseTempMeasFromExternal)
		status = Lib_Comp_Pressure((uint32_t)(Comp_Pressure_CRCCheckEnable | Comp_Pressure_DesiredPSet | COM_MEIF_EXT_CFG_TEMP_MEAS), MeasResults);	// temperature value from external
	else
		status = Lib_Comp_Pressure((uint32_t)(Comp_Pressure_CRCCheckEnable | Comp_Pressure_DesiredPSet /*| COM_MEIF_EXT_CFG_TEMP_MEAS*/), MeasResults);	// temperature value from internal
	if(status != COM_MEIF_EXT_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_PCompError);
		printf("     ->Error during Lib_Comp_Pressure, status = ");
		printf("%x", status);
		printf("\r\n");
	}
	SensorValuePacked->TX_buf_b.PRES = MeasResults->Comp_Value / 16;
	printf("    Pressure: ");
	printf("%d", MeasResults->Comp_Value / 16);
	printf(" kPa, ");
	printf("%x", MeasResults->RAW_Value);
	printf(" LSB\r\n");
}

void MeasCompPrint_Acceleration(Com_MEIF_Ext_Meas_t* MeasResults, uint8_t UseTempMeasFromExternal, VAL_FRAME_t* SensorValuePacked)
{
	uint32_t status;
	
	// Temperature is needed for Lib_Comp_Auto_Acc_Offset. If not supplied from external, measure it now (without compensation, just scaled temp)
	if(!UseTempMeasFromExternal)
	{
		status = Lib_Meas_Comp_Temp((uint32_t)(MeasComp_Temperature_CRCCheckEnable | COM_MEIF_EXT_CFG_SKIP_TEMP_COMP), MeasResults);	// do skip T-compensation
		if(status != COM_MEIF_EXT_STAT_OK)
		{
			//ErrorHandling(ErrorNumber_TMeasCompError);
			printf("     ->Error during Lib_Meas_Comp_Temp, status = ");
			printf("%x", status);
			printf("\r\n");
		}
	}
	
	// Measure, compensate and send Acceleration via Uart
	status = Lib_Meas_Raw_Acceleration((uint32_t)(BITSIZE(Meas_RawAcceleration_NumSamples) | Meas_RawAcceleration_WBCCheckEnable | Meas_RawAcceleration_RDCheckEnable | Meas_RawAcceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range), MeasResults);
	if(status != COM_MEIF_EXT_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_AMeasError);
		printf("     ->Error during Lib_Meas_Raw_Acceleration, status = ");
		printf("%x", status);
		printf("\r\n");
	}
	else
	{
		// It's required to ALWAYS run the auto-offset-feature after an acceleration-measurement
		status = Lib_Comp_Auto_Acc_Offset((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range), (uint8_t)(DesiredAutoAccOffsetBlocksize-1), (uint16_t)(DesiredAutoAccOffsetMovingThreshold_LSBsqr), MeasResults);
		if(status > COM_MEIF_EXT_ACC_AO_TEMP_ERR)
		{
			//ErrorHandling(ErrorNumber_AMeasError);
			printf("     ->Error during Lib_Comp_Auto_Acc_Offset, status = ");
			printf("%x", status);
			printf("\r\n");
		}
	}
	status = Lib_Comp_Acceleration((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range | Comp_Acceleration_Offset_Source | COM_MEIF_EXT_CFG_TEMP_MEAS), Comp_Acceleration_Offset_Arg, MeasResults);	// temperature value from external
	if(status != COM_MEIF_EXT_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_ACompError);
		printf("     ->Error during Lib_Comp_Acceleration, status = ");
		printf("%x", status);
		printf("\r\n");
	}
	SensorValuePacked->TX_buf_b.VEL = MeasResults->Comp_Value / 16;
	printf("    Acceleration: ");
	printf("%d", MeasResults->Comp_Value / 16);
	printf(" g, ");
	printf("%x", MeasResults->RAW_Value);
	printf(" LSB\r\n");
}

void MeasCompPrint_SupplyVoltage(Com_MEIF_Ext_Meas_t* MeasResults, VAL_FRAME_t* SensorValuePacked)
{
	uint32_t status;
	
	status = Lib_Meas_Comp_SupplyV((uint32_t)(MeasComp_SupplyV_CRCCheckEnable), MeasResults);
	if(status != COM_MEIF_EXT_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_VMeasCompError);
		printf("     ->Error during Lib_Meas_Comp_SupplyV, status = ");
		printf("%x", status);
		printf("\r\n");
	}
	SensorValuePacked->TX_buf_b.BAT = round((MeasResults->Comp_Value / 8) * 100 / MAX_VOL_PWR_SUP);
	printf("    Supply-Voltage: ");
	printf("%d", MeasResults->Comp_Value / 8);
	printf(" mV, ");
	printf("%x", MeasResults->Scaled_Temp_Value);
	printf(" LSB\r\n");
}

void MeasCompPrint_AIN(Com_MEIF_Ext_Meas_t* MeasResults, VAL_FRAME_t* SensorValuePacked)
{
	uint32_t status; 
	// Initialize pin for AIN on PP0 or PP3
	#if (MeasComp_AIN_Pin == _AIN_Pin_PP0)
		Wakeup_Controller->GPIO_b.PPD0 = 1;		// PP0 = input
		Wakeup_Controller->GPIO_b.PPO0 = 0;		// No pulling-resistors for PP0
	#else
		Wakeup_Controller->GPIO_b.PPD3 = 1;		// PP3 = input
		Wakeup_Controller->GPIO_b.PPO3 = 0;		// No pulling-resistors for PP3
	#endif
	
	status = Lib_Serv_Pin_Config((uint8_t)(LIB_SERV_PIN_CFG_EN | MeasComp_AIN_Pin), 0);	// EN = Enable, DIR = Analog-Input
	if(status != LIB_SERV_PIN_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_PinConfigError);
		//Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);

		Uart_SendString((uint8_t*)"     ->Error during Lib_Serv_Pin_Config, status = ");
		Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
		printf("\r\n");
	}
	status = Lib_Meas_Comp_Ain_Voltage((uint32_t)(MeasComp_AIN_CRCCheckEnable), MeasResults);
	if(status != COM_MEIF_EXT_STAT_OK)
	{
		//ErrorHandling(ErrorNumber_AinMeasError);
		//Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);
		Uart_SendString((uint8_t*)"     ->Error during Lib_Meas_Comp_Ain_Voltage, status = ");
		Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
		printf("\r\n");
	}
	Lib_Serv_Pin_Config(0, 0);			// EN = Disable
	//Uart_Init(UART_CONFIG_REFCLK_HPRC, BR_19k2);
	#if (MeasComp_AIN_Pin == _AIN_Pin_PP0)
		Uart_SendString((uint8_t *)"    Voltage at AIN (PP0): ");
	#else
		Uart_SendString((uint8_t *)"    Voltage at AIN (PP3): ");
	#endif
	Uart_SendDecimalSigned(MeasResults->Comp_Value / 8);
	Uart_SendString((uint8_t *)" mV, ");
	Uart_SendHex16(MeasResults->RAW_Value, UART_SENDHEX_PREFIX_ENABLED);
	Uart_SendString((uint8_t *)" LSB\r\n");
}

void ErrorHandling(int ErrorNumber)
{
	// Blink 'ErrorNumber' times and while looping forever
//	Wakeup_Controller->GPIO_b.PPD0 = 0;		// Make PP0 an output

//	while(1)
//	{
//		//Blink
//		for(int i = ErrorNumber; i > 0; i--)
//		{
//			Wakeup_Controller->GPIO_b.PPO0 = 1;		// Set output high
//			Delay1ms(500);
//			Wakeup_Controller->GPIO_b.PPO0 = 0;		// Set output low
//			Delay1ms(500);
//		}
//		
//		Delay1ms(2000);
//	
//		// Watchdog reset
//		Corelogic->TIMERCFG01_b.WDRES = 1;
//	}
	printf("error: %d", ErrorNumber);
}

void InitI2C()
{
	Wakeup_Controller->GPIO_b.PPD0 = 1;
	Wakeup_Controller->GPIO_b.PPO0 = 1;
	Wakeup_Controller->GPIO_b.PPULL0 = 1;
	Wakeup_Controller->GPIO_b.PPIEN0 = 1;
	Wakeup_Controller->GPIO_b.PPD1 = 1;
	Wakeup_Controller->GPIO_b.PPO1 = 1;
	Wakeup_Controller->GPIO_b.PPULL1 = 1;
	Wakeup_Controller->GPIO_b.PPIEN1 = 1;
	Wakeup_Controller->DEVCTRL_b.I2CEN = 1;
}