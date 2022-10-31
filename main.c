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
// -------------------------------------
// Defined for APS application
// -------------------------------------
#define _APS_FIR_Type_FIR1				(0)
#define _APS_FIR_Type_FIR2				(1)
#define _APS_ACQ_Type_HWAvg				(0)
#define _APS_ACQ_Type_FIR					(1)
#define APS_Type						_APS_Type_Extrapolation // Choose here the desired APS-type (e.g. extrapolation of time&phase approach)
#define _APS_Type_Extrapolation		(0)
#define _APS_Type_TimeAndPhase		(1)
#if (APS_Type == _APS_Type_Extrapolation)
	uint16_t APS_ExtrapolationAngle	=(3072)	;					// Define here the desired angle for extrapolation in [°/360*4096] (range from [0 ... 4096]). 
																										// 0 (0°)   		=> zero crossing
																										// 1024 (90°)		=> bottom of rotation
																										// 2048 (180°) 	=> zero crossing
																										// 3072 (270°) 	=> top of rotation
#endif
// When the device is mounted with the pressure-whole facing towards the outside of the tire (Acc_Direction_Positive),
//	then (and only then!) we can execute the standard APS algorithm. Otherwise we would need the APS_Ontire.
#if (MeasComp_Acceleration_Direction == _Acc_Direction_Positive)
	// K1, corresponding to tire size according to formula: K1 = round(16 / pi * sqrt(9.81 / 2 / d)) with d = diameter of rotation of TPMS module in [m]
	//		Examples for K1: 	d = [0.31 ... 0.33] 	=> K1 = 20
	//											d = [0.34 ... 0.37] 	=> K1 = 19
	//											d = [0.38 ... 0.41] 	=> K1 = 18
	//											d = [0.42 ... 0.46] 	=> K1 = 17
	//											d = [0.47 ... 0.52] 	=> K1 = 16
	//											d = [0.53 ... 0.60] 	=> K1 = 15
	#define APS_K1																(18)	// Define here the K1 fitting to your tire-size
	#define APS_NumSamples_PerPeriod							(6)		// Define here the number of samples per period for APS. Default = 6
	#define APS_RawAcceleration_CRCCheckEnable	_CRC_Check_Enabled			// Enable or Disable the CRC-check for APS-acceleration-measurement
	#define APS_RawAcceleration_WBCCheckEnable	_WBC_Check_Enabled			// Enable or Disable the WBC-check for APS-acceleration-measurement
	#define APS_RawAcceleration_RDCheckEnable		_RD_Check_Enabled				// Enable or Disable the RD-check for APS-acceleration-measurement
	// Generally FIR1 is comparable with HW-averaging of 16 samples, but has much better damping above 2kHz.
	// Generally FIR2 is comparable with HW-averaging of 64 samples, but has better damping between 600Hz and 2.5kHz
	// Final selection of filter-type depends on wheel (-type), module and mounting and has to be done by the user.
	#define APS_AcqType		_APS_ACQ_Type_FIR						// Choose here between hardware-averaging and FIR-filter for acquistion of APS-acceleration-samples
	#if (APS_AcqType == _APS_ACQ_Type_HWAvg)
		#define APS_RawAcceleration_NumSamples			(32)	// Define here how many RAW-samples shall be averaged for eache APS-acceleration-sample
		#define APS_SampleTimeCorrection						((uint16_t)((50.61 + APS_RawAcceleration_NumSamples*7.16)+0.5))		// Number of MPRC clock-ticks for sample-time correction, do not change
	#endif		
	#if (APS_AcqType == _APS_ACQ_Type_FIR)
		#define APS_FIRType		_APS_FIR_Type_FIR2						// Choose here between FIR1 (12 samples, cut-off frequency ~2kHz) or FIR2 (36 samples, cut-off frequency ~600Hz)
		#if (APS_FIRType == _APS_FIR_Type_FIR1)
			#define APS_SampleTimeCorrection						(35)	// Number of MPRC clock-ticks for sample-time correction, do not change
		#endif
		#if (APS_FIRType == _APS_FIR_Type_FIR2)
			#define APS_SampleTimeCorrection						(117)	// Number of MPRC clock-ticks for sample-time correction, do not change
		#endif
	#endif
	#define APS_Thr_ApsDif			(1000)										// Define here the threshold for RC1 (AbsDif). If you experience bad APS-accuracy related to noisy signal, you can decrease this value.
	#define APS_Thr_MaxMin			(1000)										// Define here the threshold for RC2 (MaxMin). If you experience bad APS-accuracy related to strong acceleration / braking of the vehicle, you can decrease this value.
	#define APS_Thr_DFTPtA			(21)										// Define here the threshold for RC3 (DFTPtA). If you experience bad APS-accuracy, you can increase this value. If you experience many rejects, you can try to decrease this value.
	
	#if (APS_Type == _APS_Type_TimeAndPhase)		
		// Some time in [MPRC-cycles] is needed after finishing APS, until the RF-frame is really started. 
		//		The default value value was measured with and should only be changed, if code was changed in the timing-critical section below.
		#define APS_RFTx_StartDelay_MPRC							(118+StartXtal_Delay100us*9)
	#endif
#endif
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

#define DEBUG_UART 1
#define	WU_IT			 0			// wake up source is interval timer
#define	WU_EXT		 1			// wake up source is PP2 (pull down)
#define MAX_VOL_PWR_SUP				3300			
/**********************************************************************************
 Global Variables
 
***********************************************************************************/
uint32_t * frame_cnt = (void *)0x30000000; //retention RAM variable for getting sensor counter

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
/**********************************************************************************
 Function Implementations
***********************************************************************************/
int main(void)
{		
	VAL_FRAME_t SensorValuePacked;
	*frame_cnt = *frame_cnt + 1;
	SensorValuePacked.TX_buf_b.START = 0x22;
	SensorValuePacked.TX_buf_b.FRAME_CNT = *frame_cnt;
	SensorValuePacked.TX_buf_b.RFU = 0x00;
	SensorValuePacked.TX_buf_b.END = 0x03;
	
	// A wakeup from powerdown occured
	if(Wakeup_Controller->DEVSTATUS_b.WUPDWN != 0)
	{
		// Check the actual wakeup-source. Wakeup-flags can be set, but only if they are not masked, they can generate a wakeup.
		//	For this purpose we calculate a mask to be applied to the flags in DEVSTATUS:
		//			-All mask-bits from EXT_MASK (Bit 9) to CD_MASK (Bit0) --> invert and use Bits9 ... Bit0 (0x03FF) in DEVCTRL
		//			-ITIM cannot be masked in normal mode --> OR with ITIM_MASK_Msk
		//			-LPM-TRIGGER flags are only set, if their mask-bit in DEVCTRL is 0 --> No need to consider DEVCTRL.LPMx_MASK, just OR with DEVSTATUS.LPMx_FLAG_Msk
		uint32_t triggered_wakeup_mask = ((~(Wakeup_Controller->DEVCTRL) & 0x03FF) | Wakeup_Controller_DEVCTRL_ITIM_MASK_Msk | Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk | Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk | Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk);
		uint32_t triggered_wakeup = (Wakeup_Controller->DEVSTATUS & (triggered_wakeup_mask));
		
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
			Com_MEIF_Ext_Meas_t MeasResults;
			
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
			printf("Measurement results:\r\n");
			
			// Measure, compensate and send temperature via Uart
			MeasCompPrint_Temperature(&MeasResults, &SensorValuePacked);
			
			// Measure, compensate and send Pressue via Uart
			MeasCompPrint_Pressure(&MeasResults, 1, &SensorValuePacked);
			
			// Measure, compensate and send Acceleration via Uart
			MeasCompPrint_Acceleration(&MeasResults, 1, &SensorValuePacked);

			// Measure, compensate and send Voltage via Uart
			MeasCompPrint_SupplyVoltage(&MeasResults, &SensorValuePacked);
			
			// Estimate frequency rotation
			//EarlyRollingMode_GenerateAPSTrial_SendRF(&MeasResults, 1);
		}
		// Wakeup by external
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_EXT_FLAG_Msk)
		{
			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_EXT_FLAG_Msk;	// Write-clear EXT_FLAG
			Com_MEIF_Ext_Meas_t MeasResults;
			
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
			printf("Measurement results:\r\n");
			
			// Measure, compensate and send temperature via Uart
			MeasCompPrint_Temperature(&MeasResults, &SensorValuePacked);
			
			// Measure, compensate and send Pressue via Uart
			MeasCompPrint_Pressure(&MeasResults, 1, &SensorValuePacked);
			
			// Measure, compensate and send Acceleration via Uart
			MeasCompPrint_Acceleration(&MeasResults, 1, &SensorValuePacked);
			
			// Measure, compensate and send Voltage via Uart
			MeasCompPrint_SupplyVoltage(&MeasResults, &SensorValuePacked);
			
			// Measure voltage at AIN (PP0 or PP3)
			//MeasCompPrint_AIN(&MeasResults, &SensorValuePacked);
		}
		// Wakeup by LPM-P -> Pressure outside thresholds
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk)
		{
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMP_FLAG_Msk);	// Write-clear LPMP_FLAG
		}
		// Wakeup by LPM-T -> Temperature outside thresholds
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk)
		{
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMT_FLAG_Msk);	// Write-clear LPMT_FLAG
		}	
		// Wakeup by LPM-A -> Acceleration outside thresholds
		if(triggered_wakeup & Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk)
		{
			Wakeup_Controller->DEVSTATUS = (Wakeup_Controller_DEVSTATUS_LPMA_FLAG_Msk);	// Write-clear LPMA_FLAG
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
		// Enable retention for ret. RAM
		Wakeup_Controller->DEVCTRL_b.RETMEM0 = 1;
		Wakeup_Controller->DEVCTRL_b.RETMEM1 = 1;
		Wakeup_Controller->DEVCTRL_b.RETMEM2 = 1;
		*frame_cnt = 0;
		// Power-on reset
		if(Wakeup_Controller->DEVSTATUS_b.POR != 0)
		{
			uint8_t status;
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
			status = Lib_Serv_Init_Interval_Timer(9);								// Set postcounter to 9 -> 1s interval-timing
			if(status != 0)
				ErrorHandling(ErrorNumber_LibServInitITFailed);
			#endif
			//Enable wake up by external by poll PP2 to GND
			#if WU_EXT
			Wakeup_Controller->GPIO_b.PPIEN2 = 1;
			Wakeup_Controller->GPIO_b.PPD2 = 1;
			Wakeup_Controller->GPIO_b.PPO2 = 1;
			Wakeup_Controller->GPIO_b.PPULL2 = 1;
			Wakeup_Controller->DEVCTRL_b.EXT_MASK = 0;
			Wakeup_Controller->DEVCTRL_b.WUP_PIN_SEL = 0;
			#endif
			Wakeup_Controller->DEVSTATUS = 0xFFFFFFFF;		// Write-clear all flags in DEVSTATUS				
		}
//		// Software reset
//		else if(Wakeup_Controller->DEVSTATUS_b.SRESET != 0)
//		{
//			// Was this a hard-fault?
//			uint32_t HFaultFlags = Lib_Serv_Hard_Fault(0xFFFFFFFF);	 // Delete all HFault-flags
//			if(HFaultFlags != LIB_SERV_HF_OK)
//			{
//				if(HFaultFlags & LIB_SERV_HFSFRSIZE)	// SFR Data Size Fault
//				{}
//				if(HFaultFlags & LIB_SERV_HFADDR)			// Address Fault
//				{}
//				if(HFaultFlags & LIB_SERV_HFSFRPROT)	// SFR protected Fault
//				{}
//				if(HFaultFlags & LIB_SERV_HFROMREAD)	// ROM Read Fault
//				{}
//				if(HFaultFlags & LIB_SERV_HFROMEXEC)	// ROM Execution Fault
//				{}
//				if(HFaultFlags & LIB_SERV_HF)					// Hard Fault occurred (ARM internal or SP49 specific)
//				{}
//			}
//			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_SRESET_Msk;// Write-clear SRESET
//		}
//		// Watchdog reset
//		else if(Wakeup_Controller->DEVSTATUS_b.WDOG != 0)
//		{
//			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_WDOG_Msk;	// Write-clear WDOG
//		}
//		// Undervoltage reset
//		else if(Wakeup_Controller->DEVSTATUS_b.UVR != 0)
//		{
//			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_UVR_Msk;		// Write-clear UVR
//		}
//		// Thermal-shutdown-release reset
//		else if(Wakeup_Controller->DEVSTATUS_b.TDET != 0)
//		{
//			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_TDET_Msk;	// Write-clear TDET
//		}
//		// Brown-out reset
//		else if(Wakeup_Controller->DEVSTATUS_b.BOD != 0)
//		{
//			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_BOD_Msk;		// Write-clear BOD
//		}
//		// ECC2 reset
//		else if(Wakeup_Controller->DEVSTATUS_b.ECC2 != 0)
//		{
//			Wakeup_Controller->DEVSTATUS = Wakeup_Controller_DEVSTATUS_ECC2_Msk;	// Write-clear ECC2
//		}
		// This should never happen here
		else
		{
			ErrorHandling(ErrorNumber_UnknownReset);
		}	
	}
	// All wakeup- and reset-flags should be cleared here from branches above. Go to PDWN.
	swapBytes(&SensorValuePacked.TX_buf_b.START, sizeof(SensorValuePacked.TX_buf_b.START));
	swapBytes(&SensorValuePacked.TX_buf_b.FRAME_CNT, sizeof(SensorValuePacked.TX_buf_b.FRAME_CNT));
	swapBytes(&SensorValuePacked.TX_buf_b.TEMP, sizeof(SensorValuePacked.TX_buf_b.TEMP));
	swapBytes(&SensorValuePacked.TX_buf_b.PRES, sizeof(SensorValuePacked.TX_buf_b.PRES));
	swapBytes(&SensorValuePacked.TX_buf_b.VEL, sizeof(SensorValuePacked.TX_buf_b.VEL));
	swapBytes(&SensorValuePacked.TX_buf_b.BAT, sizeof(SensorValuePacked.TX_buf_b.BAT));
	swapBytes(&SensorValuePacked.TX_buf_b.RFU, sizeof(SensorValuePacked.TX_buf_b.RFU));
	swapBytes(&SensorValuePacked.TX_buf_b.END, sizeof(SensorValuePacked.TX_buf_b.END));
	uint8_t UartFrameEndcode[32];
	uint8_t UartFrameDecode[16];
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
		//printf("%02x", UartFrameEndcode[i]);
		Uart_SendHex8(UartFrameEndcode[i], 0);
	}
	printf("\r\n");
	printf("Decoded frame: ");
	hamming_8_4_array_dec(UartFrameEndcode, 32, UartFrameDecode);
	for(int i = 0; i < 16; i++)
	{
		printf("%02x", UartFrameDecode[i]);
	}
	printf("\r\n");
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
}
void EarlyRollingMode_GenerateAPSTrial_SendRF(Com_MEIF_Ext_Meas_t* MeasResults, uint8_t UseTempMeasFromExternal)
{
	sint16_t APS_Sample_Array[21];
	#if (MeasComp_Acceleration_Direction == _Acc_Direction_Positive)
		// We do the standard APS below
		uint32_t status;
		Lib_Adv_APS_Data_t APS_Data;
		
		// Below we will try to generate an APS-trial several times, until it succeeds. A certain delay between one un-succesful trial to the next is done in standby
		uint8_t MaxTrialRepetitions = 4;
		uint8_t APSSuccesfullyFinished = 0;
		for(; MaxTrialRepetitions != 0; MaxTrialRepetitions --)
		{
			System_Controller->STMR_b.STMREN = 0;
			uint32_t status;
			if(!UseTempMeasFromExternal)
			{
				status = Lib_Meas_Comp_Temp((uint32_t)(MeasComp_Temperature_CRCCheckEnable | COM_MEIF_EXT_CFG_SKIP_TEMP_COMP), MeasResults);	// do skip T-compensation
				if(status != COM_MEIF_EXT_STAT_OK)
				{
					ErrorHandling(ErrorNumber_TMeasCompError);
					Uart_SendString((uint8_t*)"     ->Error during Lib_Meas_Comp_Temp, status = ");
					Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
					Uart_SendString((uint8_t*)"\r\n");
				}
			}
		
			// Measure, compensate and send Acceleration via Uart
			status = Lib_Meas_Raw_Acceleration((uint32_t)(BITSIZE(Meas_RawAcceleration_NumSamples) | Meas_RawAcceleration_WBCCheckEnable | Meas_RawAcceleration_RDCheckEnable | Meas_RawAcceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range), MeasResults);
			if(status != COM_MEIF_EXT_STAT_OK)
			{
				ErrorHandling(ErrorNumber_AMeasError);
				Uart_SendString((uint8_t*)"     ->Error during Lib_Meas_Raw_Acceleration, status = ");
				Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
				Uart_SendString((uint8_t*)"\r\n");
			}
			else
			{
				// It's required to ALWAYS run the auto-offset-feature after an acceleration-measurement
				status = Lib_Comp_Auto_Acc_Offset((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range), (uint8_t)(DesiredAutoAccOffsetBlocksize-1), (uint16_t)(DesiredAutoAccOffsetMovingThreshold_LSBsqr), MeasResults);
				if(status > COM_MEIF_EXT_ACC_AO_TEMP_ERR)
				{
					ErrorHandling(ErrorNumber_AMeasError);
					Uart_SendString((uint8_t*)"     ->Error during Lib_Comp_Auto_Acc_Offset, status = ");
					Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
					Uart_SendString((uint8_t*)"\r\n");
				}
			}
			status = Lib_Comp_Acceleration((uint32_t)(Comp_Acceleration_CRCCheckEnable | MeasComp_Acceleration_Direction | MeasComp_Acceleration_Range | Comp_Acceleration_Offset_Source | COM_MEIF_EXT_CFG_TEMP_MEAS), Comp_Acceleration_Offset_Arg, MeasResults);	// temperature value from external
			if(status != COM_MEIF_EXT_STAT_OK)
			{
				ErrorHandling(ErrorNumber_ACompError);
				Uart_SendString((uint8_t*)"     ->Error during Lib_Comp_Acceleration, status = ");
				Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
				Uart_SendString((uint8_t*)"\r\n");
			}
			APS_Data.Thr_abs_dif = APS_Thr_ApsDif;
			APS_Data.Thr_max_min = APS_Thr_MaxMin;
			APS_Data.Num_samples = APS_NumSamples_PerPeriod;
			APS_Data.K1 = APS_K1;
			#if (APS_AcqType == _APS_ACQ_Type_HWAvg)
				status = Lib_Adv_APS_Sample((uint32_t)(BITSIZE(APS_RawAcceleration_NumSamples) | (APS_RawAcceleration_WBCCheckEnable*0) | (APS_RawAcceleration_RDCheckEnable*0) | (APS_RawAcceleration_CRCCheckEnable*0)), &measresult, APS_Sample_Array, &APS_Data);
			#endif
			#if (APS_AcqType == _APS_ACQ_Type_FIR)
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
				status = Lib_Adv_APS_Sample((uint32_t)((COM_MEIF_EXT_CFG_ACQ_TYPE) | (APS_FIRType * COM_MEIF_EXT_CFG_FIR_TYPE) | (APS_RawAcceleration_WBCCheckEnable*0) | (APS_RawAcceleration_RDCheckEnable*0) | (APS_RawAcceleration_CRCCheckEnable*0)), MeasResults, APS_Sample_Array, &APS_Data);
			#endif
			if(status != COM_MEIF_EXT_STAT_OK)
			{
				//ErrorHandling(status);
				Uart_SendString((uint8_t*)"     ->Error during Lib_Adv_APS_Sample, status = ");
				Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
				Uart_SendString((uint8_t*)"\r\n");
			}
			if(status & (LIB_ADV_APS_ADC_UNDERFLOW | LIB_ADV_APS_ADC_OVERFLOW | LIB_ADV_APS_OVERFLOW | LIB_ADV_APS_SAMP_PER_TOO_LOW))
				break;
			if(status == 0)
			{
				uint8_t DFT_PtA;
				printf("APS_Data.Sample_period: %d \r\n", APS_Data.Sample_period);
				printf("APS_Data.Thr_abs_dif: %d \r\n", APS_Data.Thr_abs_dif);
				printf("APS_Data.Thr_max_min: %d \r\n", APS_Data.Thr_max_min);
				printf("APS_Data.Phase_estimate: %d \r\n", APS_Data.Phase_estimate);
				printf("APS_Data.Frequency_estimate: %d \r\n", APS_Data.Frequency_estimate);
				printf("APS_Data.Num_samples: %d \r\n", APS_Data.Num_samples);
				printf("APS_Data.K1: %d \r\n", APS_Data.K1);
				// Calibrate the sampling-timer for 4s (32 * 125ms), which is used for time-measurment during post-processing. The number of MPRC-ticks per 4000ms can then be found in SFR STMR, bitfield STMRPERIOD
//				Lib_Calib_Sample_Timer(1, 4000);
//				
//				// STMR.STMRPERIOD is now updated with the right value. Actually starting the ST must happen earlier than 1 MPRC-cycle or 
//				//		later than 4 MPRC-cycles after a prior write to SFR STMR. STMR was written as last action by Lib_Calib_Sample_Timer, so it is safe to start the STMR directly afterwards
//				System_Controller->STMR_b.STMREN = 1;
//				
//				// RC1 and RC2 were pass above, call the DFT-function to get the center-frequency and to to the check of RC3 (DFT-peak-to-average) check afterwards
//				status = Lib_Adv_APS_DFT(APS_Sample_Array, &APS_Data, &DFT_PtA);
//				if(status != COM_MEIF_EXT_STAT_OK)
//				{
//					//ErrorHandling(status);
//					Uart_SendString((uint8_t*)"     ->Error during Lib_Adv_APS_DFT, status = ");
//					Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
//					Uart_SendString((uint8_t*)"\r\n");
//				}
//				
//				// Check RC3, only continue if it is pass
//				if(DFT_PtA >= APS_Thr_DFTPtA)
//				{
//					// Estimate phase of reference sample
//					status = Lib_Adv_APS_Sin_Param_Estimate(APS_Sample_Array, &APS_Data);
//					if(status != COM_MEIF_EXT_STAT_OK)
//					{
//						//ErrorHandling(status);
//						Uart_SendString((uint8_t*)"     ->Error during Lib_Adv_APS_DFT, status = ");
//						Uart_SendHex32(status, UART_SENDHEX_PREFIX_ENABLED);
//						Uart_SendString((uint8_t*)"\r\n");
//					}
//					
//					// Only continue if no OVERFLOW occured (only remaining error from above)
//					if(status == 0)
//					{
//						// Leave the loop with 'success'
//						APSSuccesfullyFinished = 1;
//						break;
//					}
//				}
			}
			Delay1ms(500);	
		}
		if(APSSuccesfullyFinished)
		{
			#if (APS_Type == _APS_Type_Extrapolation)
				printf("APS_Data.Sample_period: %d \r\n", APS_Data.Sample_period);
				printf("APS_Data.Thr_abs_dif: %d \r\n", APS_Data.Thr_abs_dif);
				printf("APS_Data.Thr_max_min: %d \r\n", APS_Data.Thr_max_min);
				printf("APS_Data.Phase_estimate: %d \r\n", APS_Data.Phase_estimate);
				printf("APS_Data.Frequency_estimate: %d \r\n", APS_Data.Frequency_estimate);
				printf("APS_Data.Num_samples: %d \r\n", APS_Data.Num_samples);
				printf("APS_Data.K1: %d \r\n", APS_Data.K1);
				sint32_t Np = (sint32_t)((8*APS_Data.Sample_period*(sint32_t)(APS_ExtrapolationAngle - APS_Data.Phase_estimate))/(sint32_t)APS_Data.Frequency_estimate);
				uint32_t Nt = (uint32_t)(((uint32_t)32768 * APS_Data.Sample_period) / (uint32_t)APS_Data.Frequency_estimate);
				uint32_t T0 = System_Controller->STMR_b.STMRPERIOD + APS_Data.Sample_period * 10 * 8 + APS_SampleTimeCorrection;
				
					// Find next occurence of extrapolation phase. The next timepoint must be before the current sampling-timer value, plus some buffer related to RF-Init-Delay
					uint32_t T_next = T0 - Np;
					while(T_next > (System_Controller->STMRCNT_b.CNTVAL - (uint16_t)(((uint16_t)((StartXtal_Delay100us+13+5)/10)+1)*94.5+1)))	// Add 1 as buffer if ST would decrease between here until Start_Init_Delay. Multiply with 94.5 to come from ms to max MPRC-cycles
						T_next -= Nt;
					
					// Calculate requried delay-time in [ms], then start init-delay-timer
					uint16_t T_delay = (uint16_t)(((uint32_t)(System_Controller->STMRCNT_b.CNTVAL-T_next) * 1000 * 4) / (System_Controller->STMR_b.STMRPERIOD));			
					
					// Only continue if we found a valid initial delay time
					if(T_delay >= ((StartXtal_Delay100us+13+5)/10) && T_delay < 5000)
					{
						// Start the init-delay

						
						// Send the RF-frame (only once!), wait for init-delay
						
					}
			#endif
			#if (APS_Type == _APS_Type_TimeAndPhase)
				// ********** Timing-critical section **********
				// From now on, timing should not be changed. Alternatvely the define 'RF_Tx_Start_Delay_MPRC' must be updated! 	
				// Timepoint in [MPRC ticks] at reference sample
				uint32_t T0 = System_Controller->STMR_b.STMRPERIOD + APS_Data.Sample_period * 10 * 8 + APS_SampleTimeCorrection;
				
				// Calculate time since reference-sample in [ms], then write it to APS_Data.Thr_abs_dif
				uint16_t T_delay = (uint16_t)(((uint32_t)(T0 - System_Controller->STMRCNT_b.CNTVAL + APS_RFTx_StartDelay_MPRC) * 1000 * 4) / (System_Controller->STMR_b.STMRPERIOD));
				APS_Data.Thr_abs_dif = T_delay;
				
				// Send the RF-frame (only once!)
				RFTransmitter_PrepareRFFrame(RFFrameType_TimeAndPhase, (uint32_t)(&APS_Data));
				uint32_t status = RFTransmitter_SendRFFrameBurst(1, 0, 0);
				// ********** End of timing-critical section **********
				
				ErrorHandling(status);
				#endif
		}
		else
		{
			// Send the RF-frame
		}
	#endif
}
