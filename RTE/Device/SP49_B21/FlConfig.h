//********************* Flash configuration header-file

//****************************** User configurable section *************************
#define FL_CONFIG_NUM_BOOT_SEC	3			//Number of available boot-sectors. Valid: [1 ... 7]
#define FL_CONFIG_NUM_UCS_SEC	1				//Number of available user-configuration-sectors. Valid: [0 ... 6]
//**********************************************************************************

//****************************** Calculated variables - DO NOT CHANGE! *************
// Other defines, do not change!
#define FL_CONFIG_START_ADDR	(0x10000800)
#define FL_CONFIG_SECT_SIZE		(1024)
#define FL_CONFIG_SIZE				(19 * FL_CONFIG_SECT_SIZE)
#define FL_CONFIG_NUM_VAR_SEC	(6)
#define FL_CONFIG_CONFIGLINE_SIZE	(0x20)

#define FL_CONFIG_BOOT_SECT_START	(2)
#define FL_CONFIG_BOOT_SECT_END		(FL_CONFIG_BOOT_SECT_START + FL_CONFIG_NUM_BOOT_SEC - 1)
#if (FL_CONFIG_NUM_UCS_SEC > 0)
	#define FL_CONFIG_UCS_SECT_START	(FL_CONFIG_BOOT_SECT_START + FL_CONFIG_NUM_BOOT_SEC)
	#define FL_CONFIG_UCS_SECT_END	(FL_CONFIG_UCS_SECT_START + FL_CONFIG_NUM_UCS_SEC - 1)
#elif (FL_CONFIG_NUM_UCS_SEC == 0)
	#define FL_CONFIG_UCS_SECT_START	(0)
	#define FL_CONFIG_UCS_SECT_END 		(0)
#endif
#define FL_CONFIG_CODE_SECT_START (FL_CONFIG_BOOT_SECT_START + FL_CONFIG_NUM_BOOT_SEC + FL_CONFIG_NUM_UCS_SEC)
#define FL_CONFIG_CODE_SECT_END		(10)

#if (FL_CONFIG_NUM_BOOT_SEC < 1)
	#error "At least one boot-sector is required!"
#endif
#if ((FL_CONFIG_NUM_BOOT_SEC+FL_CONFIG_NUM_UCS_SEC) > (FL_CONFIG_NUM_VAR_SEC+1))
	#error "Maximum number of boot- plus UCS-sectors exceeded!"
#endif
	
#define FL_CONFIG_BOOT_START_ADDR		(FL_CONFIG_START_ADDR + FL_CONFIG_CONFIGLINE_SIZE)
#define FL_CONFIG_BOOT_SIZE					(FL_CONFIG_NUM_BOOT_SEC * FL_CONFIG_SECT_SIZE - FL_CONFIG_CONFIGLINE_SIZE)
#define FL_CONFIG_UCS_START_ADDR		(FL_CONFIG_START_ADDR + (FL_CONFIG_NUM_BOOT_SEC * FL_CONFIG_SECT_SIZE))
#define FL_CONFIG_UCS_SIZE					(FL_CONFIG_NUM_UCS_SEC * FL_CONFIG_SECT_SIZE)
#define FL_CONFIG_CODE_START_ADDR		(FL_CONFIG_START_ADDR + (FL_CONFIG_NUM_BOOT_SEC * FL_CONFIG_SECT_SIZE) + (FL_CONFIG_NUM_UCS_SEC * FL_CONFIG_SECT_SIZE))
#define FL_CONFIG_CODE_SIZE					(FL_CONFIG_SIZE - FL_CONFIG_CONFIGLINE_SIZE - FL_CONFIG_BOOT_SIZE - FL_CONFIG_UCS_SIZE)
//**********************************************************************************
