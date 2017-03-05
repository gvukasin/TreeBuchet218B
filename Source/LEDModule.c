/****************************************************************************
 Module
   LEDModule.c

 Description
	 Provide functions to control all LED related actions
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "RobotTopSM.h"
#include "ShootingSubSM.h"
#include "SPIService.h"
#include "HallEffectModule.h"
#include "LEDModule.h"
#include "ShiftRegisterWrite.h"

// the common headers for C99 types 
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "BITDEFS.H"

/*----------------------------- Module Defines ----------------------------*/
#define ON 1
#define GREEN 0
#define YELLOW_ON 0x40 //01000000
#define RED_ON 0x08 //00001000
#define GREEN_ON 0x10 //00010000
#define BLUE_ON 0x04 //00000100
#define LEDS_OFF 0x00


/*----------------------------- Module Functions ----------------------------*/
static void InitializeLEDHardware(void);

/*---------------------------- Module Variables ---------------------------*/


/*----------------------------- Public Functions ----------------------------*/
void TurnOnOffYellowLEDs(bool ONorOFF, bool TeamColor) 
{
	// Initialize hardware
  InitializeLEDHardware();
	
	// Turn on/off
	if(ONorOFF == ON)
	{
		if (TeamColor == GREEN)
			SR_Write((YELLOW_ON|GREEN_ON));
		else 
			SR_Write((YELLOW_ON|RED_ON));
	}
	
	else //Leave only team color led ON
	{
		if (TeamColor == GREEN)
			SR_Write(GREEN_ON);
		else 
			SR_Write(RED_ON);
	}
}

void TurnOnOffBlueLEDs(bool ONorOFF, bool TeamColor)
{
	// Initialize hardware
	InitializeLEDHardware();

	// Turn on/off
		// Initialize hardware
  InitializeLEDHardware();
	
	// Turn on/off
	if(ONorOFF == ON)
	{
		if (TeamColor == GREEN)
			SR_Write((BLUE_ON|GREEN_ON));
		else 
			SR_Write((BLUE_ON|RED_ON));
	}
	
	else //Leave only team color led ON
	{
		if (TeamColor == GREEN)
			SR_Write(BLUE_ON);
		else 
			SR_Write(BLUE_ON);
	}
}

void TurnOnOFFTeamColorLEDs(bool ONorOFF, bool TeamColor)
{
	// Initialize hardware
	InitializeLEDHardware();

	// Turn on/off
	if(ONorOFF == ON)
	{
		if (TeamColor == GREEN)
				SR_Write(GREEN_ON);
		else 
				SR_Write(RED_ON);
	}
	else
		SR_Write(LEDS_OFF);
}

/*----------------------------- Module Functions ----------------------------*/
static void InitializeLEDHardware(void)
{
	// Init shift register
	SR_Init();
}
