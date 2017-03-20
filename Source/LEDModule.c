/****************************************************************************
 Module
   LEDModule.c

 Description
	 Provide functions to control all LED related actions
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/

//#define TEST 

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
#define OFF 0
#define GREEN 1
#define YELLOW_ON 0x40 //01000000
#define RED_ON 0x10 //00010000
#define GREEN_ON 0x08 //00001000
#define BLUE_ON 0x04 //00000100
#define LEDS_OFF 0x00


/*----------------------------- Module Functions ----------------------------*/
static void InitializeLEDHardware(void);

/*----------------------------- Public Functions ----------------------------*/
/****************************************************************************************
	TurnOnOffYellowLEDs
		Turn on or off construction LEDs, while maintaining TeamColor LEDs
*******************************************************************************************/

void TurnOnOffYellowLEDs(bool ONorOFF, bool TeamColor) 
{
	// Initialize hardware
  InitializeLEDHardware();
	
	// If ONorOFF is true
	if(ONorOFF == ON)
	{
		// If TeamColor is Green
		if (TeamColor == GREEN)
			// Write to SR green and yellow LED bits
			SR_Write((YELLOW_ON|GREEN_ON));
		// If TeamColor is Red
		else 
			// Write to SR red and yellow LED bits
			SR_Write((YELLOW_ON|RED_ON));
	}
	// Else If ONorOFF is false
	else //Leave only team color led ON
	{
		// If TeamColor is green, write green bits to SR
		if (TeamColor == GREEN)
			SR_Write(GREEN_ON);
		// Else If TeamColor is red, write red bits to SR
		else 
			SR_Write(RED_ON);
	}
}

/****************************************************************************************
	TurnOnOffBlueLEDs
		Turn on or off communication LEDs, while maintaining TeamColor LEDs
*******************************************************************************************/
void TurnOnOffBlueLEDs(bool ONorOFF, bool TeamColor)
{
	// Initialize hardware
	InitializeLEDHardware();
	
	// If ONorOFF is true
	if(ONorOFF == ON)
	{
		// If TeamColor is green
		if (TeamColor == GREEN)
			// Write to SR green and blue bits
			SR_Write((BLUE_ON|GREEN_ON));
		// Else If TeamColor is red
		else 
			// Write to SR red and blue bits
			SR_Write((BLUE_ON|RED_ON));
	}
	
	// Else If ONorOFF is false
	else //Leave only team color led ON
	{
		// If TeamColor is green, write to SR green bits
		if (TeamColor == GREEN)
			SR_Write(GREEN_ON);
		// Else If TeamColor is red, write to SR red bits
		else 
			SR_Write(RED_ON);
	}
}

/****************************************************************************************
	TurnOnOffTeamColorLEDs
		Turn on or off TeamColor LEDs, chosing either red or green
*******************************************************************************************/
void TurnOnOFFTeamColorLEDs(bool ONorOFF, bool TeamColor)
{
	// Initialize hardware
	InitializeLEDHardware();

	// If ONorOFF is true
	if(ONorOFF == ON)
	{
		// If TeamColor is Green
		if (TeamColor == GREEN)
				// Write to SR bits that turn on Green LEDS
				SR_Write(GREEN_ON);
		// Else if TeamColor is Red
		else 
			// Write to SR bits that turn on Red LEDS
				SR_Write(RED_ON);
	}
	// else 
	else
		// Turn off red and gree LEDS
		SR_Write(LEDS_OFF);
}
/****************************************************************************************
	InitializeLEDHardware
		Initialize hardware needed for LEDs to be turned on
*******************************************************************************************/
/*----------------------------- Module Functions ----------------------------*/
static void InitializeLEDHardware(void)
{
	// Init shift register
	SR_Init();
}
