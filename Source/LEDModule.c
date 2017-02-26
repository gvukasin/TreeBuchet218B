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
#include "MagneticModule.h"

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

/*---------------------------- Module Variables ---------------------------*/

void TurnOnYellowLEDs(void)
{
	// Initialize hardware
	// Turn on
}

void TurnOnBlueLEDs(void)
{
	// Initialize hardware
	// Control blinking
}

void TurnOnTeamColorLEDs(bool TeamColor)
{
	// Initialize hardware
	// Turn on
}
