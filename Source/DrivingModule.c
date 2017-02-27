/****************************************************************************
 Module
   DrivingModule.c

 Description
	 Handles driving to a wire, driving on a wire, and driving to reload
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "RobotTopSM.h"
#include "ShootingSubSM.h"
#include "SPIService.h"
#include "HallEffectModule.h"
#include "DrivingModule.h"

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


void DriveOnWire(void) //TODO
{
	
}

void Drive2Wire(void)
{
	// FOR 2/26 CHECKOFF RUN
	// drive randomly in 4 directions until wire is sensed
	
	// FOR PROJECT RUN
	// determine if on red or green side
	// align with appropriate IR signal
	// drive towards wire accordingly
}

void Drive2Reload(void)
{
	// determine if on red or green side
	// align with corresponding IR signal
	// drive to reloading station (might need limit switch to know when to stop)
}
