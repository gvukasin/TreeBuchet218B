/****************************************************************************
WireSensingModule
	Define functions for Wire Sensing from 2 RLC circuit
 
Events to receive:
					None
 
Events to post:
					None
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "ADMulti.h"

#include "MotorActionsModule.h"
#include "HallEffectModule.h"


/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define TicksPerMS 40000
#define BitsPerNibble 4
/*---------------------------- Module Variables ---------------------------*/


/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitRLCSensor

 Parameters
     void

 Returns
     void

 Description
			Initialize PE0 & PE1 for wire sensing(Analog Input)

 Author
     Team 16 
****************************************************************************/
void InitRLCSensor( void )
{
	
	//Enable PE0 and PE1 for analog input
	ADC_MultiInit(2);

}

/****************************************************************************
 Function
     ReadRLCSensor

 Description
     Write the reading from left and right RLC sensor to the array RLCReading

****************************************************************************/
void ReadRLCSensor(int RLCReading[2])
{
	uint32_t CurrentADRead[4];
	
  // Get the voltages from the input line	
  ADC_MultiRead(CurrentADRead);

	RLCReading[0] = CurrentADRead[0];
	RLCReading[1] = CurrentADRead[1];
}
