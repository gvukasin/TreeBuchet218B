/****************************************************************************
Magnetic Module
	Define functions for Wire Sensing & Stage Sensing
 
Events to receive:

 
Events to post:

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

#include "ActionService.h"
#include "MotorActionsModule.h"
#include "HallEffectModule.h"


/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define TicksPerMS 40000
#define BitsPerNibble 4
//#define numbNibblesShifted 6
//#define pinC6Mask 0xf0ffffff
//#define ALIGN_BEACON 0x20 

//#define STOP 0x00
/*---------------------------- Module Variables ---------------------------*/
//static uint32_t LastCapture;

//static uint32_t LastCapture;
//static uint32_t ThisCapture;
//static uint32_t MeasuredSignalSpeedHz;
//static uint32_t AveragedMeasuredSignalSpeedHz = 0;
//static uint32_t SpeedAddition = 0;
//static uint32_t DesiredFreqLOBoundary;
//static uint32_t DesiredFreqHIBoundary;
//static uint32_t MeasuredSignalPeriod;
//static uint8_t counter = 1;

////Initialize freq boundaries for IR beacon
//static uint32_t	DesiredFreqLOBoundary = lab8BeaconFreqHz - 0.2*lab8BeaconFreqHz;
//static uint32_t	DesiredFreqHIBoundary = lab8BeaconFreqHz + 0.2*lab8BeaconFreqHz;

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
			Initialize PC7 for stage sensing

 Author
     Team 16 
****************************************************************************/
void InitRLCSensor( void )
{
	
	//Enable PE0 and PE1 for analog input
	ADC_MultiInit(2);
	
//	//Enable the clock to Port C	
//	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
//	while (((HWREG(SYSCTL_PRGPIO)) & SYSCTL_PRGPIO_R4) != SYSCTL_PRGPIO_R4){
//		;
//  }
//	// Enable Pin 7 as digital input
//	HWREG(GPIO_PORTC_BASE+GPIO_O_DEN) |= GPIO_PIN_7; 
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
	// printf("\r\nLeft = %d,%d,Right = %d,%d\r\n",CurrentADRead[0],RLCReading[0],CurrentADRead[1],RLCReading[1]);
}
	

