/****************************************************************************
IRBeacon Module
	Define the Initialization and ISR for the IR sensing interrupt
	Whenever beacon is detected, post event BeaconSensed
 
Events to receive:
  None
 
Events to post:
	TapeSensed (to ActionService)
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

#include "ActionService.h"
#include "TapeModule.h"
#include "MotorActionsModule.h"


/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define TicksPerMS 40000
#define lab8BeaconFreqHz 1950
#define BitsPerNibble 4
#define numbNibblesShifted 6
#define pinC6Mask 0xf0ffffff
#define ALIGN_BEACON 0x20 

#define STOP 0x00
/*---------------------------- Module Variables ---------------------------*/
static uint32_t LastCapture;

static uint32_t LastCapture;
static uint32_t ThisCapture;
static uint32_t MeasuredSignalSpeedHz;
static uint32_t AveragedMeasuredSignalSpeedHz = 0;
static uint32_t SpeedAddition = 0;
static uint32_t DesiredFreqLOBoundary;
static uint32_t DesiredFreqHIBoundary;
static uint32_t MeasuredSignalPeriod;
static uint8_t counter = 1;

//Initialize freq boundaries for IR beacon
static uint32_t	DesiredFreqLOBoundary = lab8BeaconFreqHz - 0.2*lab8BeaconFreqHz;
static uint32_t	DesiredFreqHIBoundary = lab8BeaconFreqHz + 0.2*lab8BeaconFreqHz;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitInputCaptureForIRDetection

 Parameters
     void

 Returns
     void

 Description
			Initialization for interrupt response for input capture

 Author
     Team 16 
****************************************************************************/
void InitInputCaptureForIRDetection( void )
{
	//Start by enabling the clock to the timer (Wide Timer 1)
	HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1;
	
	//Enable the clock to Port C	
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
	
	//Make sure that timer (Timer A) is disabled before configuring
	HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
	
	//Set it up in 32bit wide
	HWREG(WTIMER1_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;
	
	//Initialize the Interval Load register to 0xffff.ffff
	HWREG(WTIMER1_BASE + TIMER_O_TAILR) = 0xffffffff;
	
	//Set up timer A in capture mode (TAMR=3, TAAMS = 0), for edge time (TACMR = 1) and up-counting (TACDIR = 1)
	HWREG(WTIMER1_BASE + TIMER_O_TAMR) =
	(HWREG(WTIMER1_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) | (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
	
	//Set the event to rising edge
	HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;
	
	//Set up the port to do the capture  -- we will use C6 because we are using wide timer 1A
	HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= BIT6HI;
	
	//map bit 4's alternate function to WT1CCP0
	HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) & pinC6Mask) + (7 << (BitsPerNibble*numbNibblesShifted));
	
	//Enable pin 6 on Port C for digital I/O
	HWREG(GPIO_PORTC_BASE + GPIO_O_DEN) |= BIT6HI;
	
	//make pin 6 on Port C into an input
	HWREG(GPIO_PORTC_BASE + GPIO_O_DIR) &= BIT6LO;
	
	//Enable a local capture interrupt
	HWREG(WTIMER1_BASE + TIMER_O_IMR) |= TIMER_IMR_CAEIM;
	
	//Enable the Timer A in Wide Timer 1 interrupt in the NVIC (wide timer 1A <--> interrupt 96)
	HWREG(NVIC_EN3) |= BIT0HI;
	
	//Make sure interrupts are enabled globally
	__enable_irq();
	
	printf("\r\nGot through IR interrupt init\r\n");
	
}

/****************************************************************************
 Function
     EnableIRInterrupt

 Description
     Define the interrupt response routine
****************************************************************************/
void EnableIRInterrupt(void)
{
	//Kick timer off by enabling timer and enabling the timer to stall while stopped by the debugger
	HWREG(WTIMER1_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}
	

/****************************************************************************
 Function
     InputCaptureISR

 Parameters
     void

 Returns
     void

 Description
			Interrupt response for input capture --> 
			will give us the period of the detected IR signal

 Author
     Team 16 
****************************************************************************/ 
void InputCaptureForIRDetectionResponse( void )  
{
	//Clear the source of the interrupt, the input capture event
	HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
	
	//Grab the captured value 
	ThisCapture = HWREG(WTIMER1_BASE + TIMER_O_TAR);
	MeasuredSignalPeriod = ThisCapture - LastCapture;
	
	//Update LastCapture to prepare for the next edge
	LastCapture = ThisCapture;
	
	//Calculate measured signal speed 
	//and keep count of the addition of the speeds to later calculate the average
	MeasuredSignalSpeedHz = (1000*TicksPerMS)/MeasuredSignalPeriod;
	SpeedAddition += MeasuredSignalSpeedHz;
	
	//Check to see if we have found the beacon and if we have sent a stop event
	if((counter>10) && (MeasuredSignalSpeedHz > DesiredFreqLOBoundary) && (MeasuredSignalSpeedHz < DesiredFreqHIBoundary)) //Post STOP event to ActionService
	{
		//Disable interrupt
		HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
		//Command to stop
		stop();
	}
	else // keep looking for tape and update averaged measured signal speed
	{
		AveragedMeasuredSignalSpeedHz = SpeedAddition/counter;
		SpeedAddition = 0;
		ES_Event ThisEvent;
		ThisEvent.EventType = IRBeaconSensed;
		ThisEvent.EventParam = ALIGN_BEACON;
		PostActionService(ThisEvent);
	}
	counter = counter + 1;
}
