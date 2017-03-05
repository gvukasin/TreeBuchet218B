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
#include "MotorActionsModule.h"


/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define TicksPerMS 40000
#define BitsPerNibble 4
#define numbNibblesShifted 6
#define pinC6Mask 0xf0ffffff

// IR frequency codes
#define code800us 0x00 // 1250Hz (Green supply depot)
#define code690us 0x01 // 1450Hz (Bucket nav beacon)
#define code588us 0x02 // 1700Hz (Red nav beacon)
#define code513us 0x03 // 1950Hz (Red supply depot)
#define code455us 0x11 // 2200Hz (Green nav beacon)
#define codeInvalidIRFreq 0xff


/*---------------------------- Module Variables ---------------------------*/
static uint16_t IRSignalCode;
static uint16_t IRSignalPeriod_Tolerance = 10;
// Valid periods from IR beacons in us
// 455 us -> 2200 Hz (Green nav beacon)
// 513 us -> 1950 Hz (Red supply depot)
// 588 us -> 1700 Hz (Red nav beacon)
// 690 us -> 1450 Hz (Bucket nav beacon)
// 800 us -> 1250 Hz (Green supply depot)
static uint16_t ValidIRSignalPeriods[5] = {455, 513, 588, 690, 800};

static uint16_t PeriodBuffer[10];
static uint8_t CaptureIndex = 0;

// For IR Signal Frequency Capture
static uint32_t LastEdge;
static uint32_t CurrentEdge;
static uint32_t MeasuredIRSignalPeriod;
static uint8_t counter = 0;
static int IRSignalPeriod = 0;
static int IRSignalPeriodAddition = 0;
static int SampleSize = 10;

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
	
	//map bit 6's alternate function to WT1CCP0
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
	
	// grab captured value and calc period 
	CurrentEdge = HWREG(WTIMER0_BASE+TIMER_O_TAR);
	MeasuredIRSignalPeriod = CurrentEdge - LastEdge;
	MeasuredIRSignalPeriod = 1000*MeasuredIRSignalPeriod/TicksPerMS; // Unit: us
	
	//Write this captured period into the array
	PeriodBuffer[counter] = MeasuredIRSignalPeriod;
	
	//Move to the next element the next time
	counter ++;
		
	// Update the module level variable StagingAreaPeriod to be the average of the past ten catches
	// Update it every 10 interrupts
	if(counter >= SampleSize){
		counter = 0;
  }
}

/****************************************************************************
 Function
    GetIRCode

 Parameters
   None

 Returns
   uint8_t Code indicating the frequency of IR 
****************************************************************************/
uint8_t GetIRCode( void )
{
	uint8_t IRFreqCode;
	
	if ( (IRSignalPeriod < (ValidIRSignalPeriods[0] + IRSignalPeriod_Tolerance)) && (IRSignalPeriod > (ValidIRSignalPeriods[4] - IRSignalPeriod_Tolerance)) ){
		if ( (IRSignalPeriod > (ValidIRSignalPeriods[0] - IRSignalPeriod_Tolerance)) && (IRSignalPeriod < (ValidIRSignalPeriods[0] + IRSignalPeriod_Tolerance)) ){
			IRFreqCode = code455us;
		}

		else if ( (IRSignalPeriod > (ValidIRSignalPeriods[1] - IRSignalPeriod_Tolerance)) && (IRSignalPeriod < (ValidIRSignalPeriods[1] + IRSignalPeriod_Tolerance)) ){
			IRFreqCode = code513us;
		}

		else if ( (IRSignalPeriod > (ValidIRSignalPeriods[2] - IRSignalPeriod_Tolerance)) && (IRSignalPeriod < (ValidIRSignalPeriods[2] + IRSignalPeriod_Tolerance)) ){
			IRFreqCode = code588us;
		}

		else if ( (IRSignalPeriod > (ValidIRSignalPeriods[3] - IRSignalPeriod_Tolerance)) && (IRSignalPeriod < (ValidIRSignalPeriods[3] + IRSignalPeriod_Tolerance)) ){
			IRFreqCode = code690us;
		}

		else if ( (IRSignalPeriod > (ValidIRSignalPeriods[4] - IRSignalPeriod_Tolerance)) && (IRSignalPeriod < (ValidIRSignalPeriods[4] + IRSignalPeriod_Tolerance)) ){
			IRFreqCode = code800us;
		}
	}
	else{
		IRFreqCode = codeInvalidIRFreq;
	}
	
	return IRFreqCode;
}

