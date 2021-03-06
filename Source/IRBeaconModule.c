/****************************************************************************
IRBeacon Module
	Define the Initialization and ISR for the IR sensing interrupt
	Whenever beacon is detected, post event BeaconSensed
 
Events to receive:
  None
 
Events to post:
	IRBeaconSensed
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
#include "MotorActionsModule.h"


/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define TicksPerMS 40000
#define BitsPerNibble 4
#define numbNibblesShifted 6
#define pinC6Mask 0xf0ffffff
#define pinD2Mask 0xfffff0ff

// IR frequency codes
#define code800us 0x00 // 1250Hz (Green supply depot)
#define code690us 0x01 // 1450Hz (Bucket nav beacon)
#define code588us 0x02 // 1700Hz (Red nav beacon)
#define code513us 0x03 // 1950Hz (Red supply depot)
#define code455us 0x04 // 2200Hz (Green nav beacon)
#define codeInvalidIRFreq 0xff


/*---------------------------- Module Variables ---------------------------*/
// For Front IR Signal Frequency Capture
// The "front" of the robot receives balls
static uint16_t Front_IRSignalCode;
static uint16_t Front_IRSignalCode_Tolerance = 10;
static uint32_t Front_LastEdge;
static uint32_t Front_CurrentEdge;
static uint32_t Front_MeasuredIRSignalPeriod;
static uint8_t Front_counter = 0;
static int Front_IRSignalPeriod = 0;
static int Front_IRSignalPeriodAddition = 0;
static uint16_t Front_PeriodBuffer[5];

// For Back IR Signal Frequency Capture
// The "back" of the robot shoots balls
static uint16_t Back_IRSignalCode;
static uint16_t Back_IRSignalCode_Tolerance = 10;
static uint32_t Back_LastEdge;
static uint32_t Back_CurrentEdge;
static uint32_t Back_MeasuredIRSignalPeriod;
static uint8_t Back_counter = 0;
static int Back_IRSignalPeriod = 0;
static int Back_IRSignalPeriodAddition = 0;
static uint16_t Back_PeriodBuffer[5];

static int SampleSize = 4;
static uint8_t CaptureIndex = 0;

// Valid periods from IR beacons in us
// 455 us -> 2200 Hz (Green nav beacon)
// 513 us -> 1950 Hz (Red supply depot)
// 588 us -> 1700 Hz (Red nav beacon)
// 690 us -> 1450 Hz (Bucket nav beacon)
// 800 us -> 1250 Hz (Green supply depot)

//static uint16_t ValidIRSignalPeriods[5] = {455, 513, 588, 690, 800};
static uint16_t ValidIRSignalPeriods[5] = {800, 690, 588, 513, 455};

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitInputCaptureForFrontIRDetection

 Parameters
     void

 Returns
     void

 Description
			Initialization for interrupt response for input capture

 Author
     Team 16 
****************************************************************************/
void InitInputCaptureForFrontIRDetection( void )
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
     InitInputCaptureForBackIRDetection

 Parameters
     void

 Returns
     void

 Description
			Initialization for interrupt response for input capture

 Author
     Team 16 
****************************************************************************/
void InitInputCaptureForBackIRDetection( void )
{
	//Start by enabling the clock to the timer (Wide Timer 3A)
	HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R3;
	
	//Enable the clock to Port D
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
	
	//Make sure that timer (Timer A) is disabled before configuring
	HWREG(WTIMER3_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
	
	printf("\r\n See me1");
	
	//Set it up in 32bit wide
	HWREG(WTIMER3_BASE + TIMER_O_CFG) = TIMER_CFG_16_BIT;
	
	//Initialize the Interval Load register to 0xffff.ffff
	HWREG(WTIMER3_BASE + TIMER_O_TAILR) = 0xffffffff;
	
	//Set up timer  Ain capture mode (TAMR=3, TAAMS = 0), for edge time (TACMR = 1) and up-counting (TACDIR = 1)
	HWREG(WTIMER3_BASE + TIMER_O_TAMR) =
	(HWREG(WTIMER3_BASE + TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) | (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
	
	//Set the event to rising edge
	HWREG(WTIMER3_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;
	
	//Set up the port to do the capture  -- we will use D2 because we are using wide timer 3A
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= BIT2HI;
	
	//map bit 2's alternate function to WT1CCP0
	HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) & pinD2Mask) + (7 << (BitsPerNibble*2));
	
	//Enable pin 2 on Port D for digital I/O
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= BIT2HI;
	
	//make pin 2 on Port D into an input
	HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) &= BIT2LO;
	
	//Enable a local capture interrupt
	HWREG(WTIMER3_BASE + TIMER_O_IMR) |= TIMER_IMR_CAEIM;
	
	// enable the Timer A in Wide Timer 3 interrupt in the NVIC
	// it is interrupt number 100 so appears in EN3 at bit 4
	HWREG(NVIC_EN3) |= BIT4HI;
	
	//Make sure interrupts are enabled globally
	__enable_irq();
	
}

/****************************************************************************
 Function
     EnableFrontIRInterrupt

 Description
     Define the interrupt response routine
****************************************************************************/
void EnableFrontIRInterrupt(void)
{
	//Kick timer off by enabling timer and enabling the timer to stall while stopped by the debugger
	HWREG(WTIMER1_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}

// SEE ME (change to Wide Timer 3 subtimer A)
/****************************************************************************
 Function
     EnableBackIRInterrupt

 Description
     Define the interrupt response routine
****************************************************************************/
void EnableBackIRInterrupt(void)
{
	//Kick timer off by enabling timer and enabling the timer to stall while stopped by the debugger
	HWREG(WTIMER3_BASE + TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}

/****************************************************************************
 Function
		 InputCaptureForFrontIRDetection

 Description
			Interrupt response for input capture --> 
			will give us the period of the detected IR signal

 Author
     Team 16 
****************************************************************************/ 
void InputCaptureForFrontIRDetection( void )  
{
	//Clear the source of the interrupt, the input capture event
	HWREG(WTIMER1_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
	
	// grab captured value and calc period 
	Front_CurrentEdge = HWREG(WTIMER1_BASE + TIMER_O_TAR);
	
	Front_MeasuredIRSignalPeriod = Front_CurrentEdge - Front_LastEdge;
	Front_MeasuredIRSignalPeriod = 1000*Front_MeasuredIRSignalPeriod/TicksPerMS; // Unit: us
	
	//Write this captured period into the array
	Front_PeriodBuffer[Front_counter] = Front_MeasuredIRSignalPeriod;
	
	//Move to the next element the next time
	Front_counter++;
		
	// Update the module level variable StagingAreaPeriod to be the average of the past ten catches
	// Update it every 10 interrupts
	if(Front_counter >= SampleSize)
		Front_counter = 0;
	
	// update LastCapture to prepare for the next edge
	Front_LastEdge = Front_CurrentEdge;

	// SEE ME
	// Running average calculation. Can use this to make calculation more robust
	// Front_IRSignalPeriodAddition  = (90*Front_IRSignalPeriodAddition + 10*Front_MeasuredIRSignalPeriod)/100;
	// Front_IRSignalPeriod = Front_IRSignalPeriodAddition
}

/****************************************************************************
 Function
		 InputCaptureForBackIRDetection

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
void InputCaptureForBackIRDetection( void )  //ISR
{
	//Clear the source of the interrupt, the input capture event
	HWREG(WTIMER3_BASE + TIMER_O_ICR) = TIMER_ICR_CAECINT;
	
	// grab captured value and calc period 
	Back_CurrentEdge = HWREG(WTIMER3_BASE + TIMER_O_TAR);
	
	Back_MeasuredIRSignalPeriod = Back_CurrentEdge - Back_LastEdge;
	Back_MeasuredIRSignalPeriod = 1000*Back_MeasuredIRSignalPeriod/TicksPerMS; // Unit: us
	
	//Write this captured period into the array
	Back_PeriodBuffer[Back_counter] = Back_MeasuredIRSignalPeriod;
	
	//Move to the next element the next time
	Back_counter++;
		
	// Update the module level variable StagingAreaPeriod to be the average of the past ten catches
	// Update it every 10 interrupts
	if(Back_counter >= SampleSize){
		Back_counter = 0;
  }
	
	// update LastCapture to prepare for the next edge
	Back_LastEdge = Back_CurrentEdge;

	// SEE ME
	// Running average calculation. Can use this to make calculation more robust
	// Back_IRSignalPeriodAddition  = (90*Back_IRSignalPeriodAddition + 10*Back_MeasuredIRSignalPeriod)/100;
	// Back_IRSignalPeriod = Back_IRSignalPeriodAddition
}

/****************************************************************************
 Function
    Front_GetIRCodeSingle

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/

uint8_t Front_GetIRCodeSingle( uint16_t thePeriod )
{
		if( (thePeriod < (ValidIRSignalPeriods[0] + Front_IRSignalCode_Tolerance)) && (thePeriod > (ValidIRSignalPeriods[4] - Front_IRSignalCode_Tolerance)) ){
			if ( (thePeriod > (ValidIRSignalPeriods[0] - Front_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[0] + Front_IRSignalCode_Tolerance)) ){
				Front_IRSignalCode = code800us;
				printf("\r\n code800 %x",Front_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[1] - Front_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[1] + Front_IRSignalCode_Tolerance)) ){
				Front_IRSignalCode = code690us;
				printf("\r\n code690 %x",Front_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[2] - Front_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[2] + Front_IRSignalCode_Tolerance)) ){
				Front_IRSignalCode = code588us;
			 printf("\r\n code588 %x",Front_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[3] - Front_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[3] + Front_IRSignalCode_Tolerance)) ){
				Front_IRSignalCode = code513us;
				printf("\r\n code513 %x",Front_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[4] - Front_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[4] + Front_IRSignalCode_Tolerance)) ){
				Front_IRSignalCode = code455us;
				printf("\r\n code455 %x",Front_IRSignalCode);
			}
		}
		else{
			Front_IRSignalCode = codeInvalidIRFreq;
			printf("\r\n Invalid, period = %d\r\n",Front_MeasuredIRSignalPeriod);
		}	

	//printf("\r\n code %x",Front_IRSignalCode);
	return Front_IRSignalCode;
}

/****************************************************************************
 Function
    Back_GetIRCodeSingle

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/

uint8_t Back_GetIRCodeSingle( uint16_t thePeriod )
{
		if( (thePeriod < (ValidIRSignalPeriods[0] + Back_IRSignalCode_Tolerance)) && (thePeriod > (ValidIRSignalPeriods[4] - Back_IRSignalCode_Tolerance)) ){
			if ( (thePeriod > (ValidIRSignalPeriods[0] - Back_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[0] + Back_IRSignalCode_Tolerance)) ){
				Back_IRSignalCode = code800us;
				//printf("\r\n code800 %x",Back_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[1] - Back_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[1] + Back_IRSignalCode_Tolerance)) ){
				Back_IRSignalCode = code690us;
				//printf("\r\n code690 %x",Back_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[2] - Back_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[2] + Back_IRSignalCode_Tolerance)) ){
				Back_IRSignalCode = code588us;
				//printf("\r\n code588 %x",Back_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[3] - Back_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[3] + Back_IRSignalCode_Tolerance)) ){
				Back_IRSignalCode = code513us;
				//printf("\r\n code513 %x",Back_IRSignalCode);
			}

			else if ( (thePeriod > (ValidIRSignalPeriods[4] - Back_IRSignalCode_Tolerance)) && (thePeriod < (ValidIRSignalPeriods[4] + Back_IRSignalCode_Tolerance)) ){
				Back_IRSignalCode = code455us;
				//printf("\r\n code455 %x",Back_IRSignalCode);
			}
		}
		else{
			Back_IRSignalCode = codeInvalidIRFreq;
			//printf("\r\n code70 %x",Back_IRSignalCode);
		}	

	//printf("\r\n code %x",Back_IRSignalCode);
	return Back_IRSignalCode;
}

/****************************************************************************
 Function
    Front_GetIRCode

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/

uint8_t Front_GetIRCode(void){
	uint8_t i = 0;
	uint8_t returnCode = Front_GetIRCodeSingle(Front_PeriodBuffer[0]);
//	printf("\r\n-------------Code=%u------------\r\n",returnCode);
//	printf("\r\n!!!!!!!!!!!Period0=%d\r\n",Front_PeriodBuffer[0]);
//	printf("\r\n!!!!!!!!!!!Period1=%d\r\n",Front_PeriodBuffer[1]);
//	printf("\r\n!!!!!!!!!!!Period2=%d\r\n",Front_PeriodBuffer[2]);
//	printf("\r\n!!!!!!!!!!!Period3=%d\r\n",Front_PeriodBuffer[3]);
//	printf("\r\n!!!!!!!!!!!Period4=%d\r\n",Front_PeriodBuffer[4]);
//	printf("\r\n!!!!!!!!!!!%u\r\n",returnCode);
//	printf("\r\n!!!!!!!!!!!Counter=%u\r\n",Front_counter);
	
	for(i = 1; i < SampleSize; i++){
		if(Front_GetIRCodeSingle(Front_PeriodBuffer[i]) != returnCode){
			return codeInvalidIRFreq;
		}
	}
	//printf("\r\n--------------ValidCode--------%u\r\n",returnCode);
	return returnCode;
	
}

/****************************************************************************
 Function
    Back_GetIRCode

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/

uint8_t Back_GetIRCode(void){
	uint8_t i = 0;
	uint8_t returnCode = Back_GetIRCodeSingle(Back_PeriodBuffer[0]);
	//printf("\r\n%u\r\n",returnCode);
  //	printf("\r\n!!!!!!!!!!!Period0=%d\r\n",Back_PeriodBuffer[0]);
  //	printf("\r\n!!!!!!!!!!!Period1=%d\r\n",Back_PeriodBuffer[1]);
  //	printf("\r\n!!!!!!!!!!!Period2=%d\r\n",Back_PeriodBuffer[2]);
  //	printf("\r\n!!!!!!!!!!!Period3=%d\r\n",Back_PeriodBuffer[3]);
  //	printf("\r\n!!!!!!!!!!!Period4=%d\r\n",Back_PeriodBuffer[4]);
	
	for(i = 1; i < SampleSize; i++){
		if(Back_GetIRCodeSingle(Back_PeriodBuffer[i]) != returnCode){
			return codeInvalidIRFreq;
		}
	}
	
	return returnCode;
}
