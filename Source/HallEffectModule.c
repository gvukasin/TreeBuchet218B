/****************************************************************************
HallEffectModule
	Define functions for staging area sensing
 
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
#define BitsPerNibble 4
#define TicksPerMS 40000

/*---------------------------- Module Variables ---------------------------*/
//static ES_Event HallEffectEdgeDetected;
static uint16_t StagingAreaCode;
static uint8_t StagingAreaPeriod_Tolerance = 20;
static uint16_t StagingAreaPeriods[16] = {1333, 1277, 1222, 1166, 1111, 1055, 1000, 944, 889, 833, 778, 722, 667, 611, 556, 500};

// For Staging Area Frequency Capture
static uint32_t LastEdge;
static uint32_t CurrentEdge;
static uint32_t MeasuredStagingAreaPeriod;
static uint8_t counter = 0;
static int StagingAreaPeriod = 0;
static int StagingAreaPeriodAddition = 0;

// staging area frequency codes
uint16_t code1333us = 0000;
uint16_t code1277us = 0001;
uint16_t code1222us = 0010;
uint16_t code1166us = 0011;
uint16_t code1111us = 0100;
uint16_t code1055us = 0101;
uint16_t code1000us = 0110;
uint16_t code944us = 0111;
uint16_t code889us = 1000;
uint16_t code833us = 1001;
uint16_t code778us = 1010;
uint16_t code722us = 1011;
uint16_t code667us = 1100;
uint16_t code611us = 1101;
uint16_t code556us = 1110;
uint16_t code500us = 1111;
uint16_t codeInvalidStagingArea = 0xff;


/*---------------------------- Module Functions ---------------------------*/
void InitStagingAreaISR( void );
void EnableStagingAreaISR( void );
void StagingAreaISR( void );

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
    InitStagingAreaISR

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
void InitStagingAreaISR( void )
{
	// enable Wide Timer 0 
	HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
	
	//enable clock to port C
	HWREG(SYSCTL_RCGCGPIO) |=SYSCTL_RCGCGPIO_R2;
		
	// make sure timer A is disabled before configuring
	HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
	
	// set up time in 32bit wide mode (non-concatinated mode)
	HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
	
	// use full 32 bit count by initializing the Interval Load Register to all ones 
	HWREG(WTIMER0_BASE+TIMER_O_TAILR) = 0xffffffff;
	
	// set up timer A in capture mode (TAMR=3,TAAMS=0), for edge time (TACMR=1), 
	// and upcounting (TACDIR = 1);
	HWREG(WTIMER0_BASE+TIMER_O_TAMR) = (HWREG(WTIMER0_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS)|
	(TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
	
	// set event to rising edge by modifying TAEVENT bits in GPTMCTL to 00 (clear bits)
	HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;
	
	// set up port to do capture by setting alt function for C4
	HWREG(GPIO_PORTC_BASE+GPIO_O_AFSEL) |= BIT4HI;
	
	// map bit 4's alt function to WT0CCP0 (7), by clearing nibble then
	// shifting 7 to fourth nibble 
	HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) = (HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) & 0xfff0ffff) + (7<<(4*BitsPerNibble));
	
	// set pin C4 to digital
	HWREG(GPIO_PORTC_BASE+GPIO_O_DEN) |= BIT4HI;
	
	// set pin C4 to input
	HWREG(GPIO_PORTC_BASE+GPIO_O_DIR) &= BIT4LO;
	
	// enable local capture interrupt
	HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_CAEIM;
	
	// enable Timer A in Wide Timer 0 interrupt in the NVIC (94--> EN2, bit 30)
	HWREG(NVIC_EN2) |= BIT30HI;
	
	// ensure interrupts are enabled globally
	__enable_irq();
	
	// enable timer and enable timer to stall when program stopped by the debugger
	// HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}

/****************************************************************************
 Function
    EnableStagingAreaISR

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
void EnableStagingAreaISR( void )
{
	// enable timer and enable timer to stall when program stopped by the debugger
	HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}

/****************************************************************************
 Function
    StagingAreaISR

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
void StagingAreaISR( void )
{
	// clear the source of the interrupt 
	HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;
	
	// grab captured value and calc period 
	CurrentEdge = HWREG(WTIMER0_BASE+TIMER_O_TAR);
	MeasuredStagingAreaPeriod = CurrentEdge - LastEdge;
	MeasuredStagingAreaPeriod = 1000*MeasuredStagingAreaPeriod/TicksPerMS; // Unit: us
	
	// Update the module level variable StagingAreaPeriod to be the average of the past ten catches
	// Update it every 10 interrupts
	if(counter >= 10){
		StagingAreaPeriod = StagingAreaPeriodAddition/10;

		StagingAreaPeriodAddition = 0;
		counter = 0;
  }
  
	StagingAreaPeriodAddition  += MeasuredStagingAreaPeriod;
	counter ++;
	
	// update LastCapture to prepare for the next edge
	LastEdge = CurrentEdge;
}

/****************************************************************************
 Function
    GetStagingAreaCode

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
uint16_t GetStagingAreaCode( void )
{
	if ( (StagingAreaPeriod < (StagingAreaPeriods[0] + StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod > (StagingAreaPeriods[15] - StagingAreaPeriod_Tolerance)) ){
		if ( (StagingAreaPeriod > (StagingAreaPeriods[0] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[0] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code1333us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[1] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[1] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code1277us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[2] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[2] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code1222us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[3] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[3] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code1166us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[4] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[4] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code1111us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[5] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[5] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code1055us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[6] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[6] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code1000us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[7] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[7] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code944us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[8] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[8] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code889us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[9] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[9] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code833us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[10] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[10] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code778us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[11] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[11] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code722us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[12] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[12] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code667us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[13] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[13] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code611us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[14] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[14] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code556us;
		}

		else if ( (StagingAreaPeriod > (StagingAreaPeriods[15] - StagingAreaPeriod_Tolerance)) && (StagingAreaPeriod < (StagingAreaPeriods[15] + StagingAreaPeriod_Tolerance)) ){
			StagingAreaCode = code500us;
		}
	}
	else{
		StagingAreaCode = codeInvalidStagingArea;
	}
	
	printf("\r\n code %x",StagingAreaCode);
	return StagingAreaCode;
}
