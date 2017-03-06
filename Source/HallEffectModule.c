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

// staging area frequency codes
#define code1333us (BIT0LO) //0000;
#define code1277us (BIT0HI) //0001;
#define code1222us (BIT1HI) //0010;
#define code1166us (BIT1HI|BIT0HI) //0011;
#define code1111us (BIT2HI) //0100;
#define code1055us (BIT2HI|BIT0HI) //0101;
#define code1000us (BIT2HI|BIT1HI) //0110;
#define code944us (BIT2HI|BIT1HI|BIT0HI) //0111;
#define code889us BIT3HI //1000;
#define code833us (BIT3HI|BIT0HI) //1001;
#define code778us (BIT3HI|BIT1HI) // 1010;
#define code722us (BIT3HI|BIT1HI|BIT0HI) //1011;
#define code667us (BIT3HI|BIT2HI) //1100;
#define code611us (BIT3HI|BIT2HI|BIT0HI) //1101;
#define code556us (BIT3HI|BIT2HI|BIT1HI) //1110;
#define code500us (BIT3HI|BIT2HI|BIT1HI|BIT0HI) //1111;
#define codeInvalidStagingArea 0xff

/*---------------------------- Module Variables ---------------------------*/
// For Staging Area Frequency Capture
static uint16_t StagingAreaCode;
static uint16_t StagingAreaPeriod_Tolerance = 10;
static uint32_t LastEdge;
static uint32_t CurrentEdge;
static uint32_t MeasuredStagingAreaPeriod;
static uint8_t counter = 0;
// static int StagingAreaPeriod = 0; // Can use this to make period calculation more robust
// static int StagingAreaPeriodAddition = 0; // Can use this to make period calculation more robust
static uint16_t PeriodBuffer[10];
static int SampleSize = 10;
static uint8_t CaptureIndex = 0;
static uint16_t StagingAreaPeriods[16] = {1333, 1277, 1222, 1166, 1111, 1055, 1000, 944, 889, 833, 778, 722, 667, 611, 556, 500};

/*---------------------------- Module Functions ---------------------------*/

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
void InitStagingAreaISR ( void )
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
	
	// SEE ME (might need to enable stall for debugger here instead of EnableStagingAreaISR)
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
	
	//Write this captured period into the array
	PeriodBuffer[counter] = MeasuredStagingAreaPeriod;
	
	//Move to the next element the next time
	counter++;
		
	// Update the module level variable StagingAreaPeriod to be the average of the past ten catches
	// Update it every 10 interrupts
	if(counter >= SampleSize){
		counter = 0;
  }
	
	// update LastCapture to prepare for the next edge
	LastEdge = CurrentEdge;
	
	// Update the module level variable StagingAreaPeriod to be the average of the past ten catches
	// Update it every 10 interrupts
//	if(counter >= SampleSize){
//		StagingAreaPeriod = StagingAreaPeriodAddition/SampleSize;

//		StagingAreaPeriodAddition = 0;
//		counter = 0;
//  }
	
	// SEE ME
	// Running average calculation. Can use this to make calculation more robust
	// StagingAreaPeriodAddition  = (90*StagingAreaPeriodAddition + 10*MeasuredStagingAreaPeriod)/100;
	// StagingAreaPeriod = StagingAreaPeriodAddition;

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
//uint16_t GetStagingAreaCode( void )
//{
//	uint16_t thePeriod = StagingAreaPeriod;
//	printf("\r\n stag per %i",thePeriod);
//	if( (thePeriod < (StagingAreaPeriods[0] + StagingAreaPeriod_Tolerance)) && (thePeriod > (StagingAreaPeriods[15] - StagingAreaPeriod_Tolerance)) ){
//		if ( (thePeriod > (StagingAreaPeriods[0] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[0] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code1333us;
//			printf("\r\n code1333 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[1] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[1] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code1277us;
//			printf("\r\n code1277 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[2] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[2] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code1222us;
//			printf("\r\n code1222 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[3] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[3] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code1166us;
//			printf("\r\n code1166 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[4] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[4] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code1111us;
//			printf("\r\n code1111 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[5] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[5] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code1055us;
//			printf("\r\n code1055 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[6] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[6] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code1000us;
//			printf("\r\n code1000 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[7] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[7] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code944us;
//			printf("\r\n code944 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[8] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[8] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code889us;
//			printf("\r\n code889 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[9] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[9] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code833us;
//			printf("\r\n code833 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[10] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[10] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code778us;
//			printf("\r\n code778 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[11] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[11] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code722us;
//			printf("\r\n code722 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[12] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[12] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code667us;
//			printf("\r\n code667 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[13] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[13] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code611us;
//			printf("\r\n code611 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[14] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[14] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code556us;
//			printf("\r\n code566 %x",StagingAreaCode);
//		}

//		else if ( (thePeriod > (StagingAreaPeriods[15] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[15] + StagingAreaPeriod_Tolerance)) ){
//			StagingAreaCode = code500us;
//			printf("\r\n code500 %x",StagingAreaCode);
//		}
//	}
//	else{
//		StagingAreaCode = codeInvalidStagingArea;
//		printf("\r\n code70 %x",StagingAreaCode);
//	}
//	
//	//printf("\r\n code %x",StagingAreaCode);
//	return StagingAreaCode;
//}

/****************************************************************************
 Function
    GetStagingAreaCodeSingle

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

uint8_t GetStagingAreaCodeSingle( uint16_t thePeriod )
{
		if( (thePeriod < (StagingAreaPeriods[0] + StagingAreaPeriod_Tolerance)) && (thePeriod > (StagingAreaPeriods[15] - StagingAreaPeriod_Tolerance)) ){
			if ( (thePeriod > (StagingAreaPeriods[0] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[0] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code1333us;
				//printf("\r\n code1333 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[1] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[1] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code1277us;
				//printf("\r\n code1277 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[2] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[2] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code1222us;
				//printf("\r\n code1222 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[3] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[3] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code1166us;
				//printf("\r\n code1166 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[4] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[4] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code1111us;
				//printf("\r\n code1111 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[5] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[5] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code1055us;
				//printf("\r\n code1055 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[6] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[6] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code1000us;
				//printf("\r\n code1000 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[7] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[7] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code944us;
				//printf("\r\n code944 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[8] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[8] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code889us;
				//printf("\r\n code889 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[9] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[9] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code833us;
				//printf("\r\n code833 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[10] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[10] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code778us;
				//printf("\r\n code778 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[11] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[11] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code722us;
				//printf("\r\n code722 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[12] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[12] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code667us;
				//printf("\r\n code667 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[13] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[13] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code611us;
				//printf("\r\n code611 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[14] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[14] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code556us;
				//printf("\r\n code566 %x",StagingAreaCode);
			}

			else if ( (thePeriod > (StagingAreaPeriods[15] - StagingAreaPeriod_Tolerance)) && (thePeriod < (StagingAreaPeriods[15] + StagingAreaPeriod_Tolerance)) ){
				StagingAreaCode = code500us;
				//printf("\r\n code500 %x",StagingAreaCode);
			}
		}
		else{
			StagingAreaCode = codeInvalidStagingArea;
			//printf("\r\n code70 %x",StagingAreaCode);
		}	

	//printf("\r\n code %x",StagingAreaCode);
	return StagingAreaCode;
}

/****************************************************************************
 Function
    GetStagingAreaCodeArray

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

uint8_t GetStagingAreaCodeArray(void){
	uint8_t i = 0;
	uint8_t returnCode = GetStagingAreaCodeSingle(PeriodBuffer[0]);
	printf("\r\n%u\r\n",returnCode);
	
	for(i = 1; i < SampleSize; i++){
		if(GetStagingAreaCodeSingle(PeriodBuffer[i]) != returnCode){
			return codeInvalidStagingArea;
		}
	}
	
	return returnCode;
}
