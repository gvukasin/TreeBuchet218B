/****************************************************************************
Tape Module
	Define the Initialization and ISR for the tape sensing interrupt
	Whenever tape is detected, post event TapeSensed
 
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


/*----------------------------- Module Defines ----------------------------*/
#define ONE_SEC 976 //assume a 1.000mS/tick timing
#define ALL_BITS (0xff<<2)

#define STOP 0x00
/*---------------------------- Module Variables ---------------------------*/
static uint32_t LastCapture;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitTapeInterrupt
		 
 Description
     Initialize harware for wide timer 0 and enable interrupt on PC4
****************************************************************************/
void InitTapeInterrupt (void){
	
	//Enable the clock to Wider Timer 0
	HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
	
	//Enable the clock to port C
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
	
	//Make sure Timer A is disabled before configuring
	HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
	
	//Set it up to 32 bit wide, individual mode
	HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
	
	//Initialize the Interval Load register to 0xFFFFFFFF
	HWREG(WTIMER0_BASE+TIMER_O_TAILR) = 0xffffffff;
	
	//Set up Timer A in capture mode, for edge time, up-counting
	HWREG(WTIMER0_BASE+TIMER_O_TAMR) = (HWREG(WTIMER0_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS)|(TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
	
	//Set event to rising edge
	HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;
	
	//Set up the port to do the capture
	//Set the alternate function for PC4
	HWREG(GPIO_PORTC_BASE+GPIO_O_AFSEL) |= BIT4HI;
	//Map PC4's alternate function to WT0CCP0
	HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) = (HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) & 0xfff0ffff) + (7<<16);
	//Enable PC4 to be digital input
	HWREG(GPIO_PORTC_BASE+GPIO_O_DEN) |= BIT4HI;
	HWREG(GPIO_PORTC_BASE+GPIO_O_DIR) &= BIT4LO;
	
	//Enable a local capture interrupt
	HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_CAEIM;
	
	//Enable Timer A in Wide Timer 0 interrupt in the NVIC
	HWREG(NVIC_EN2) |= BIT30HI;
	
	//Enable interrupts globally
	__enable_irq();

		printf("\r\nGot through tape interrupt init\r\n");
}

/****************************************************************************
 Function
     EnableTapeInterrupt

 Description
     Define the interrupt response routine
****************************************************************************/
void EnableTapeInterrupt(void)
{
 //Kick the timer off and enable it to stall while stopped by the debugger
	HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}

/****************************************************************************
 Function
     TapeInterruptResponse

 Description
     Define the interrupt response routine
****************************************************************************/
void TapeInterruptResponse(void){
	
	uint32_t ThisCapture;
	
	//Start by clearing out the source of the interrupt
	HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;
	
	//Get the captured value 
	ThisCapture = HWREG(WTIMER0_BASE+TIMER_O_TAR);
	
	//Post event to ActionService
	ES_Event ThisEvent;
	ThisEvent.EventType = TapeSensed;
	ThisEvent.EventParam = END_RUN;
	PostActionService(ThisEvent);
	
	LastCapture = ThisCapture;
}

uint32_t GetTapeSensedTime(void){
	return LastCapture;
}

