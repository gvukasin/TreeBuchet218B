/****************************************************************************
 Module
   ActionService.c

 Description
	PWM generation 	 
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"

#include "ActionService.h"
#include "PWMmodule.h"
#include "SPIService.h"
#include "MotorActionsModule.h"
#include "IRBeaconModule.h"

#include <stdio.h>
#include <termio.h>

// the common headers for C99 types 
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "BITDEFS.H"
#include "inc/hw_timer.h"

/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define TEST_MODE

#define STOP 0x00
#define CW_90 0x02 
#define CW_45 0x03 
#define CCW_90 0x04 
#define CCW_45 0x05 
#define FORWARD_HALF_SPEED 0x08 
#define FORWARD_FULL_SPEED 0x09 
#define REVERSE_HALF_SPEED 0x10 
#define REVERSE_FULL_SPEED 0x11 
#define ALIGN_BEACON 0x20 
#define DRIVE2TAPE 0x40 
#define READY4NEXTCOMMAND 0xff

#define FORWARD 1
#define BACKWARD 0
#define CW 1
#define CCW 0
#define LEFT 1
#define RIGHT 0

#define DUTY_HALF_SPEED 75 //might need to be changed
#define DUTY_FULL_SPEED 100

// using 40 MHz clock
#define TicksPerMS 40000

#define Rotate90Timeout 1050
#define Rotate45Timeout 550
#define AlignWithBeaconTimeout 5000
#define Post2SPITimeout 100

#define QueryBits 0xAA

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service*/
static void InitOneShotISR(void);
static void SetTimeoutAndStartOneShot( uint32_t);
static void Look4Beacon(uint32_t);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static bool runActionSwitchFlag = 0;
static uint32_t OneShotTimeoutMS;
static ES_Event LastEvent;
static ES_Event SPIEvent;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitializeActionService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     Team 16 
****************************************************************************/
 bool InitializeActionService (uint8_t Priority)
 { 

 	// ES_Event ThisEvent;
 	MyPriority = Priority;
	  
	// Initialize PWM functionality
	InitializePWM();
	 
	// Initialize Interrupts
	InitInputCaptureForIRDetection();
	InitOneShotISR();
	
	// Initialize last event parameter to 0
	LastEvent.EventParam = 0xff;

	return true;
 }

/****************************************************************************
 Function
     PostActionService

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Team 16 
****************************************************************************/
bool PostActionService(ES_Event ThisEvent)
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunActionService

 Parameters
   ES_Event : the event to process
   ThisEvent will be the time that the event occurred.

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Controls the necessary parameters to get the desired motion 
   
 Author
     Team 16 
****************************************************************************/
ES_Event RunActionService(ES_Event ThisEvent)
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	
	if (ThisEvent.EventParam == READY4NEXTCOMMAND)
	{
		printf("FF");
		runActionSwitchFlag = 1;
	}
	
	if ((runActionSwitchFlag == 1) && (ThisEvent.EventParam != READY4NEXTCOMMAND))
	{
		printf("\r\n %x\r\n", ThisEvent.EventParam);
		runActionSwitchFlag = 0;
		switch(ThisEvent.EventParam)
		{
			//Case 1 
			case STOP:
				//printf("\r\n STOP Received\n");
				stop();
				break;
			
			//Case 2
			case CW_90:
			//	printf("\r\n CW_90 Received\n");
				SetTimeoutAndStartOneShot(Rotate90Timeout);
				start2rotate(CW);
				break;
			
			//Case 3
			case CW_45:
				//printf("\r\n CW_45 Received\n");
				SetTimeoutAndStartOneShot(Rotate45Timeout);
				start2rotate(CW);			
				break;
			
			//Case 4
			case CCW_90:
				//printf("\r\n CCW_90 Received\n");
				SetTimeoutAndStartOneShot(Rotate90Timeout);
				start2rotate(CCW);
				break;
			
			//Case 5
			case CCW_45:
				//printf("\r\n CCW_45 Received\n");
				SetTimeoutAndStartOneShot(Rotate45Timeout);
				start2rotate(CCW);
				break;
			
			//Case 6
			case FORWARD_HALF_SPEED:
				//printf("\r\n FORWARD_HALF_SPEED Received\n");
				drive(DUTY_HALF_SPEED, FORWARD);
				break;
			
			//Case 7
			case FORWARD_FULL_SPEED:
				//printf("\r\n FORWARD_FULL_SPEED Received\n");
				drive(DUTY_FULL_SPEED, FORWARD);
				break;
			
			//Case 8
			case REVERSE_HALF_SPEED:
			//	printf("\r\n REVERSE_HALF_SPEED Received\n");
				drive(DUTY_HALF_SPEED, BACKWARD);
				break;
			
			//Case 9
			case REVERSE_FULL_SPEED:
				//printf("\r\n REVERSE_FULL_SPEED Received\n");
				drive(DUTY_FULL_SPEED, BACKWARD);
				break;
			
			//Case 10
			case ALIGN_BEACON:
				rotate2beacon();
				EnableIRInterrupt();
				break;
			
			//Case 11
			case DRIVE2TAPE:
				//printf("\r\n DRIVE2TAPE Received\n");
				drive(DUTY_FULL_SPEED, FORWARD);
				break;
			
			//Case 12
			case END_RUN:
				//printf("\r\n END_RUN Received\n");
				// stop motors and stop posting events
				stop();
				break;
		}
	}
	
	// post to SPI service after each execution of Action Service to get the next command
	// exception: after END_RUN is executed
	return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
     InitOneShotISR

 Parameters
     void

 Returns
     void

 Description
			Initialization for interrupt response for one shot timer to set
			speed to 0
			
 Notes

 Author
     Team 16 
****************************************************************************/
static void InitOneShotISR(){
	
	// start by enabling the clock to the timer (Wide Timer 0)
	HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;

	// kill a few cycles to let the clock get going
	while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R0) != SYSCTL_PRWTIMER_R0){}

	// make sure that timer (Timer A) is disabled before configuring
	HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //TAEN = Bit1

	// set it up in 32bit wide (individual, not concatenated) mode
	// the constant name derives from the 16/32 bit timer, but this is a 32/64
	// bit timer so we are setting the 32bit mode
	HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT; //bits 0-2 = 0x04

	// set up timer A in 1-shot mode so that it disables timer on timeouts
	// first mask off the TAMR field (bits 0:1) then set the value for
	// 1-shot mode = 0x01
	HWREG(WTIMER0_BASE+TIMER_O_TBMR) = (HWREG(WTIMER0_BASE+TIMER_O_TBMR)& ~TIMER_TBMR_TBMR_M)| TIMER_TBMR_TBMR_1_SHOT;
	
	// set timeout
	OneShotTimeoutMS = 1000; //arbitrary initialization value
	HWREG(WTIMER0_BASE+TIMER_O_TBILR) = TicksPerMS*OneShotTimeoutMS;
	
	// enable a local timeout interrupt. TBTOIM = bit 1
	HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_TBTOIM; // bit1

	// enable the Timer B in Wide Timer 0 interrupt in the NVIC
	// it is interrupt number 95 so appears in EN2 at bit 31
	HWREG(NVIC_EN2) |= BIT31HI;
	
	// set priority of this timer 
	HWREG(NVIC_PRI23) |= NVIC_PRI23_INTD_M;

	// make sure interrupts are enabled globally
	__enable_irq();
	
	printf("\r\nGot through one shot interrupt init\r\n");
}

/****************************************************************************
 Function
     SetTimeoutAndStartOneShot

 Parameters
     void

 Returns
     void

 Description
			Restart the oneshot timer

			
 Notes

 Author
     Team 16 
****************************************************************************/ 
static void SetTimeoutAndStartOneShot( uint32_t OneShotTimeoutMS )
{
	// set timeout
	HWREG(WTIMER0_BASE+TIMER_O_TBILR) = TicksPerMS*OneShotTimeoutMS;
	
	// now kick the timer off by enabling it and enabling the timer to stall while stopped by the debugger
	HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
}

/****************************************************************************
 Function
     OneShotISR

 Parameters
     void

 Returns
     void

 Description
			Interrupt response for one shot timer --> 
			stops whatever motion is going on
			
 Notes

 Author
     Team 16 
****************************************************************************/ 
void OneShotISR(void){
	
	// clear interrupt
	HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_TBTOCINT; 
	
	// stop current motion
	stop();
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
