/****************************************************************************
 Module
   ReloadingSubSM.c

 Revision
   2.0.1

 Description
   This is a template file for implementing state machines.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/20/17 10:14 jec      correction to Run function to correctly assign 
                         ReturnEvent in the situation where a lower level
                         machine consumed an event.
 02/03/16 12:38 jec      updated comments to reflect changes made in '14 & '15
                         converted unsigned char to bool where appropriate
                         spelling changes on true (was True) to match standard
                         removed local var used for debugger visibility in 'C32
                         commented out references to Start & RunLowerLevelSM so
                         that this can compile. 
 02/07/13 21:00 jec      corrections to return variable (should have been
                         ReturnEvent, not CurrentEvent) and several EV_xxx
                         event names that were left over from the old version
 02/08/12 09:56 jec      revisions for the Events and Services Framework Gen2
 02/13/10 14:29 jec      revised Start and run to add new kind of entry function
                         to make implementing history entry cleaner
 02/13/10 12:29 jec      added NewEvent local variable to During function and
                         comments about using either it or Event as the return
 02/11/10 15:54 jec      more revised comments, removing last comment in during
                         function that belongs in the run function
 02/09/10 17:21 jec      updated comments about internal transitions on During funtion
 02/18/09 10:14 jec      removed redundant call to RunLowerlevelSM in EV_Entry
                         processing in During function
 02/20/07 21:37 jec      converted to use enumerated type for events & states
 02/13/05 19:38 jec      added support for self-transitions, reworked
                         to eliminate repeated transition code
 02/11/05 16:54 jec      converted to implment hierarchy explicitly
 02/25/03 10:32 jec      converted to take a passed event parameter
 02/18/99 10:19 jec      built template from MasterMachine.c
 02/14/99 10:34 jec      Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// Basic includes for a program using the Events and Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"

/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ReloadingSubSM.h"
#include "SPIService.h"
#include "RobotTopSM.h"
#include "LEDModule.h"
#include "ShootingSubSM.h"

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
// define constants for the states for this machine
// and any other local defines
#define LEDS_ON 1
#define LEDS_OFF 0
#define IR_LED BIT2HI
#define IR_LED_OFF BIT2LO
#define ALL_BITS (0xff<<2)

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event DuringRequestingBall( ES_Event Event);
static ES_Event DuringWaiting4Ball( ES_Event Event);
static void EmmitIR();

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static ReloadingState_t CurrentState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunTemplateSM

 Parameters
   ES_Event: the event to process

 Returns
   ES_Event: an event to return

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 2/11/05, 10:45AM
****************************************************************************/
ES_Event RunReloadingSM( ES_Event CurrentEvent )
{
   bool MakeTransition = false;/* are we making a state transition? */
   ReloadingState_t NextState = CurrentState;
   ES_Event EntryEventKind = { ES_ENTRY, 0 };// default to normal entry to new state
   ES_Event ReturnEvent = CurrentEvent; // assume we are not consuming event

   switch ( CurrentState )
   {
			 // CASE 1/2
       case REQUESTING_BALL :       
         // Execute During function 
         CurrentEvent = DuringRequestingBall(CurrentEvent);
         //process any events
         if (( CurrentEvent.EventType != ES_NO_EVENT ) && ( CurrentEvent.EventType == WAIT4BALL_DELIVERY )) //If an event is active and it's the correct one
         {       
                  NextState = WAITING4BALL;//Decide what the next state will be
                  MakeTransition = true; //mark that we are taking a transition
                  // if transitioning to a state with history change kind of entry
                  //EntryEventKind.EventType = ES_ENTRY_HISTORY;
                  ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
                  break;
          }
				 else if ( CurrentEvent.EventType == ES_NO_EVENT )// Current Event is now ES_NO_EVENT. Correction 2/20/17 
         {     																						//Probably means that CurrentEvent was consumed by lower level
            ReturnEvent = CurrentEvent; // in that case update ReturnEvent too
         }
				 else
				 {
					 printf("\r\nERROR: robot in reloading>requestingBall with NOT VALID EVENT\r");
				 }
       break;
      
				 // CASE 2/2				 
			 case WAITING4BALL:
			 // During function
       CurrentEvent = DuringWaiting4Ball(CurrentEvent);
			 // Process events	
			 switch (CurrentEvent.EventType)
			 {
				 case RELOAD_BALLS:
					 NextState = REQUESTING_BALL;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;						 
					 break;
				 case BALLS_AVAILABLE:
					 //ReturnEvent.EventType = ES_NO_EVENT;	 DONT KNOW WHAT TO DO
					 break;
				 default:
					 printf("\r\nERROR: robot in reloading>requestingBall with NOT VALID EVENT\r");					 
			 }
    }
    //   If we are making a state transition
    if (MakeTransition == true)
    {
       //   Execute exit function for current state
       CurrentEvent.EventType = ES_EXIT;
       RunShootingSM(CurrentEvent);

       CurrentState = NextState; //Modify state variable

       //   Execute entry function for new state
       // this defaults to ES_ENTRY
       RunShootingSM(EntryEventKind);
     }
     return(ReturnEvent);
}
/****************************************************************************
 Function
     StartTemplateSM

 Parameters
     None

 Returns
     None

 Description
     Does any required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 2/18/99, 10:38AM
****************************************************************************/
void StartReloadingSM ( ES_Event CurrentEvent )
{
   // to implement entry to a history state or directly to a substate
   // you can modify the initialization of the CurrentState variable
   // otherwise just start in the entry state every time the state machine
   // is started
   if ( ES_ENTRY_HISTORY != CurrentEvent.EventType )
   {
        CurrentState = REQUESTING_BALL;
   }
   // call the entry function (if any) for the ENTRY_STATE
   RunReloadingSM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/
ReloadingState_t QueryReloadingSM ( void )
{
   return(CurrentState);
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event DuringRequestingBall( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or consumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // implement any entry actions required for this state machine
        TurnOnOffBlueLEDs(LEDS_ON, GetTeamColor());
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // do any local exit functionality
        TurnOnOffBlueLEDs(LEDS_OFF, GetTeamColor());
    }
		
		// do the 'during' function for this state
		else 
    {
        // Send 10 pulses (10ms ON + 30ms OFF) 
				
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringWaiting4Ball( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // Start 3 second timer
    }
    else if ( Event.EventType == ES_EXIT )
    {
    }
		
		// do the 'during' function for this state
		else 
    {
				// we are just waiting to get a timeout
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/********************************************************************************
********************************************************************************/
static void EmmitIR()
{
	//Initialize hardware
	HWREG(SYSCTL_RCGCGPIO)|= SYSCTL_RCGCGPIO_R1; //port B
 	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1);

	// Set bit 2 in port B as digital output:
 	HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= IR_LED;	
 	HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= IR_LED;
	
	//IR pulses	
	HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= IR_LED;
	HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= IR_LED_OFF;
}
