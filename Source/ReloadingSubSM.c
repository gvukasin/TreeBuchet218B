/****************************************************************************
 Module
   ReloadingSubSM.c

 Revision
   2.0.1

 Description
   Loading a new ball

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
#include "PWMModule.h"

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
#define TimeWaiting4Ball 3000 //3sec must wait between ball requests
#define STOP_PWM 0
#define START_PWM 1
#define PulseDuration 600 // time needed to get 15 complete pulses of the PWM with f=25Hz and d.c. = 75%

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event DuringRequestingBall( ES_Event Event);
static ES_Event DuringWaiting4Ball( ES_Event Event);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static ReloadingState_t CurrentState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunReloadingSM

 Parameters
   ES_Event: the event to process

 Returns
   ES_Event: an event to return

 Description
   Uses nested switch/case to implement a statemachine to reload the robot from the supply depots
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
         //If event type is not ES_NO_EVENT or ES_TIMEOUT, process the event
         if (( CurrentEvent.EventType != ES_NO_EVENT ) && ( CurrentEvent.EventType == ES_TIMEOUT ) && (CurrentEvent.EventParam == SendingIRPulses_TIMER)) //If an event is active and it's the correct one
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
       break;
      
				 // CASE 2/2				 
			 case WAITING4BALL:
			 // During function
       CurrentEvent = DuringWaiting4Ball(CurrentEvent);
			 // Process events	
			 if(CurrentEvent.EventType == RELOAD_BALLS)
			 {
					 NextState = REQUESTING_BALL;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;						 
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
     // Return the return event
     return(ReturnEvent);
}
/****************************************************************************
 Function
     StartReloadingSM

 Parameters
     ES_Event CurrentEvent

 Returns
     None

 Description
     Initializes this state machine
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
     QueryReloadingSM

 Parameters
     None

 Returns
     The current state of this state machine

 Description
     Returns the current state of this state machine
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/
ReloadingState_t QueryReloadingSM ( void )
{
  // Return the current state of this state machine
   return(CurrentState);
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
     DuringRequestingBall

 Parameters
     ES_Event Event to respond to 

 Returns
     ES_Event ReturnEvent to return to the rest of the state machine infrastructure

 Description
     During function for RequestingBall state, which sends IR pulses to request a ball
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/
static ES_Event DuringRequestingBall( ES_Event Event) 
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or consumption

    // IF event is ES_ENTRY or ES_ENTRY_HISTORY
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // do nothing
    }
    // Else If event is ES_EXIT
    else if ( Event.EventType == ES_EXIT )
    {
      // turn off communication LEDS
        TurnOnOffBlueLEDs(LEDS_OFF, GetTeamColor());
    }
		
		// Else do the 'during' function for this state
		else 
    {
				// Turn off communication LEDs
				TurnOnOffBlueLEDs(LEDS_ON, GetTeamColor());	
        // Kick off SendingIRPulses_TIMER		
				ES_Timer_InitTimer(SendingIRPulses_TIMER, PulseDuration);
        // Send 15 pulses (10ms ON + 30ms OFF)
				EmitIR(START_PWM);	
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/****************************************************************************
 Function
     DuringWaiting4Ball

 Parameters
     ES_Event Event to respond to 

 Returns
     ES_Event ReturnEvent to return to the rest of the state machine infrastructure

 Description
     During function for Waiting4Ball state, which waits 3 seconds in between requests
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/
static ES_Event DuringWaiting4Ball( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // If Event is ES_ENTRY or ES_ENTRY_HISTORY 
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
      // Stop sending IR pulses
			EmitIR(STOP_PWM);	

			// Start 3 second timer
			ES_Timer_InitTimer(Waitin4Ball_TIMER, TimeWaiting4Ball);
    }
    // Else If Event is ES_EXIT
    else if ( Event.EventType == ES_EXIT )
    {
      // do nothing
    }
		
		// Else do the 'during' function for this state
		else 
    {
				// we are just waiting to get a timeout OR repeating if we want more balls
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

