/****************************************************************************
 Module
   RobotTopSM.c

 Revision
   2.0.1

 Description
   This is the robot top state machine

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/20/17 14:30 jec      updated to remove sample of consuming an event. We 
                         always want to return ES_NO_EVENT at the top level 
                         unless there is a non-recoverable error at the 
                         framework level
 02/03/16 15:27 jec      updated comments to reflect small changes made in '14 & '15
                         converted unsigned char to bool where appropriate
                         spelling changes on true (was True) to match standard
                         removed local var used for debugger visibility in 'C32
                         removed Microwave specific code and replaced with generic
 02/08/12 01:39 jec      converted from MW_MasterMachine.c
 02/06/12 22:02 jec      converted to Gen 2 Events and Services Framework
 02/13/10 11:54 jec      converted During functions to return Event_t
                         so that they match the template
 02/21/07 17:04 jec      converted to pass Event_t to Start...()
 02/20/07 21:37 jec      converted to use enumerated type for events
 02/21/05 15:03 jec      Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the 
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "RobotTopSM.h"
#include "ShootingSubSM.h"
#include "SPIService.h"
#include "HallEffectModule.h"
#include "LEDModule.h"
#include "DrivingModule.h"
#include "ReloadingSubSM.h"

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
#define RED_BUTTON GPIO_PIN_3 
#define GREEN_BUTTON GPIO_PIN_4
#define RED_BUTTON_PRESSED BIT3HI
#define GREEN_BUTTON_PRESSED BIT4HI

#define LEDS_ON 1
#define LEDS_OFF 0

//Magnetic frequency codes
#define code1333us 0000
#define code1277us 0001
#define code1222us 0010
#define code1166us 0011
#define code1111us 0100
#define code1055us 0101
#define code1000us 0110
#define code944us 0111
#define code889us 1000
#define code833us 1001
#define code778us 1010
#define code722us 1011
#define code667us 1100
#define code611us 1101
#define code556us 1110
#define code500us 1111

/*---------------------------- Module Functions ---------------------------*/
static ES_Event DuringWaiting2Start( ES_Event Event);
static ES_Event DuringDriving2Staging( ES_Event Event);
static ES_Event DuringCheckIn( ES_Event Event);
static ES_Event DuringShooting( ES_Event Event);
static ES_Event DuringShooting( ES_Event Event);
static ES_Event DuringDriving2Reload( ES_Event Event);
static ES_Event DuringReloading( ES_Event Event);
static ES_Event DuringEndingStrategy( ES_Event Event);
static ES_Event DuringStop( ES_Event Event);

static void InitializeTeamButtonsHardware(void);


/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, though if the top level state machine
// is just a single state container for orthogonal regions, you could get
// away without it
static RobotState_t CurrentState;
static uint8_t MyPriority;
uint32_t PositionDifference;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMasterSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     boolean, False if error in initialization, True otherwise

 Description
     Saves away the priority,  and starts
     the top level state machine
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:06
****************************************************************************/
bool InitRobotTopSM ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;  // save our priority

  ThisEvent.EventType = ES_ENTRY;
	
	// Initialize hardware
	//InitializeTeamButtonsHardware();   //UNCOMMENT AFTER CHECK OFF
  
	// Start the Master State machine
  StartRobotTopSM( ThisEvent );

  return true;
}

/****************************************************************************
 Function
     PostMasterSM

 Parameters
     ES_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the post operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostRobotTopSM( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMasterSM

 Parameters
   ES_Event: the event to process

 Returns
   ES_Event: an event to return

 Description
   the run function for the top level state machine 
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 02/06/12, 22:09
****************************************************************************/
ES_Event RunRobotTopSM( ES_Event CurrentEvent )
{
   bool MakeTransition = false;/* are we making a state transition? */
   RobotState_t NextState = CurrentState;
   ES_Event EntryEventKind = { ES_ENTRY, 0 };// default to normal entry to new state
   ES_Event ReturnEvent = { ES_NO_EVENT, 0 }; // assume no error

   switch ( CurrentState )
   {
				// CASE 1/8
				case WAITING2START:               
				// Execute During function for WAITING2START
        CurrentEvent = DuringWaiting2Start(CurrentEvent);
				//process any events
				if (CurrentEvent.EventType == START) //an event is active and it is START
				{
					 NextState = DRIVING2STAGING;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}
        break;
			
				// CASE 2/8				 
			 case DRIVING2STAGING:
			 // During function
       CurrentEvent = DuringDriving2Staging(CurrentEvent);
			 // Process events			 
			 if (CurrentEvent.EventType == STATION_REACHED)
				{
					 NextState = CHECKING_IN;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}
				 break;
				
			 // CASE 3/8				 
			 case CHECKING_IN:
			 // During function
       CurrentEvent = DuringCheckIn(CurrentEvent);
			 // Process events			 
			 if ( CurrentEvent.EventType != ES_NO_EVENT ) //If an event is active
         {
            switch (CurrentEvent.EventType)
            {
               case CHECK_IN_SUCCESS : 
                  NextState = SHOOTING;
                  MakeTransition = true; 
                  break;
               case CHECK_IN_FAIL :
								  NextState = CHECKING_IN; // Self transition
                  MakeTransition = true; 
                  break;
							 case FINISH_STRONG :
								 NextState = ENDING_STRATEGY;
								 MakeTransition = true;
								 break;
							 default:
								 printf("\r\nERROR: Robot is in CHECKING_IN and EVENT NOT VALID\n");
            }
					}
				 break;

			 // CASE 4/8				 
			 case SHOOTING:
			 // During function
       CurrentEvent = DuringShooting(CurrentEvent);
			 // Process events			 
			 if ( CurrentEvent.EventType != ES_NO_EVENT ) //If an event is active
         {
            switch (CurrentEvent.EventType)
            {
               case SCORED : 
                  NextState = DRIVING2STAGING;
                  MakeTransition = true; 
                  break;
               case MISSED_SHOT :
								  NextState = SHOOTING; // Self transition
                  MakeTransition = true; 
                  break;
							 case NO_BALLS :
								  NextState = DRIVING2RELOAD; 
                  MakeTransition = true; 
                  break;
							 case FINISH_STRONG :
								 NextState = ENDING_STRATEGY;
								 MakeTransition = true;
								 break;
							 default:
								 printf("\r\nERROR: Robot is in SHOOTING and the event received is NOT VALID\n");
            }
					}
				 break;

			 // CASE 5/8				 
			 case DRIVING2RELOAD:
			 // During function
       CurrentEvent = DuringDriving2Reload(CurrentEvent);
			 // Process events			 
			 if (CurrentEvent.EventType == RELOAD_BALLS)
				{
					 NextState = RELOADING;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}				 
				 break;
				
			 // CASE 6/8				 
			 case RELOADING:
			 // During function
       CurrentEvent = DuringReloading(CurrentEvent);
			 // Process events			 
			 if (CurrentEvent.EventType == BALLS_AVAILABLE)
				{
					 NextState = DRIVING2STAGING;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}							 
				 break;
				
			 // CASE 7/8				 			 
			 case ENDING_STRATEGY:
			 // During function
       CurrentEvent = DuringEndingStrategy(CurrentEvent);
			 // Process events			 
			 if (CurrentEvent.EventType == GAME_OVER)
				{
					 NextState = DRIVING2STAGING;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}							 
				 break;	
			
			 // CASE 8/8				 			 
			 case STOP:
			 // During function
       CurrentEvent = DuringStop(CurrentEvent);	
				 break;				 
		}
 	 
    //   If we are making a state transition
    if (MakeTransition == true)
    {
       //   Execute exit function for current state
       CurrentEvent.EventType = ES_EXIT;
       RunRobotTopSM(CurrentEvent);

       CurrentState = NextState; //Modify state variable

       // Execute entry function for new state
       // this defaults to ES_ENTRY
       RunRobotTopSM(EntryEventKind);
     }
		
   // in the absence of an error the top level state machine should
   // always return ES_NO_EVENT, which we initialized at the top of func
   return(ReturnEvent);
}
/****************************************************************************
 Function
     StartMasterSM

 Parameters
     ES_Event CurrentEvent

 Returns
     nothing

 Description
     Does any required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:15
****************************************************************************/
void StartRobotTopSM ( ES_Event CurrentEvent )
{
  // if there is more than 1 state to the top level machine you will need 
  // to initialize the state variable
  CurrentState = WAITING2START;
  // now we need to let the Run function init the lower level state machines
  // use LocalEvent to keep the compiler from complaining about unused var
  RunRobotTopSM(CurrentEvent);
  return;
}


/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
During Functions:

ES_ENTRY & ES_EXIT are processed here and allow 
the lower level state machines to re-map or consume 
the event
****************************************************************************/

static ES_Event DuringWaiting2Start( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // entry actions required for this state machine
       
    }
    else if ( Event.EventType == ES_EXIT )
    { 
    }
		
		// do the 'during' function for this state
		else 
    {
        // do any activity that is repeated as long as we are in this state
				// SHOULD WE USE THE BUTTON SERVICE FROM LAB 5????????????
				ES_Event PostEvent;
				PostEvent.EventType = TEAM_COLOR;
				
				if (RED_BUTTON_PRESSED)
					PostEvent.EventParam = 0;
				else if (GREEN_BUTTON_PRESSED)
					PostEvent.EventParam = 1;
				else
					PostEvent.EventParam = 0; //Defaults to 0

					//printf("\r\nYou need to press a button for team selection!\r\n");
			
				PostSPIService(PostEvent);
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringDriving2Staging( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
      // Initialize magnetic sensing hardware  
			InitMagneticSensor();
    }
    else if ( Event.EventType == ES_EXIT )
    {
    }
		
		// do the 'during' function for this state
		else 
    {
			// Wire following
			CheckWirePosition();
						
			
			//If a station has been reached post an event   MAKE THE IF!!!!!!!!!!!!!!
			ES_Event PostEvent;
			PostEvent.EventType = STATION_REACHED;
			PostRobotTopSM(PostEvent); // We want to move to the next state
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringCheckIn( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption
		ES_Event PostEvent;
	
    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // Check Ball count
			
				// Check time
    }
    else if ( Event.EventType == ES_EXIT)
    {
	
    }
		
		// do the 'during' function for this state
		else 
    {
        // run any lower level state machine
        // ReturnEvent = RunLowerLevelSM(Event);
      
        // repeat for any concurrent lower level machines
      
        // do any activity that is repeated as long as we are in this state
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringShooting( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
        //Yellow LEDs ON to signal shooting is going to start
				TurnOnOffYellowLEDs(LEDS_ON);
			
        // start any lower level machines that run in this state
        StartShootingSM(Event);  
	
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // on exit, give the lower levels a chance to clean up first
        RunShootingSM(Event);   
				
			  // Turn OFF LEDs
				TurnOnOffYellowLEDs(LEDS_OFF);
    }
		
		// do the 'during' function for this state
		else 
    {
        // run any lower level state machine
        ReturnEvent = RunShootingSM(Event);  // I THINK THIS WILL WORK??????????????????
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringDriving2Reload( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
    }
    else if ( Event.EventType == ES_EXIT )
    {
    }
		
		// do the 'during' function for this state
		else 
    {
        // do any activity that is repeated as long as we are in this state
				Drive2Reload();
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringReloading( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {        
        // after that start any lower level machines that run in this state
        StartReloadingSM( Event );
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // on exit, give the lower levels a chance to clean up first
        RunReloadingSM(Event); 
    }
		
		// do the 'during' function for this state
		else 
    {
        // run any lower level state machine
        ReturnEvent = RunReloadingSM(Event);
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringEndingStrategy( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // implement any entry actions required for this state machine
        
        // after that start any lower level machines that run in this state
        //StartLowerLevelSM( Event );
        // repeat the StartxxxSM() functions for concurrent state machines
        // on the lower level
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // on exit, give the lower levels a chance to clean up first
        //RunLowerLevelSM(Event);
        // repeat for any concurrently running state machines
        // now do any local exit functionality
      
    }
		else // do the 'during' function for this state
    {
        // run any lower level state machine
        // ReturnEvent = RunLowerLevelSM(Event);
      
        // repeat for any concurrent lower level machines
      
        // do any activity that is repeated as long as we are in this state
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringStop( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // implement any entry actions required for this state machine
        
        // after that start any lower level machines that run in this state
        //StartLowerLevelSM( Event );
        // repeat the StartxxxSM() functions for concurrent state machines
        // on the lower level
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // on exit, give the lower levels a chance to clean up first
        //RunLowerLevelSM(Event);
        // repeat for any concurrently running state machines
        // now do any local exit functionality
      
    }
		else // do the 'during' function for this state
    {
        // run any lower level state machine
        // ReturnEvent = RunLowerLevelSM(Event);
      
        // repeat for any concurrent lower level machines
      
        // do any activity that is repeated as long as we are in this state
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/****************************************************************************
Hardware Functions:
****************************************************************************/
static void InitializeTeamButtonsHardware(void)
{
	// Initialize port F to monitor the buttons
 	HWREG(SYSCTL_RCGCGPIO) |= BIT5HI;
 	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R5) != SYSCTL_PRGPIO_R5);
	HWREG(GPIO_PORTF_BASE+GPIO_O_PUR) |= (RED_BUTTON|GREEN_BUTTON);	//NEED THIS????????????????????????????????
	
	// Set bits 3 and 4 in port F as input:
 	HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= (RED_BUTTON|GREEN_BUTTON);	
 	HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) &= (~(RED_BUTTON)| ~(GREEN_BUTTON));	
}


/*********************************************************  THE END *************************************************************/

				
