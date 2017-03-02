 /****************************************************************************
 Module
   ShootingSubSM.c

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
#include "ShootingSubSM.h"
#include "SPIService.h"
#include "RobotTopSM.h"
#include "MotorActionsModule.h"
#include "IRBeaconModule.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

// these times assume a 1.000mS/tick timing
#define ONE_SEC 976
#define Looking4Beacon_TIME ONE_SEC/100 

#define CW 1
#define CCW 0

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event DuringCalibrating( ES_Event Event);
static ES_Event DuringLoadingBall( ES_Event Event);
static ES_Event DuringWaiting4ShotComplete( ES_Event Event);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static ShootingState_t CurrentState;

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
ES_Event RunShootingSM( ES_Event CurrentEvent )
{
   bool MakeTransition = false;/* are we making a state transition? */
   ShootingState_t NextState = CurrentState;
   ES_Event EntryEventKind = { ES_ENTRY, 0 };// default to normal entry to new state
   ES_Event ReturnEvent = CurrentEvent; // assume we are not consuming event

   switch ( CurrentState )
   {
			 // CASE 1/3
       case CALIBRATING :       
         // Execute During function 
         CurrentEvent = DuringCalibrating(CurrentEvent);
         //process any events
         if (( CurrentEvent.EventType != ES_NO_EVENT ) && ( CurrentEvent.EventType == READY2SHOOT )) //If an event is active and it's the correct one
         {       
            NextState = LOADING_BALL;//Decide what the next state will be
            MakeTransition = true; //mark that we are taking a transition
            // if transitioning to a state with history change kind of entry
            //EntryEventKind.EventType = ES_ENTRY_HISTORY;
            ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
            break;
          }
				 else if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == Looking4Beacon_TIMER)) // Self Transition
				 {
						NextState = CALIBRATING;
						ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
				 }
				 else if ( CurrentEvent.EventType == ES_NO_EVENT )// Current Event is now ES_NO_EVENT. Correction 2/20/17 
         {     																						//Probably means that CurrentEvent was consumed by lower level
            ReturnEvent = CurrentEvent; // in that case update ReturnEvent too
         }
				 else
				 {
						printf("\r\nERROR: robot in shooting>calibrating with NOT VALID EVENT\r");
				 }
       break;
      
				 // CASE 2/3				 
			 case LOADING_BALL:
			 // During function
       CurrentEvent = DuringLoadingBall(CurrentEvent);
			 // Process events			 
			 if (CurrentEvent.EventType == BALL_FLYING)
				{
					 NextState = WATING4SHOT_COMPLETE;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}							 
				 break;
				
				// CASE 3/3				 
			 case WATING4SHOT_COMPLETE:
			 // During function
       CurrentEvent = DuringWaiting4ShotComplete(CurrentEvent);
			 // Process events			 
			 if (CurrentEvent.EventType == SHOOTING_TIMEOUT)
				{
					 ReturnEvent.EventType = SHOOTING_TIMEOUT; //re-map this event for the upper level state machine
				}							 
				 break;
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
void StartShootingSM ( ES_Event CurrentEvent )
{
   // to implement entry to a history state or directly to a substate
   // you can modify the initialization of the CurrentState variable
   // otherwise just start in the entry state every time the state machine
   // is started
   if ( ES_ENTRY_HISTORY != CurrentEvent.EventType )
   {
        CurrentState = CALIBRATING;
   }
   // call the entry function (if any) for the ENTRY_STATE
   RunShootingSM(CurrentEvent);
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
ShootingState_t QueryShootingSM ( void )
{
   return(CurrentState);
}

/***************************************************************************
 private functions
 ***************************************************************************/

static ES_Event DuringCalibrating( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or consumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // Start ISR for IR frequency detection (Initialization is done in Init function of top SM)
			  EnableIRInterrupt();
			
			  // Start Rotating
			  start2rotate(CCW,80);
			
			  // Start the timer to periodically check the IR frequency
        ES_Timer_InitTimer(Looking4Beacon_TIMER,Looking4Beacon_TIME);
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // Stop Rotating
			  stop();      
    }
		else if (Event.EventType == ES_TIMEOUT && (Event.EventParam == Looking4Beacon_TIMER))
    {
        // Read the detected IR frequency
			  uint8_t IRFreqCode = GetIRCode();
			  // If detected the frequency we are looking for, post READY2SHOOT event
			  if(IRFreqCode == 0xff)                                  // !!!!!!!!!!!!!!!!!!!!!!Need to change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
															                                  // !!!!!!!!!!!!!!!!!Valid Code should be from LOC!!!!!!!!!!!!!!!!!!!
				{
					ES_Event Event2Post;
					Event2Post.EventType = READY2SHOOT;
					Event2Post.EventParam = IRFreqCode;
					PostRobotTopSM(Event2Post);
				}
			  // Else, restart the timer
				else
				{
					ES_Timer_InitTimer(Looking4Beacon_TIMER,Looking4Beacon_TIME);
				}					
				
    }
		else
		{
			
		}
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

static ES_Event DuringLoadingBall( ES_Event Event)
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

static ES_Event DuringWaiting4ShotComplete( ES_Event Event)
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
