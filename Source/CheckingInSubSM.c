 /****************************************************************************
 Module
   CheckingInSubSM.c
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
#include "PWMmodule.h"
#include "LEDModule.h"
#include "CheckingInSubSM.h"
#include "HallEffectModule.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

// these times assume a 1.000mS/tick timing
#define ONE_SEC 976
#define ReportInterval_TIME ONE_SEC/5 //200ms

#define RED 0
#define GREEN 1

// staging area frequency codes
#define code1333us 0 //0000
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

// report status byte masks
#define ACK_MASK 0
#define Inactive_MASK (BIT7HI)
#define NACK_MASK (BIT7HI|BIT6HI)

// query byte masks
#define GAME_STATUS_BIT BIT7HI
#define RESPONSE_READY 0xAA00
#define RESPONSE_NOT_READY 0x0000
#define RESPONSE_READY_MASK 0xff00

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event DuringFirstReportDone( ES_Event Event);
static ES_Event DuringWaiting4FirstResponseReady( ES_Event Event);
static ES_Event DuringSecondReportDone( ES_Event Event);

// static void SetServoAndFlyWheelSpeed(); // Do we still need this?

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static CheckingInState_t CurrentState;
static uint8_t CurrentStageCode = codeInvalidStagingArea;



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
ES_Event RunCheckingInSM( ES_Event CurrentEvent )
{
   bool MakeTransition = false;/* are we making a state transition? */
   CheckingInState_t NextState = CurrentState;
   ES_Event EntryEventKind = { ES_ENTRY, 0 }; // default to normal entry to new state
   ES_Event ReturnEvent = CurrentEvent; // assume we are not consuming event

	 /*
	 FirstReportDone
	 
	 Waiting4FirstResponseReady

	 SecondReportDone
 
	 */
	 
   switch ( CurrentState )
   {
			 // CASE 1/3
			 case FirstReportDone :
				 // Execute During function 
         CurrentEvent = DuringFirstReportDone(CurrentEvent);				 
				 // process events
				 if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == ReportInterval_TIMER))
				 {
					 NextState = Waiting4FirstResponseReady;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				 }
				 break;
		 
		   // CASE 2/3
       case Waiting4FirstResponseReady : 			 
         // Execute During function 
         CurrentEvent = DuringWaiting4FirstResponseReady(CurrentEvent);
         //process any events
			   
			   // Internal Transition
         if (CurrentEvent.EventType == COM_QUERY_RESPONSE) 
         {       
            NextState = Waiting4FirstResponseReady;
            MakeTransition = false; 
            ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
          }
				 else if ( CurrentEvent.EventType == ACK )
         {     																						
            NextState = SecondReportDone;
            MakeTransition = true; 
            ReturnEvent.EventType = ES_NO_EVENT;
         }

         break;
				 
				// CASE 3/3				 
			  case SecondReportDone :  			 		 
			    // During function
				  CurrentEvent = DuringSecondReportDone(CurrentEvent);
				  // Process events	
				
			    // Internal Transition
          if (CurrentEvent.EventType == COM_QUERY_RESPONSE) 
          {       
             NextState = SecondReportDone;
             MakeTransition = false; 
             ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
           }
					// Internal Transition
          if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == ReportInterval_TIMER))
          {       
             NextState = SecondReportDone;
             MakeTransition = false; 
             ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
           }
						
				 break;
    }
    //   If we are making a state transition
    if (MakeTransition == true)
    {
			 printf("\r\n-------------State Transition from %u to %u-----------\r\n",CurrentState,NextState);
       //   Execute exit function for current state
       CurrentEvent.EventType = ES_EXIT;
       RunCheckingInSM(CurrentEvent);

       CurrentState = NextState; //Modify state variable

       //   Execute entry function for new state
       // this defaults to ES_ENTRY
       RunCheckingInSM(EntryEventKind);
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
void StartCheckingInSM ( ES_Event CurrentEvent )
{
   // to implement entry directly to a substate
   // you can modify the initialization of the CurrentState variable
   // otherwise just start in the entry state every time the state machine
   // is started
   if ( ES_ENTRY_HISTORY != CurrentEvent.EventType )
   {
        CurrentState = FirstReportDone;
   }
   // call the entry function (if any) for the ENTRY_STATE
   RunCheckingInSM(CurrentEvent);
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
//ShootingState_t QueryShootingSM ( void )
//{
//   return(CurrentState);
//}

/***************************************************************************
  DuringLooking4Goal
 ***************************************************************************/

static ES_Event DuringFirstReportDone( ES_Event Event)  
{
	ES_Event ReturnEvent = Event; 
	ES_Event Event2Post;
	
	// ****************** ENTRY
	if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
  {
		//Report the 1st Frequency
		CurrentStageCode = returnCurrentStageCode();
		Event2Post.EventType = ROBOT_FREQ_RESPONSE;
		Event2Post.EventParam = CurrentStageCode;
		PostSPIService(Event2Post);
		printf("\r\n 1st Report freq = %u posted to spi \r\n",CurrentStageCode);
		
		// Start the 200ms Timer
		ES_Timer_InitTimer(ReportInterval_TIMER,ReportInterval_TIME);
	}
	// ****************** EXIT
	else if ( Event.EventType == ES_EXIT )
  {
	
	}
	
	// ****************** DURING
	else
	{
		 if (Event.EventType == ES_TIMEOUT && (Event.EventParam == ReportInterval_TIMER)){
			 
		 }
	}
	return ReturnEvent;
}

/***************************************************************************
DuringWaiting4FirstResponseReady
***************************************************************************/
static ES_Event DuringWaiting4FirstResponseReady( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or consumption
		ES_Event Event2Post;

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||(Event.EventType == ES_ENTRY_HISTORY) )
		{
			// Query
			printf("\r\n ROBOT_QUERY to SPI\r\n");
			Event2Post.EventType = ROBOT_QUERY;
			PostSPIService(Event2Post);
		}
    else if ( Event.EventType == ES_EXIT )
    {  				

    }
		//DURING
		else 
		{
		 if (Event.EventType == COM_QUERY_RESPONSE)
		 {
			 //Check if response ready
			 //If Yes, post events accordingly
			 //If not, query again
			 if((Event.EventParam & RESPONSE_READY_MASK) == RESPONSE_READY)
			 {
				  printf("\r\n---------Response Ready---------\r\n");
				  if((Event.EventParam & NACK_MASK) == NACK_MASK)
					{
							printf("\r\n -------NACK"); 
						  // Post KEEP_DRIVING Event and get out of SubSM
						  Event2Post.EventType = KEEP_DRIVING;
							PostRobotTopSM(Event2Post);
					}
					else if((Event.EventParam & NACK_MASK) == Inactive_MASK)
					{
							printf("\r\n -------INACTIVE");
						  // Post KEEP_DRIVING Event and get out of SubSM
						  Event2Post.EventType = KEEP_DRIVING;
						  Event2Post.EventParam = CurrentStageCode;
							PostRobotTopSM(Event2Post);
					}
					else if((Event.EventParam & NACK_MASK) == ACK_MASK)
					{
							printf("\r\n -------ACTIVE");
						  Event2Post.EventType = ACK;
						  PostRobotTopSM(Event2Post);
					}
			 }
			 else if((Event.EventParam & RESPONSE_READY_MASK)== RESPONSE_NOT_READY)
			 {
				 	// Query
				  printf("\r\n Response Not Ready, Re-QUERY to SPI\r\n");
			    Event2Post.EventType = ROBOT_QUERY;
			    PostSPIService(Event2Post);
		   }
		 }	
		}
		
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/***************************************************************************
DuringSecondReportDone
 ***************************************************************************/
static ES_Event DuringSecondReportDone( ES_Event Event)  
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption
		ES_Event Event2Post;

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
				// Read frequency 
			  uint8_t newRead = GetStagingAreaCodeArray();
			  printf("\r\n-------Stage Freq New Read = %u-------\r\n",newRead);
				while((newRead == codeInvalidStagingArea)||(newRead == CurrentStageCode)){
					newRead = GetStagingAreaCodeArray();
				}
				CurrentStageCode = newRead;
			  printf("\r\n ---2ND REPORT newRead: %u---\r\n",CurrentStageCode);
				
				//Report the 2nd frequency
				printf("\r\n 2nd Report freq posted to spi \r\n");
				Event2Post.EventType = ROBOT_FREQ_RESPONSE;
				Event2Post.EventParam = newRead;
				PostSPIService(Event2Post);
				
				// Start the 200ms Timer
		    ES_Timer_InitTimer(ReportInterval_TIMER,ReportInterval_TIME);
    }
    else if ( Event.EventType == ES_EXIT )
    {    
			
    }
		else 
    {  
			if (Event.EventType == COM_QUERY_RESPONSE)
		  {
			 //Check if response ready
			 //If Yes, post events accordingly
			 //If not, query again
			 if((Event.EventParam & RESPONSE_READY_MASK) == RESPONSE_READY)
			 {
				  printf("\r\n---------Response Ready---------\r\n");
					if((Event.EventParam & NACK_MASK) == ACK_MASK)
					{
							printf("\r\n -------2nd ACTIVE");
							// Post CHECK_IN_SUCCESS event and get out of SubSM
							printf("\r\n CHECK_IN_SUCCESS event posted by SubSM to SPI\r\n");
	            Event2Post.EventType = CHECK_IN_SUCCESS;
		          PostRobotTopSM(Event2Post);
					}
					else
					{
							printf("\r\n 2nd response ready but not successful\r\n");
					}
			 }
			 else if((Event.EventParam & RESPONSE_READY_MASK)== RESPONSE_NOT_READY)
			 {
				 	// Query
				  printf("\r\n Response not ready. Re-ROBOT_QUERY to SPI\r\n");
			    ES_Event PostEvent;
			    PostEvent.EventType = ROBOT_QUERY;
			    PostSPIService(PostEvent);
		   }
		  }
			if(Event.EventType == ES_TIMEOUT && (Event.EventParam == ReportInterval_TIMER))
			{
					//Query
				  printf("\r\n ROBOT_QUERY to SPI after the second report\r\n");
					Event2Post.EventType = ROBOT_QUERY;
					PostSPIService(Event2Post);
			}
    }
    return(ReturnEvent);
}
		


