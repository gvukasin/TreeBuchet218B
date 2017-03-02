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
#include "WireSensingModule.h"
#include "IRBeaconModule.h"
#include "LEDModule.h"
#include "DrivingModule.h"
#include "ReloadingSubSM.h"
#include "MotorActionsModule.h"
#include "PWMModule.h"
#include "EventCheckers.h"

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

// to print comments to the terminal
#include <stdio.h>
#include "termio.h" 
#define clrScrn() 	printf("\x1b[2J")


/*----------------------------- Module Defines ----------------------------*/
#define RED_BUTTON BIT4HI
#define ALL_BITS (0xff<<2)

#define RED 0
#define GREEN 1

#define LEDS_ON 1
#define LEDS_OFF 0

#define Time4FrequencyReport 200


#define GAME_STATUS_BIT BIT7HI
#define RESPONSE_READY 0x0000
#define RESPONSE_NOT_READY 0xAA00
#define RESPONSE_READY_MASK 0xff00
#define ACK1_HI BIT15HI
#define ACK0_HI BIT14HI
#define ACK1_LO BIT15LO
#define ACK0_LO BIT14LO

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
#define codeInvalidStagingArea 0xff

// Wire Following Control Defines
// these times assume a 1.000mS/tick timing
#define ONE_SEC 976
#define WireFollow_TIME ONE_SEC/10
#define PWMOffset 80
#define PWMProportionalGain 0.05

// MotorActionDefines
#define FORWARD 1
#define BACKWARD 0

/*---------------------------- Module Functions ---------------------------*/
static ES_Event DuringWaiting2Start( ES_Event Event);
static ES_Event DuringDriving2Staging( ES_Event Event);
static ES_Event DuringCheckIn( ES_Event Event);
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
static uint8_t FrequencyCode;
static int RLCReading[2]; //RLCReading[0] = Left Sensor Reading; RLCReading[1] = Right Sensor Reading
static int PositionDifference;
static bool DoFirstTimeFlag;
static uint16_t CurrentButtonState;
static uint16_t LastButtonState;
static bool TeamColor;
static bool CheckFlag = 1;
static uint8_t BallCount = 3; //We will start with 3 balls
static uint8_t PreviousScore = 0;

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
	
  InitializeTeamButtonsHardware();   
	
	// Initialize PWM hardware to drive the motors
	InitializePWM();
	
	// Initialize RLC hardware 
	InitRLCSensor();
	
	// Initialize hardware for IR but not kicking the timer off 
	InitInputCaptureForIRDetection();
	
		// Initialize TIMERS
	// Initialize 200ms timer for handshake
	ES_Timer_SetTimer(FrequencyReport_TIMER, Time4FrequencyReport);

  	
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
	 //printf("\r\n event : %i\r\n", CurrentEvent.EventType);
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
			 case DRIVING2STAGING:   //USE TEAM_OPTION VARIABLE FOR DIFFERENT DRIVING			 	 
			 // During function
       CurrentEvent = DuringDriving2Staging(CurrentEvent);	 
			 // Process events			 
			 if (CurrentEvent.EventType == STATION_REACHED)
				{
					 //printf("\r\nReceived STATION_REACHED event at DRIVING2STAGING state \r\n");
					 NextState = CHECKING_IN;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}
				if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == WireFollow_TIMER))
				{
					 //printf("\r\nReceived TIME_OUT event at DRIVING2STAGING state \r\n");			
					 // Internal self transition
					 NextState = DRIVING2STAGING;
					 ReturnEvent.EventType = ES_NO_EVENT;
				}
				break;
				
			 // CASE 3/8				 
			 case CHECKING_IN:
				printf("\r\n checking in \r\n");
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
						 case ES_TIMEOUT:
								if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == FrequencyReport_TIMER))
									NextState = CHECKING_IN; // Internal Self transition
								break;
						 case COM_QUERY_RESPONSE:
								NextState = CHECKING_IN; // Internal Self transition
								break;
						 case FINISH_STRONG :
							 NextState = ENDING_STRATEGY;
							 MakeTransition = true;
							 break;	
						case START :
							 NextState = DRIVING2STAGING;
							 MakeTransition = true;
							 break;						 
						 default:
							 if(ES_ENTRY|ES_EXIT){}
							 else
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
							 if(ES_ENTRY|ES_EXIT){}
							 else
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
       CurrentEvent = DuringStop(CurrentEvent);	
			 break;				 
		}
 	 
    //   If we are making a state transition
    if (MakeTransition == true)
    {
			 printf("\r\nState Transition Made\r\n");
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
	
	//SEE ME
  //CurrentState = WAITING2START;
	CurrentState = DRIVING2STAGING;
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
			//Post team color to SPIService
				ES_Event PostEvent;
				PostEvent.EventType = TEAM_COLOR;			
			
				if (TeamColor == RED)
					PostEvent.EventParam = 0;
			
				else 
					PostEvent.EventParam = 1;
				
				PostSPIService(PostEvent);
				
    }
    else if ( Event.EventType == ES_EXIT )
    { 
    }
		
		else if(Event.EventType == COM_STATUS)
		{
			// printf("\r\nCOM_STATUS: %x \r\n",Event.EventParam);
							
			// check game status bit
			if( ((Event.EventParam & GAME_STATUS_BIT) == GAME_STATUS_BIT) && (Event.EventParam != 0xff))
			{			
				printf("\r\n start \r\n");
				// change return event to START to begin the game
				ReturnEvent.EventType = START;
			}			
			else 
			{		
				//printf("\r\n ask loc again 1\r\n");					
				//ask LOC for GAME STATUS again (until it says we're ready to start)
				ES_Event PostEvent;
				PostEvent.EventType = ROBOT_STATUS;	
				PostSPIService(PostEvent);
			}
		}
		else 
		{	
			printf("\r\n ask loc again 2\r\n");					
			//ask LOC for GAME STATUS again (until it says we're ready to start)
			ES_Event PostEvent;
			PostEvent.EventType = ROBOT_STATUS;	
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
			printf("\r\nDriving2Staging ENTRY\r\n");
			
			// When getting into this state from other states,
			// Start the timer to periodically read the sensor values
			ES_Timer_InitTimer(WireFollow_TIMER,WireFollow_TIME);
			
			// Initialize stage area frequency reading
			InitStagingAreaISR();			
    }
    else if ( Event.EventType == ES_EXIT )
    {
			// Stop the motor
			stop();
    }
		
		// do the 'during' function for this state
		else if (Event.EventType == ES_TIMEOUT && (Event.EventParam == WireFollow_TIMER))
    {
			// Read the RLC sensor values
			// Positive when too left, negative when too right
			ReadRLCSensor(RLCReading);
			PositionDifference = RLCReading[1] - RLCReading[0];
			
			// P Control
			int PWMLeft = (float)PWMOffset + (float)PWMProportionalGain * PositionDifference;
			int PWMRight = (float)PWMOffset - (float)PWMProportionalGain * PositionDifference;
			  
			//Clamp the value to 0-100
			if(PWMLeft < 0){
				PWMLeft = 0;
			}else if(PWMLeft > 100){
				PWMLeft = 100;
			}
				
			if(PWMRight < 0){
				PWMRight = 0;
			}else if(PWMRight > 100){
				PWMRight = 100;
			}
			 
			//printf("\r\nRLC:Left=%d,Right=%d,Difference=%d,LeftDuty=%u,RightDuty=%u\r\n",RLCReading[0],RLCReading[1],PositionDifference,PWMLeft,PWMRight);
			
			// Drive the motors using new PWM duty cycles
			driveSeperate(PWMLeft,PWMRight,FORWARD);
			//printf("\r\ndrive\r\n");
			
			// Restart the timer
			ES_Timer_InitTimer(WireFollow_TIMER,WireFollow_TIME);
			
			// Check if a staging area has been reached
			FrequencyCode = GetStagingAreaCode();
			printf("\r\nstaging area code=%u \r\n",FrequencyCode);
			
			if(FrequencyCode != codeInvalidStagingArea)
			{
				//printf("\r\nstage detected in Driving2Stage during routine\r\n");
				ES_Event PostEvent;
			  PostEvent.EventType = STATION_REACHED;
			  PostRobotTopSM(PostEvent); // Move to the CheckingIn state
			}
    }
		else{
			
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
//			PostEvent.EventType = ROBOT_STATUS;
//			PostSPIService(PostEvent);
//			if (TeamColor == RED)
			 
			// Check time		
			DoFirstTimeFlag = 1;			
    }
    else if ( Event.EventType == ES_EXIT)
    {
    }
		
		// do the 'during' function for this state
		else 
    {	
			// ONLY DO (1) & (2) ON FIRST ENTRY TO THIS DURING FUNCTION
			if(DoFirstTimeFlag)
			{
			//(1) Report frequency
			printf("\r\n Report freq posted to spi \r\n");
			PostEvent.EventType = ROBOT_FREQ_RESPONSE;
			PostEvent.EventParam = FrequencyCode;
			PostSPIService(PostEvent);
										
			 //(2) Start 200ms timer
			 ES_Timer_StartTimer(FrequencyReport_TIMER);
				
			// reset flag
				DoFirstTimeFlag = 0;
			}
			
			/* (3) If there has been a timeout -which means the reporting process 
						 has had time to be completed- Query until LOC returns a Response Ready */
			if (((Event.EventType == ES_TIMEOUT) && (Event.EventParam == FrequencyReport_TIMER)) || (Event.EventType == QUERY_AGAIN))
			{
				printf("\r\n ROBOT_QUERY to SPI\r\n");
				PostEvent.EventType = ROBOT_QUERY;
			  PostSPIService(PostEvent);
				printf("\r\n Robot query end \r\n");	
			}    

			//(4) Has the LOC received our frequency and is it correct? 
			if (Event.EventType == COM_QUERY_RESPONSE)
			{
				if((Event.EventParam & RESPONSE_READY_MASK)== RESPONSE_NOT_READY)
				{
					PostEvent.EventType = QUERY_AGAIN;
					PostRobotTopSM(PostEvent);
				}
				else if((Event.EventParam & RESPONSE_READY_MASK) == RESPONSE_READY) 
				{
					printf("\r\nResponse ready\r\n");
					
					// NACK - wrong frequency
					if(((Event.EventParam & ACK1_HI) == ACK1_HI) && ((Event.EventParam & ACK0_HI) == ACK0_HI))
					{
						printf("\r\nERROR: Reported the WRONG FREQUENCY!"); // WE SHOULD DO MORE THAN THROW AN ERROR
					}
					
					// INACTIVE - wrong staging area
					if(((Event.EventParam & ACK1_HI) == ACK1_HI) && ((Event.EventParam | ACK0_LO) == ACK0_LO))
					{
						//Go to DRIVING2STAGING
						PostEvent.EventType = START;
						PostRobotTopSM(PostEvent);	
					}
					
					// ACK - all good! 
					if(((Event.EventParam | ACK1_LO) == ACK1_LO) && ((Event.EventParam | ACK0_LO) == ACK0_LO))
					{
						//Go to SHOOTING
						PostEvent.EventType = CHECK_IN_SUCCESS;
						PostRobotTopSM(PostEvent);	
					}
				}
			}				
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
			
			// ASK LOC FOR CURRENT SCORE AND COMPARE TO OLD TO KNOW IF YOU SCORED
			// POST SCORED TO ROBOT IF YOU DID
				
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
	HWREG(SYSCTL_RCGCGPIO)|= SYSCTL_RCGCGPIO_R5;
 	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R5) != SYSCTL_PRGPIO_R5);

	// Set bit 4 in port F as digital input:
 	HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= RED_BUTTON;	
 	HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) &= ~(RED_BUTTON);
	
	// read state of button and set team color accordingly (RED = 0, GREEN = 1)
	uint16_t PinState;
	PinState = HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + ALL_BITS)) & RED_BUTTON;
	TeamColor = PinState;

	printf("\r\n Button: %x \r\n", PinState);

}

/*********************************************************  THE END *************************************************************/
