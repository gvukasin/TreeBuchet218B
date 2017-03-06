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
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"

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
//#define TEST

/*----------------------------- Module Defines ----------------------------*/
#define RED_BUTTON BIT4HI
#define ALL_BITS (0xff<<2)

// using 40 MHz clock
#define TicksPerMS 40000

#define GameTimeoutMS 120000 //2min

#define RED 0
#define GREEN 1

#define LEDS_ON 1
#define LEDS_OFF 0

#define Time4FrequencyReport 200

// query byte masks
#define GAME_STATUS_BIT BIT7HI
#define RESPONSE_READY 0x0000
#define RESPONSE_NOT_READY 0xAA00
#define RESPONSE_READY_MASK 0xff00

// report status byte masks
#define ACK1_HI BIT15HI
#define ACK0_HI BIT14HI
#define ACK1_LO BIT15LO
#define ACK0_LO BIT14LO

// status byte 1 masks
#define G1 BIT12HI //GREEN goal #i
#define G2 BIT13HI
#define G3 (BIT12HI | BIT13HI)
#define G_ALL_GOALS BIT14HI
#define R1 BIT8HI //RED goal #i
#define R2 BIT9HI
#define R3 (BIT8HI | BIT9HI)
#define R_ALL_GOALS BIT10HI

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

// Wire Following Control Defines
// these times assume a 1.000mS/tick timing
#define ONE_SEC 976
#define WireFollow_TIME ONE_SEC/10
#define PWMOffset 70
#define PWMProportionalGain 0.08
#define PWMDerivativeGain 0.01
#define NoWireDetectedReading 2000

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
//static uint16_t SaveStagingPosition( uint16_t );

static void InitGameTimer(void);
static void SetTimeoutAndStartGameTimer( uint32_t GameTimerTimeoutMS );

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, though if the top level state machine
// is just a single state container for orthogonal regions, you could get
// away without it
static RobotState_t CurrentState;
static uint8_t MyPriority;
static uint16_t PeriodCode;
static int RLCReading[2]; //RLCReading[0] = Left Sensor Reading; RLCReading[1] = Right Sensor Reading
static int RLCReading_Left;
static int RLCReading_Right;
static int PositionDifference;
static int PositionDifference_dt;
static bool CheckOnWireFlag_Left;
static bool CheckOnWireFlag_Right;
static uint16_t CurrentButtonState;
static uint16_t LastButtonState;
static bool TeamColor;
static uint8_t BallCount = 3; //We will start with 3 balls
static uint8_t PreviousScore = 0;
static uint8_t CurrentStagingArea;
static uint8_t NextStagingArea;
static uint16_t LastPeriodCode = 0xff;
static uint16_t PeriodCodeCounter = 0;
static uint16_t MaxPeriodCodeCount = 5;
static uint8_t NumberOfCorrectReports = 0;
static uint8_t newRead;
static bool ValidSecondCode = 1;
static uint32_t OneShotTimeoutMS;


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
	InitInputCaptureForFrontIRDetection();
	
	// Initialize stage area frequency reading
	InitStagingAreaISR();
	
	// Initialize 200ms timer for handshake
	ES_Timer_SetTimer(FrequencyReport_TIMER, Time4FrequencyReport);
	
	// Initialize game timer
	InitGameTimer();
	
	// Initialize fly wheel pin
		InitializeAltPWM();

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
			 // printf("\r\n current event: %i",CurrentEvent.EventType);
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
						case REPORT_SECOND_TIME: // External self transition
							 NextState = CHECKING_IN;
							 MakeTransition = true;
							 break;	
						 default:
							 if(ES_ENTRY|ES_EXIT){} //SEE ME: This might actually throw an error (Elena). And it's in other states too
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
			 if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == Waitin4Ball_TIMER))
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
			 printf("\r\n T %i\r\n",CurrentState);
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
	//Initial state
	// SEE ME
	CurrentState = WAITING2START;
	//CurrentState = DRIVING2STAGING;
	
  // now we need to let the Run function init the lower level state machines
  // use LocalEvent to keep the compiler from complaining about unused var
  RunRobotTopSM(CurrentEvent);
  return;
}


/******************************************************************
Function 
	GetTeamColor
************************************************************************/
bool GetTeamColor()
{
	return TeamColor;
}


/******************************************************************
Function 
	GetCurrentStagingArea
************************************************************************/       
uint8_t GetCurrentStagingAreaPosition()
{
	return CurrentStagingArea ;
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
				
				//Turn on TeamColor LEDs
				TurnOnOFFTeamColorLEDs(LEDS_ON, TeamColor);		
				
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
				//Start game timer
				SetTimeoutAndStartGameTimer(GameTimeoutMS);
				printf("START: time 0");
				
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
			
			EnableStagingAreaISR();
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
			ReadRLCSensor(RLCReading);
			RLCReading_Right = RLCReading[1];
			RLCReading_Left = RLCReading[0];
			// Positive when too left (right at higher voltage)
			// Negative when too right (left at higher ovltage)
			PositionDifference = RLCReading_Right - RLCReading_Left;
			PositionDifference_dt = PositionDifference/WireFollow_TIME;
			
			// Set flags to determine if either side is on the wire
			// Check if left side is on the wire
			// SEE ME
			if ( RLCReading_Left > (int)NoWireDetectedReading ){
				CheckOnWireFlag_Left = 1;
			}
			else{
				CheckOnWireFlag_Left = 0;
			}
			// Check if right side is on the wire
			if ( RLCReading_Right > (int)NoWireDetectedReading ){
				CheckOnWireFlag_Right = 1;
			}
			else{
				CheckOnWireFlag_Right = 0;
			}
			
			// P Control
			int PWMLeft = (float)PWMOffset + (float)PWMProportionalGain * PositionDifference;
			int PWMRight = (float)PWMOffset - (float)PWMProportionalGain * PositionDifference;
			
			// SEE ME

			// PD Control
			// int PWMLeft = (float)PWMOffset + (float)PWMProportionalGain * PositionDifference + (float)PWMDerivativeGain * PositionDifference_dt;
			// int PWMRight = (float)PWMOffset - (float)PWMProportionalGain * PositionDifference - (float)PWMDerivativeGain * PositionDifference_dt;
			
			// SEE ME
			// If either side is on the wire, check if either side is off the wire
			// Then, if either side is off the wire, turn off the side that is on the wire
			if ( CheckOnWireFlag_Left || CheckOnWireFlag_Right )
			{
				// If left side is off the wire, turn off the right motor
				if ( CheckOnWireFlag_Left == 0 ){
					// turn right wheel off
					PWMRight = 0;
				}
				// If right side is off the wire, turn off the left motor
				else if ( CheckOnWireFlag_Right == 0 ){
					// turn left wheel off
					PWMLeft = 0;
				}
			}
			  
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
			 
			printf("\r\nRLC:Left=%d,Right=%d,Difference=%d,LeftDuty=%u,RightDuty=%u,LeftWire=%i,RightWire=%i\r\n",RLCReading[0],RLCReading[1],PositionDifference,PWMLeft,PWMRight,CheckOnWireFlag_Left,CheckOnWireFlag_Right);
			
			// Drive the motors using new PWM duty cycles
			driveSeperate(PWMLeft,PWMRight,FORWARD);
			//printf("\r\ndrive\r\n");
			
			// Restart the timer
			ES_Timer_InitTimer(WireFollow_TIMER,WireFollow_TIME);
			
			// Check if a staging area has been reached
			PeriodCode = GetStagingAreaCodeArray();
			printf("\r\nstaging area code=%i\r\n",PeriodCode);
			
			if(PeriodCode != codeInvalidStagingArea)
			{ 
				ES_Event Event2Post;
				Event2Post.EventType = STATION_REACHED;
				Event2Post.EventParam = PeriodCode;
				PostRobotTopSM(Event2Post);
			}
    }

    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
		LastPeriodCode = PeriodCode;
    return(ReturnEvent);
}

static ES_Event DuringCheckIn( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption
		ES_Event PostEvent;
	
    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    { 
			// This won't be run if we just want to query again about the same report
			ValidSecondCode = 1;
			
			if (NumberOfCorrectReports == 1) //SECOND REPORT - read new frequency and update PeriodCode
			{
				/* Before reporting the second new read period we must check that:
							1) it is different to the previous one 
							2) it is a valid code 
				*/
				newRead = GetStagingAreaCodeArray();
				ValidSecondCode = ((newRead != codeInvalidStagingArea) & (newRead != PeriodCode));
			}
			
			// (1) REPORT and (2) START TIMER
			if (ValidSecondCode) //report the second time ONLY if the code is incorrect
			{			
				if (NumberOfCorrectReports == 1) //update code value before reporting 
				{
					// Update code 
					PeriodCode = newRead;
				}
			
				//Report frequency
				printf("\r\n Report freq posted to spi \r\n");
				PostEvent.EventType = ROBOT_FREQ_RESPONSE;
				PostEvent.EventParam = PeriodCode;
				PostSPIService(PostEvent);						
			 
				//Start 200ms timer
				ES_Timer_StartTimer(FrequencyReport_TIMER);	
			}
    }
		
    else if ( Event.EventType == ES_EXIT)
    {
    }
		
		// do the 'during' function for this state
		else 
    {	
			if (ValidSecondCode == 1) // During the first report this will be 1 so we will go into this during
			{

				/* (3) If there has been a timeout -which means the reporting process 
							 has had time to be completed- Query until LOC returns a Response Ready */
				if (((Event.EventType == ES_TIMEOUT) && (Event.EventParam == FrequencyReport_TIMER)) || (Event.EventType == QUERY_AGAIN))
				{
					printf("\r\n ROBOT_QUERY to SPI\r\n");
					PostEvent.EventType = ROBOT_QUERY;
					PostSPIService(PostEvent);
				}    

				//(4) Has the LOC received our frequency and is it correct? 
				else if (Event.EventType == COM_QUERY_RESPONSE)
				{
					if((Event.EventParam & RESPONSE_READY_MASK)== RESPONSE_NOT_READY) // Did NOT receive
					{
						PostEvent.EventType = QUERY_AGAIN;
						PostRobotTopSM(PostEvent);
					}
					else if((Event.EventParam & RESPONSE_READY_MASK) == RESPONSE_READY) // YES received
					{
						printf("\r\nResponse ready\r\n");
						
						// NACK - wrong frequency
						if(((Event.EventParam & ACK1_HI) == ACK1_HI) && ((Event.EventParam & ACK0_HI) == ACK0_HI))
						{
							printf("\r\nERROR: Reported the WRONG FREQUENCY! We will REPORT AGAIN\r\n"); 
							
							//Try reporting again
							PostEvent.EventType = STATION_REACHED;
							PostRobotTopSM(PostEvent);
						}
						
						// INACTIVE - wrong staging area
						if(((Event.EventParam & ACK1_HI) == ACK1_HI) && ((Event.EventParam | ACK0_LO) == ACK0_LO))
						{
							
							// record current driving stage (we will need next staging area too to know which direction to drive in!)
							CurrentStagingArea = GetGoalOrStagePositionFromStatus(Event.EventParam);
							
							//Go to DRIVING2STAGING
							PostEvent.EventType = START;
							PostRobotTopSM(PostEvent);	
						}
						
						// ACK - all good! 
						if(((Event.EventParam | ACK1_LO) == ACK1_LO) && ((Event.EventParam | ACK0_LO) == ACK0_LO))
						{
							// Add 1 to number of correct reports
							NumberOfCorrectReports++;
							
							// record current driving stage (SEE ME: might set the next staging area as the current staging area)
							CurrentStagingArea = GetGoalOrStagePositionFromStatus(Event.EventParam);
							
							if (NumberOfCorrectReports == 2)
							{	
								//Go to SHOOTING
								PostEvent.EventType = CHECK_IN_SUCCESS;
								//PostEvent.EventParam = CurrentStagingArea;
								PostRobotTopSM(PostEvent);	
							}
							else if (NumberOfCorrectReports == 1)							
							{ 							
								//Read new frequency and Report again --> repeat CHECKING IN
								PostEvent.EventType = REPORT_SECOND_TIME; //This will lead to an external self transition
								PostRobotTopSM(PostEvent);
							}
							else
							{
								printf("\r\nWARNING: The number of correct reports is a WEIRD #: %i", NumberOfCorrectReports);
							}
						}
					} 
				}
			}
			else if (ValidSecondCode == 0)
			{
					//read again
					PostEvent.EventType = REPORT_SECOND_TIME; //external self transition
					PostRobotTopSM(PostEvent);
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
				TurnOnOffYellowLEDs(LEDS_ON, TeamColor);
			
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
				TurnOnOffYellowLEDs(LEDS_OFF, TeamColor);
    }
		
		// do the 'during' function for this state
		else 
    {
			// run any lower level state machine
        ReturnEvent = RunShootingSM(Event);  
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
 
    }
    else if ( Event.EventType == ES_EXIT )
    {
  
    }
		else // do the 'during' function for this state
    {
      stop();
			TurnOnOFFTeamColorLEDs(LEDS_OFF, TeamColor);
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/****************************************************************************
 InitializeTeamButtonsHardware
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

/****************************************************************************
 SaveStagingPosition
****************************************************************************/
uint16_t GetGoalOrStagePositionFromStatus( uint16_t StatusResponse )
{
	
	// set staging area variable from status bytes if we are Red team
	uint16_t ReturnPosition = 0;
	
	if(TeamColor == RED){
		
		if((StatusResponse & R1) == R1){
			// set current staging area variable to R1	
			ReturnPosition = 1;
			
		} else if (StatusResponse & R2){
			
			// set current staging area variable to R2
			ReturnPosition = 2;
			
		} else if (StatusResponse & R3) {
			
			// set current staging area variable to R3
			ReturnPosition = 3;
		}
		
	// set staging area variable from status bytes if we are Green team
	} else {
		if((StatusResponse & G1) == G1){
			// set current staging area variable to G1	
			ReturnPosition = 1;
			
		} else if (StatusResponse & G2){
			
			// set current staging area variable to G2
			ReturnPosition = 2;
			
		} else if (StatusResponse & G3) {
			
			// set current staging area variable to G3
			ReturnPosition = 3;
		}
	}
	return ReturnPosition;
}

/****************************************************************************
GAME TIMER 
****************************************************************************/

static void InitGameTimer() //Wide Timer 1 subtimer B
{	
	// start by enabling the clock to the timer (Wide Timer 1)
	HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1;

	// kill a few cycles to let the clock get going
	while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R1) != SYSCTL_PRWTIMER_R1){}

	// make sure that timer (Timer B) is disabled before configuring
	HWREG(WTIMER1_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN; //TAEN = Bit1

	// set it up in 32bit wide (individual, not concatenated) mode
	// the constant name derives from the 16/32 bit timer, but this is a 32/64
	// bit timer so we are setting the 32bit mode
	HWREG(WTIMER1_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT; //bits 0-2 = 0x04

	// set up timer B in 1-shot mode so that it disables timer on timeouts
	// first mask off the TAMR field (bits 0:1) then set the value for
	// 1-shot mode = 0x01
	HWREG(WTIMER1_BASE+TIMER_O_TBMR) = (HWREG(WTIMER1_BASE+TIMER_O_TBMR)& ~TIMER_TBMR_TBMR_M)| TIMER_TBMR_TBMR_1_SHOT;
	
	// set timeout
	OneShotTimeoutMS = 1000; //arbitrary initialization value
	HWREG(WTIMER1_BASE+TIMER_O_TBILR) = TicksPerMS*OneShotTimeoutMS;
	
	// enable a local timeout interrupt. TBTOIM = bit 1
	HWREG(WTIMER1_BASE+TIMER_O_IMR) |= TIMER_IMR_TBTOIM; // bit1

	// enable the Timer B in Wide Timer 1 interrupt in the NVIC
	// it is interrupt number 96 so appears in EN3 at bit 0
	HWREG(NVIC_EN3) |= BIT0HI;
	
	// set priority of this timer 
	HWREG(NVIC_PRI24) |= NVIC_PRI24_INTA_M;

	// make sure interrupts are enabled globally
	__enable_irq();
	
	printf("\r\nGAME TIMER init done\r\n");
}

static void SetTimeoutAndStartGameTimer( uint32_t GameTimerTimeoutMS )
{
	// set timeout
	HWREG(WTIMER1_BASE+TIMER_O_TBILR) = TicksPerMS*GameTimerTimeoutMS;
	
	// now kick the timer off by enabling it and enabling the timer to stall while stopped by the debugger
	HWREG(WTIMER1_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
}

void GameTimerISR(void)
{	
	// clear interrupt
	HWREG(WTIMER1_BASE+TIMER_O_ICR) = TIMER_ICR_TBTOCINT; 
	
	// post event to go into ENDING_STRATEGY state
	ES_Event PostEvent;
	PostEvent.EventType = FINISH_STRONG;
	PostRobotTopSM(PostEvent);
	
}

#ifdef TEST
#include "termio.h"
#define clrScrn() 	printf("\x1b[2J")

int main(void){
	
	// Set the clock to run at 40MhZ using the PLL and 16MHz external crystal
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN| SYSCTL_XTAL_16MHZ);
	TERMIO_Init();
	clrScrn();
	printf("\r\n Game Timer Test \r\n");
	InitGameTimer();	
}

#endif
/*********************************************************  THE END *************************************************************/
