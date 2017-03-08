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
#include "PWMmodule.h"
#include "LEDModule.h"

/*----------------------------- Module Defines ----------------------------*/
// define constants for the states for this machine
// and any other local defines

// these times assume a 1.000mS/tick timing
#define ONE_SEC 976
#define Looking4Beacon_TIME ONE_SEC/100 

#define RED 0
#define GREEN 1

#define CW 1
#define CCW 0

#define Wait4ShotTime 10000 //10sec
#define BucketCode 0x01
#define BeaconRotationDutyCycle 80
#define GOAL_BYTE_MASK 0x00ff
#define LEDS_OFF 0

#define FlyWheelDuty 90
#define SeparatorDuty 30
#define SeparatorONTime 150 //ms

// IR frequency codes
#define code800us 0x00 // 1250Hz (Green supply depot)
#define code690us 0x01 // 1450Hz (Bucket nav beacon)
#define code588us 0x02 // 1700Hz (Red nav beacon)
#define code513us 0x03 // 1950Hz (Red supply depot)
#define code455us 0x04 // 2200Hz (Green nav beacon)


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event DuringLooking4Goal( ES_Event Event);
static ES_Event DuringSettingBallSpeed( ES_Event Event);
static ES_Event DuringWaiting4ShotComplete( ES_Event Event);

static bool DetectAGoal();
// static void SetServoAndFlyWheelSpeed(); // Do we still need this?

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well
static ShootingState_t CurrentState;
static uint8_t BallCount;
static uint8_t MyScore;
static uint8_t Back_MeasuredIRPeriodCode;
static bool setSpeedReady = 0;

static bool firstIRBeaconAlignment = 0;
static bool BeaconRotationDirection = CW;
static uint8_t CurrentStagingAreaPosition;
static uint16_t LOCResponse;
static uint16_t bucketNumber;

static bool GoalFound;
static bool Scored = 0;
static bool Missed = 0;
static bool NoBalls = 0;
static bool ScoreHasBeenSaved = 0;
static uint16_t ScoreBeforeShooting;


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
   ES_Event EntryEventKind = { ES_ENTRY, 0 }; // default to normal entry to new state
   ES_Event ReturnEvent = CurrentEvent; // assume we are not consuming event

	 /*
	 LOOKING4GOAL
		- Rotate and find an active goal, then stop
		- It has to be the correct goal because only one is active at a time
		- Query status and exit
	 
	 SETTING_BALL_SPEED
	  - When we get a status response, we start SETTING_BALL_SPEED
		- Set fly wheel and separator speed as a function of current 
		  active goal and location
	 
	 WATING4SHOT_COMPLETE
	 - Wait for a timeout and then check if you have scored or not 
		 in order to do one thing or another	 
	 */
	 
   switch ( CurrentState )
   {
			 // CASE 1/3
			 case LOOKING4GOAL :
				 printf("\r\n LOOKING4GOAL \r\n");
				 // Execute During function 
         CurrentEvent = DuringLooking4Goal(CurrentEvent);				 
				 // process events
				 if (CurrentEvent.EventType == COM_STATUS)
				 {
					 NextState = SETTING_BALL_SPEED;
					 MakeTransition = true;
					 ReturnEvent.EventType = ES_NO_EVENT;
				 }
				  else if (CurrentEvent.EventType == LOOK4GOAL_AGAIN) // External Self Transition
				 {
						NextState = LOOKING4GOAL;
					  MakeTransition = true;
						ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
				 }
				 break;
		 
		   // CASE 2/3
       case SETTING_BALL_SPEED : 
				 printf("\r\n SETTING_BALL_SPEED \r\n");				 
         // Execute During function 
         CurrentEvent = DuringSettingBallSpeed(CurrentEvent);
         //process any events
         if ( CurrentEvent.EventType == BALL_FLYING ) 
         {       
            NextState = WATING4SHOT_COMPLETE;
            MakeTransition = true; 
            ReturnEvent.EventType = ES_NO_EVENT; // consume for the upper level state machine
          }
				 else if ( CurrentEvent.EventType == ES_NO_EVENT )// Current Event is now ES_NO_EVENT. Correction 2/20/17 
         {     																						//Probably means that CurrentEvent was consumed by lower level
            ReturnEvent = CurrentEvent; // in that case update ReturnEvent too
         }
         break;
				 
				// CASE 3/3				 
			  case WATING4SHOT_COMPLETE :  //SEE ME - not sure how to get out of the sub SM. Hope this works
				  printf("\r\n WATING4SHOT_COMPLETE \r\n");				 		 
			    // During function
				  CurrentEvent = DuringWaiting4ShotComplete(CurrentEvent);
				  // Process events	
					if (CurrentEvent.EventType == KEEP_WAITING4SHOT) //Internal self transition
					{
						NextState = WATING4SHOT_COMPLETE;
            ReturnEvent.EventType = ES_NO_EVENT;
					}					
				  else if (CurrentEvent.EventType == FINISHED_SHOT)
					{ 
						ReturnEvent.EventType = FINISHED_SHOT; //re-map this event for the upper level state machine
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
   // to implement entry directly to a substate
   // you can modify the initialization of the CurrentState variable
   // otherwise just start in the entry state every time the state machine
   // is started
   if ( ES_ENTRY_HISTORY != CurrentEvent.EventType )
   {
        CurrentState = LOOKING4GOAL;
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
  DuringLooking4Goal
 ***************************************************************************/

static ES_Event DuringLooking4Goal( ES_Event Event)  
{
	ES_Event ReturnEvent = Event; 
	ES_Event Event2Post;
	
	// ****************** ENTRY
	if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
  {
		if(ScoreHasBeenSaved == 0)
		{
			// Ask for our current score
			Event2Post.EventType = ROBOT_STATUS;
			PostSPIService(Event2Post);			
		}
		else
		{
		// Start ISR for IR frequency detection (Initialization is done in Init function of top SM)
		EnableBackIRInterrupt();
		
		// Start Rotating
		start2rotate(BeaconRotationDirection,BeaconRotationDutyCycle);

		// Start the timer to periodically check the IR frequency   SEE ME - THERE IS ALREADY AN INTERRUPT
		//ES_Timer_InitTimer(Looking4Beacon_TIMER,Looking4Beacon_TIME);
		}
	}
	// ****************** EXIT
	else if ( Event.EventType == ES_EXIT )
  {
		// Stop Rotating
		stop(); 		
	}
	
	// ****************** DURING
	else
	{
		if (Event.EventType == ES_TIMEOUT && (Event.EventParam == Looking4Beacon_TIMER))
		{
			// Read the detected IR frequency
			Back_MeasuredIRPeriodCode = Back_GetIRCode();  
		
			// DETECT A GOAL
			GoalFound = DetectAGoal();
			printf("Goal found = %i", GoalFound);
			
			// If we are looking at a bucket then query LOC which will trigger moving to the next state
			if(GoalFound == true)
			{
				// Ask for the desired IR frequency from LOC by looking at what goal is active in Status Byte 1
				Event2Post.EventType = ROBOT_STATUS;
				PostSPIService(Event2Post);
				
				// Reset save score flag
				ScoreHasBeenSaved = 1;
			}
			// If we haven't found an active bucket - EXTERNAL SELF TRANSITION to start looking for beacon again
			else
			{
				printf("\r\nLook 4 goal again\r\n");		
				Event2Post.EventType = LOOK4GOAL_AGAIN;
				PostRobotTopSM(Event2Post);
			}					
		}
		
		else if (Event.EventType == COM_STATUS)
		{
			//Save current score 
			ScoreBeforeShooting = GetMyScoreFromStatusResponse(Event.EventParam);
			
			//set flag
			ScoreHasBeenSaved = 1;
			
			//make an external self transition
			Event2Post.EventType = LOOK4GOAL_AGAIN;
			PostRobotTopSM(Event2Post);
		}
	}
	return ReturnEvent;
}

/***************************************************************************
DuringSettingBallSpeed
***************************************************************************/
static ES_Event DuringSettingBallSpeed( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or consumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||(Event.EventType == ES_ENTRY_HISTORY) )
		{
		}
    else if ( Event.EventType == ES_EXIT )
    {  			
			// Speed has been set so the ball is flying towards the goal	
			ES_Event Event2Post;
			Event2Post.EventType = BALL_FLYING;
			PostRobotTopSM(Event2Post);
    }
		//DURING
		else 
		{
			// Get the goal position from the LOC response			
			LOCResponse = Event.EventParam;
			bucketNumber = GetGoalOrStagePositionFromStatus(LOCResponse);
			
			// Get current staging area
			CurrentStagingAreaPosition = GetCurrentStagingAreaPosition();
			
			// Set the speed 
			//	 - servo speed is fixed 
			// 	 - flywheel as a function of where we are  /////SEE ME - If we have time!!			
			SetServoDuty(SeparatorDuty); 
			SetFlyDuty(FlyWheelDuty);	
		}
		
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/***************************************************************************
DuringWaiting4ShotComplete
 ***************************************************************************/
static ES_Event DuringWaiting4ShotComplete( ES_Event Event)  //JUST WAIT AND THEN GET OUT OF SUB SM
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption
		ES_Event Event2Post;

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
				// start 10sec timer
				ES_Timer_InitTimer(Waiting4Shot_TIMER, Wait4ShotTime);	

				// Turn OFF LEDs
				TurnOnOffYellowLEDs(LEDS_OFF, GetTeamColor());	

				// Turn off both the pushing flywheel and the separation servo
				SetServoDuty(0);
				SetFlyDuty(0);			
    }
    else if ( Event.EventType == ES_EXIT )
    {    
    }
		else //DURING - Scored, Missed, or No Balls?
    {  
			// If shot is done				
			if((Event.EventType == ES_TIMEOUT) && (Event.EventParam == Waiting4Shot_TIMER))
			{
					// Substract 1 from ball count independetly of whether we scored or not. 			
					BallCount = BallCount - 1;
					printf("\r\n #balls = %i\r\n", BallCount);
					
					if (BallCount == 0) // go to RELOADING
					{
						printf("\r\n No balls after shooting\r\n");
						Event2Post.EventType = NO_BALLS;
						PostRobotTopSM(Event2Post);
					}
					else // Go back to the robot top state machine and from there post SCORED or MISSED
					{
						printf("\r\n Finished shooting\r\n");
						Event2Post.EventType = FINISHED_SHOT;
						PostRobotTopSM(Event2Post);
					}
				}
			
			// If shot is NOT done - keep waiting
			{
				//internal self transition
				Event2Post.EventType = KEEP_WAITING4SHOT;
				PostRobotTopSM(Event2Post);
			}			
    }
    return(ReturnEvent);
}
		

/****************************************************************************
****************************************************************************
****************************************************************************
****************************************************************************
****************************************************************************
****************************************************************************/

/****************************************************************************************
	AlignWithGoal
	- Find an active goal and return true if you found it
*******************************************************************************************/
static bool DetectAGoal()
{
		// If first beacon aligned TRUE   
	
		// SEE ME - I'm not actually sure what firstIRBeaconAlignment does. I guess make sure that
		// you have aligned with a non-goal type beacon previously
	
		if ( firstIRBeaconAlignment == 1 )
		{
			// reset flag
			firstIRBeaconAlignment = 0;  //SEE ME - I guess this needs to happen here. It was a flag that was never reset!!! - Elena 
			
			// If aligned with 1450Hz beacon - Ready to move on to SHOOTING
			if ( Back_MeasuredIRPeriodCode ==  code690us)		// We compare the value measured with the ISR with the desired code
				return true;
			else
				return false;
		}
		
		
		else{
			
			// (A) If on the GREEN side
			if ( GetTeamColor() == GREEN )
			{
				
				// If aligned1250 TRUE - Rotate CCW
				if ( Back_MeasuredIRPeriodCode == code800us ) //1250Hz
				{
					// set first beacon aligned TRUE
					firstIRBeaconAlignment = 1;
					// rotate counterclockwise
					BeaconRotationDirection = CCW;
				}
					
				// If aligned with 2200Hz beacon - Rotate CW
				else if ( Back_MeasuredIRPeriodCode == code455us )
				{
					// set first beacon aligned TRUE
					firstIRBeaconAlignment = 1;
					// rotate clockwise
					BeaconRotationDirection = CW;
				}
				
			}
				
			// (B) If on the RED side
			else if ( GetTeamColor() == RED )
			{				
				// If aligned 1700Hz - Rotate CCW
				if ( Back_MeasuredIRPeriodCode == code588us )
				{
					// set first beacon aligned TRUE
					firstIRBeaconAlignment = 1;
					// rotate counterclockwise
					BeaconRotationDirection = CCW;
				}
				
				// If aligned 1950Hz - Rotate CW
				else if ( Back_MeasuredIRPeriodCode == code513us )
				{
					// set first beacon aligned TRUE
					firstIRBeaconAlignment = 1;
					// rotate clockwise
					BeaconRotationDirection = CW;
				}
			}
			
			return false;
		}
}

/****************************************************************************************
	GetMyScoreFromStatusResponse
	- Find an active goal and return true if you found it
*******************************************************************************************/
uint16_t GetMyScoreFromStatusResponse( uint16_t StatusResponse)
{
	uint16_t score;	
	// We want to look at SB2 or SB3 four our total # of scores
	score = (StatusResponse & GOAL_BYTE_MASK) ; // VALUE IN BINARY	
	return score;
}

/****************************************************************************************
	GetMyScoreFromStatusResponse
	- Find an active goal and return true if you found it
*******************************************************************************************/
uint16_t GetScoreFromShootingSM()
{
	return ScoreBeforeShooting;
}

/****************************************************************************
GetBallCount
- The ball count needs to be updated when we shoot and when we reload
****************************************************************************/

uint8_t GetBallCount()
{
	return BallCount;
}



