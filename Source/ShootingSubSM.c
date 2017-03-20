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
#define IRAligning_TIME 100//ONE_SEC/20
#define FlyWheel_TIME ONE_SEC*4

#define AligningSpeed 40

#define RED 0
#define GREEN 1

#define CW 1
#define CCW 0

//#define Wait4ShotTime 10000 //10sec
#define Wait4ShotTime  ONE_SEC*7 //15sec
#define BucketCode 0x01
#define BeaconRotationDutyCycle 80
#define GOAL_BYTE_MASK 0x00ff
#define LEDS_OFF 0

#define FlyWheelDuty 80
#define SeparatorDuty 50
#define SeparatorONTime 1000 //ms

// IR frequency codes
#define code800us 0x00 // 1250Hz (Green supply depot)
#define code690us 0x01 // 1450Hz (Bucket nav beacon)
#define code588us 0x02 // 1700Hz (Red nav beacon)
#define code513us 0x03 // 1950Hz (Red supply depot)
#define code455us 0x04 // 2200Hz (Green nav beacon)
#define BucketCode 0x01 // 1450Hz

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine, things like during
   functions, entry & exit functions.They should be functions relevant to the
   behavior of this state machine
*/
static ES_Event DuringLooking4Bucket( ES_Event Event);
static ES_Event DuringSettingBallSpeed( ES_Event Event);
static ES_Event DuringWaiting4ShotComplete( ES_Event Event);
static bool DetectAGoal();

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

static bool TeamColor = GREEN;
static uint8_t GoalCode;
static uint8_t Front_MeasuredIRPeriodCode;
static bool rotationDirection;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    RunShootingSM

 Parameters
   ES_Event: the event to process

 Returns
   ES_Event: an event to return

 Description
   Run function for this sub state machine
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
	 LOOKING4BUCKET
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
	
	// Switch the CurrentState
   switch ( CurrentState )
   {
			
			 // CASE 1/3, LOOKING4BUCKET			 
			 case LOOKING4BUCKET :
				 // Execute During function 
         		CurrentEvent = DuringLooking4Bucket(CurrentEvent);				 
				 // If current event is BucketAligned
				 if (CurrentEvent.EventType == BucketAligned)
				 {
				 	// Change NextState to SETTING_BALL_SPEED
					 NextState = SETTING_BALL_SPEED;
					 // Set MakeTransition to true
					 MakeTransition = true;
					 // Set ReturnEvent to ES_NO_EVENT
					 ReturnEvent.EventType = ES_NO_EVENT;
				 }
				 // If CurrentEvent is ES_TIMEOUT and the param IRAligning_TIMER
				 else if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == IRAligning_TIMER))
				 {
					// Internal Transition
					NextState = LOOKING4BUCKET;
					// Set MakeTransition to false
					MakeTransition = false;
					// Set ReturnEvent to ES_NO_EVENT, consume for the upper level state machine
					ReturnEvent.EventType = ES_NO_EVENT; 
				 }
				 break;
				 
		// CASE 2/3, SETTING_BALL_SPEED
       case SETTING_BALL_SPEED : 				 
         // Execute During function 
         CurrentEvent = DuringSettingBallSpeed(CurrentEvent);
			 
		// Wait for the flywheel to rotate for a while to reach desired speed
         // If CurrentEvent is ES_TIMEOUT and the param is IRAligning_TIMER
         if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == IRAligning_TIMER))
         {       
         	// Change the state to WAITING4SHOT_COMPLETE
            NextState = WATING4SHOT_COMPLETE;
            // Set MakeTransition to true
            MakeTransition = true; 
            // Set ReturnEvent to ES_NO_EVENT, consume for upper level state machine
            ReturnEvent.EventType = ES_NO_EVENT;
         }
         break;
				 
		// CASE 3/3, WATING4SHOT_COMPLETE		 
		 case WATING4SHOT_COMPLETE :  			 		 
			// During function
			CurrentEvent = DuringWaiting4ShotComplete(CurrentEvent);
			// If CurrentEvent is ShotComplete		 
			if (CurrentEvent.EventType == ShotComplete)
         {  
         	// Consume for the upper level state machine
            ReturnEvent.EventType = ES_NO_EVENT; 
         }
         // Else If CurrentEvent is ES_TIMEOUT and param is IRAligning_TIMER
         else if (CurrentEvent.EventType == ES_TIMEOUT && (CurrentEvent.EventParam == IRAligning_TIMER))
         {  
			// Internal Transition
			// Set state to WATING4SHOT_COMPLETE
            NextState = WATING4SHOT_COMPLETE;
            // Set MakeTransition to false
            MakeTransition = false; 
            // consume for the upper level state machine
            ReturnEvent.EventType = ES_NO_EVENT; 
         }							
				 break;
				 			 
    }
	 
    // If we are making a state transition
    if (MakeTransition == true)
    {
       // Execute exit function for current state
       CurrentEvent.EventType = ES_EXIT;
       RunShootingSM(CurrentEvent);

       // Modify state variable
       CurrentState = NextState; 

       // Execute entry function for new state
       // this defaults to ES_ENTRY
       RunShootingSM(EntryEventKind);
     }
     // Return ReturnEvent
     return(ReturnEvent);
}
/****************************************************************************
 Function
     StartShootingSM

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
void StartShootingSM ( ES_Event CurrentEvent )
{
   // If current event is ES_ENTRY_HISTORY
   if ( ES_ENTRY_HISTORY != CurrentEvent.EventType )
   {
   		// change current state to LOOKING4BUCKET
        CurrentState = LOOKING4BUCKET;
   }
   // Call the entry function (if any) for the ENTRY_STATE
   RunShootingSM(CurrentEvent);
}

/****************************************************************************
 Function
     QueryShootingSM

 Parameters
     None

 Returns
     ShootingState_t The current state of this state machine

 Description
     Returns the current state of this state machine
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/
ShootingState_t QueryShootingSM ( void )
{
	// Return the current state
   return(CurrentState);
}

/***************************************************************************
 Function
     DuringLooking4Bucket

 Parameters
     ES_Event Event

 Returns
     ES_Event Return Event, event to return to the rest of the state machine

 Description
     During function for the Looking4Bucket
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/

static ES_Event DuringLooking4Bucket( ES_Event Event)  
{
	// Create variable for ReturnEvent
	ES_Event ReturnEvent = Event; 
	ES_Event Event2Post;
	
	// If Event is ES_ENTRY or ES_ENTRY_HISTORY
	if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
  {
		// Get team color
		TeamColor = GetTeamColor();
		// If TeamColor is Green
		if(TeamColor == GREEN){
			// Set GoalCode to 1950 Hz
			GoalCode = code513us; //1950 Hz
			// Set rotation direction to ccw
			rotationDirection = CCW;
		// Else If TeamColor is red
		}else if(TeamColor == RED){
			// Set GoalCode to 1250 Hz
		  GoalCode = code800us; //1250 Hz
		  // Set rotation diretion to cw
			rotationDirection = CW;
		}
		
		//Enable ISR for front IR (Initialized in TopSM Initialization)
		EnableFrontIRInterrupt();
		
		//Start rotating slowly
		start2rotate(rotationDirection, AligningSpeed);
		
		// Start the timer to periodically check the IR frequency   
		ES_Timer_InitTimer(IRAligning_TIMER,IRAligning_TIME);

	}
	// Else If ES_EXIT	
	else if ( Event.EventType == ES_EXIT )
  {
		// Stop Rotating
		stop(); 		
	}
	
	// Else, do the during function
	else
	{
		// If Event is ES_TIMEOUT and param is IRAligning_TIMER
		if (Event.EventType == ES_TIMEOUT && (Event.EventParam == IRAligning_TIMER))
		{
			// Read the detected IR frequency
			Front_MeasuredIRPeriodCode = Front_GetIRCode();		

			// If Goal Freq detected, post event; Otherwise restart timer
			if(Front_MeasuredIRPeriodCode == GoalCode){
				// Post GoalAligned to Robot Top SM
				Event2Post.EventType = GoalAligned;
				Event2Post.EventParam = Front_MeasuredIRPeriodCode;
				PostRobotTopSM(Event2Post);
		// Else 
      }else{
      			// Kick off IRAligning_TIMER
				ES_Timer_InitTimer(IRAligning_TIMER,IRAligning_TIME);
			}				
		}
	}	
	// Return ReturnEvent
	return ReturnEvent;
}

/***************************************************************************

 Function
     DuringSettingBallSpeed

 Parameters
     ES_Event Event

 Returns
     ES_Event Return Event, event to return to the rest of the state machine

 Description
     During function for the SettingBallSpeed
 Notes

 Author
     J. Edward Carryer, 2/11/05, 10:38AM
****************************************************************************/
static ES_Event DuringSettingBallSpeed( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or consumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||(Event.EventType == ES_ENTRY_HISTORY) )
		{
			// turn on fly wheel 
			SetFlyDuty(FlyWheelDuty);	
			
			// Start timer to allow the flywheel to reach desired speed
			ES_Timer_InitTimer(FlyWheel_TIMER,FlyWheel_TIME);
		}
	// Else If Event is ES_EXIT
    else if ( Event.EventType == ES_EXIT )
    {
    	// Do nothing		
			
    }
		//Else do during function
		else 
		{
			// If Event is ES_TIMEOUT and param is FlyWheel_TIMER
			if (Event.EventType == ES_TIMEOUT && (Event.EventParam == FlyWheel_TIMER))
			{
					//start servo motor 
					SetServoDuty(SeparatorDuty);
				
					// start timer to turn off servo
					ES_Timer_InitTimer(Servo_TIMER,SeparatorONTime);
				
			// Else If Event is ES_TIMEOUT and param is Servo_TIMER	
			} else if (Event.EventType == ES_TIMEOUT && (Event.EventParam == Servo_TIMER)) 
			{
				// stop servo motor
				SetServoDuty(0);
			}
		}
		
    // Return ReturnEvent
    return(ReturnEvent);
}


/***************************************************************************
DuringWaiting4ShotComplete
 ***************************************************************************/
static ES_Event DuringWaiting4ShotComplete( ES_Event Event)  
{
    ES_Event ReturnEvent = Event; // assume no re-mapping or comsumption
		ES_Event Event2Post;

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) || (Event.EventType == ES_ENTRY_HISTORY) )
    {
				// Start to rotate the seperator very slowly
				SetServoDuty(SeparatorDuty);
			
			  // start timer to load balls and shoot
				ES_Timer_InitTimer(IRAligning_TIMER, Wait4ShotTime);				
    }
    else if ( Event.EventType == ES_EXIT )
    {    
				// Turn off both the pushing flywheel and the separation servo
				SetServoDuty(0);
				SetFlyDuty(0);	
    }
		else //DURING - Scored, Missed, or No Balls?
    {  
			// Post Event to indicate shooting complete when timer expires
			if (Event.EventType == ES_TIMEOUT && (Event.EventParam == IRAligning_TIMER))
			{
					Event2Post.EventType = ShotComplete;
					PostRobotTopSM(Event2Post);
			}
			
			// If shot is done				
			if((Event.EventType == ES_TIMEOUT) && (Event.EventParam == Waiting4Shot_TIMER))
			{
					// Substract 1 from ball count independetly of whether we scored or not. 			
					BallCount = BallCount - 1;
					
					if (BallCount == 0) // go to RELOADING
					{
						Event2Post.EventType = NO_BALLS;
						PostRobotTopSM(Event2Post);
					}
					else // Go back to the robot top state machine and from there post SCORED or MISSED
					{
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

/****************************************************************************************
	AlignWithGoal
	- Find an active goal and return true if you found it
*******************************************************************************************/
static bool DetectAGoal()
{
		// If firstIRBeaconAlignment is true
		if ( firstIRBeaconAlignment == 1 )
		{
			// reset flag
			firstIRBeaconAlignment = 0;  
			
			// If aligned with 1450Hz beacon - Ready to move on to SHOOTING
			if ( Back_MeasuredIRPeriodCode ==  code690us)		// We compare the value measured with the ISR with the desired code
				// Return true
				return true;
			// Else
			else
				// Return false
				return false;
		}
		
		// Else If firstIRBeaconAlignment is false
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
			// Return False
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
	// Return score
	return score;
}

/****************************************************************************************
	GetMyScoreFromStatusResponse
	- Find an active goal and return true if you found it
*******************************************************************************************/
uint16_t GetScoreFromShootingSM()
{
	// Return ScoreBeforeShooting
	return ScoreBeforeShooting;
}

/****************************************************************************
GetBallCount
- The ball count needs to be updated when we shoot and when we reload
****************************************************************************/

uint8_t GetBallCount()
{
	// Return BallCount
	return BallCount;
}
