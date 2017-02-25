/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1 

 Description
   This is the sample for writing event checkers along with the event 
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.
   
 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function 
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header & 
// actual functionsdefinition
#include "EventCheckers.h"

#include "SPIService.h"

// This is the event checking function sample. It is not intended to be 
// included in the module. It is only here as a sample to guide you in writing
// your own event checkers

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

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so, 
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker 
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if ( IsNewKeyReady() ) // new key waiting?
  {
		// SEE ME
    ES_Event ThisEvent;
    ThisEvent.EventType = ES_NEW_KEY;
    ThisEvent.EventParam = GetNewKey();
		ES_Event CommandEvent;
    // test distribution list functionality by sending the 'L' key out via
    // a distribution list.
    if ( ThisEvent.EventParam == 'L' ){
			//PostSPIService(ThisEvent);
			CommandEvent.EventParam = CCW_90;
			PostActionService(CommandEvent);
    }
		else if( ThisEvent.EventParam == 'l' ){
			CommandEvent.EventParam = CCW_45;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'R' ){
			CommandEvent.EventParam = CW_90;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'r' ){
			CommandEvent.EventParam = CW_45;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 's' ){
			CommandEvent.EventParam = STOP;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'F' ){
			CommandEvent.EventParam = FORWARD_FULL_SPEED;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'f' ){
			CommandEvent.EventParam = FORWARD_HALF_SPEED;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'B' ){
			CommandEvent.EventParam = REVERSE_FULL_SPEED;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'b' ){
			CommandEvent.EventParam = REVERSE_HALF_SPEED;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'a' ){
			CommandEvent.EventParam = ALIGN_BEACON;
			PostActionService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'd' ){
			CommandEvent.EventParam = DRIVE2TAPE;
			PostActionService(CommandEvent);
		}	
		else if( ThisEvent.EventParam == 'G' ){
			CommandEvent.EventParam = ROBOT_QUERY;
			PostSPIService(CommandEvent);
		}	
		else if( ThisEvent.EventParam == 'g' ){
			CommandEvent.EventParam = ROBOT_FREQ_RESPONSE;
			PostSPIService(CommandEvent);
		}
		else if( ThisEvent.EventParam == 'V' ){
			CommandEvent.EventParam = ROBOT_STATUS;
			PostSPIService(CommandEvent);
		}
		else{   // otherwise post to Service 0 for processing
   
    }
    return true;
  }
  return false;
}
