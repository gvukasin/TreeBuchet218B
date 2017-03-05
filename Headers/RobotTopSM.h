/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts

 ****************************************************************************/

#ifndef TopHSMTemplate_H
#define TopHSMTemplate_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h" 

// State definitions for use with the query function
typedef enum { WAITING2START, DRIVING2STAGING, CHECKING_IN, SHOOTING, DRIVING2RELOAD, RELOADING, ENDING_STRATEGY, STOP} RobotState_t ;

// Public Function Prototypes

bool InitRobotTopSM ( uint8_t Priority );
bool PostRobotTopSM( ES_Event ThisEvent );
ES_Event RunRobotTopSM( ES_Event CurrentEvent );
void StartRobotTopSM ( ES_Event CurrentEvent );
bool GetTeamColor(void);

#endif /*TopHSMTemplate_H */
