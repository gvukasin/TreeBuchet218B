/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts
 02/08/12 adjsutments for use with the Events and Services Framework Gen2
 3/17/09  Fixed prototpyes to use Event_t
 ****************************************************************************/

#ifndef ShootingSubSM_H
#define ShootingSubSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h" 

// typedefs for the states
// State definitions for use with the query function
typedef enum { LOOKING4GOAL, SETTING_BALL_SPEED, WATING4SHOT_COMPLETE } ShootingState_t ;

// Public Function Prototypes

ES_Event RunShootingSM( ES_Event CurrentEvent );
void StartShootingSM ( ES_Event CurrentEvent );
ShootingState_t QueryShootingSM ( void );
uint8_t GetBallCount( void );
uint8_t GetMyScore( void );

#endif /*ShootingSubSM_H */
