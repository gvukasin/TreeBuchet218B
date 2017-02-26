/****************************************************************************
 Template header file for Hierarchical Sate Machines AKA StateCharts
 02/08/12 adjsutments for use with the Events and Services Framework Gen2
 3/17/09  Fixed prototpyes to use Event_t
 ****************************************************************************/

#ifndef ReloadingSubSM_H
#define ReloadingSubSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h" 

// typedefs for the states
// State definitions for use with the query function
typedef enum { REQUESTING_BALL, WAITING4BALL } ReloadingState_t ;

// Public Function Prototypes

ES_Event RunReloadingSM( ES_Event CurrentEvent );
void StartReloadingSM ( ES_Event CurrentEvent );
ReloadingState_t QueryReloadingSM ( void );

#endif /*ReloadingSubSM_H */

