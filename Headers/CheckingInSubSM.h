/****************************************************************************

 ****************************************************************************/

#ifndef CheckingInSubSM_H
#define CheckingInSubSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h" 

// typedefs for the states
// State definitions for use with the query function
typedef enum { FirstReportDone, Waiting4FirstResponseReady, SecondReportDone } CheckingInState_t ;

// Public Function Prototypes

ES_Event RunCheckingInSM( ES_Event CurrentEvent );
void StartCheckingInSM ( ES_Event CurrentEvent );

#endif /*ShootingSubSM_H */