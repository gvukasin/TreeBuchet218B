/****************************************************************************
 Header file for MotorService
   MotorService.c
	 
 Elena Galbally
*****************************************************************************/

#ifndef ActionService_H
#define ActionService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h" 

// Public Function Prototypes
bool InitializeActionService(uint8_t Priority);
bool PostActionService(ES_Event ThisEvent);
ES_Event RunActionService(ES_Event ThisEvent);	
void OneShotISR(void);
void InputCaptureForIRDetectionResponse(void);
void EnableFrontIRInterrupt(void);

#endif /* ActionService_H */
