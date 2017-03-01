/****************************************************************************
 
  Header file for IRBeaconModule Module
 ****************************************************************************/

#ifndef IRBeaconModule_H
#define IRBeaconModule_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void InitInputCaptureForIRDetection( void );
void EnableIRInterrupt(void);
void InputCaptureForIRDetectionResponse( void );
uint8_t GetIRCode( void );

#endif 

