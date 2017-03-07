/****************************************************************************
 
  Header file for IRBeaconModule Module
 ****************************************************************************/

#ifndef IRBeaconModule_H
#define IRBeaconModule_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void InitInputCaptureForFrontIRDetection( void );
void InitInputCaptureForBackIRDetection( void );
void EnableFrontIRInterrupt( void );
void EnableBackIRInterrupt( void );
void InputCaptureForFrontIRDetection( void );
void InputCaptureForBackIRDetection( void );
uint8_t Front_GetIRCodeArray( void );
uint8_t Back_GetIRCodeArray( void );

#endif 
