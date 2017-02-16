/****************************************************************************
 Header file for MotorActionsModule
   MotorActionsModule.c
	 
 Elena Galbally
*****************************************************************************/

#ifndef MotorActionsModule_H
#define MotorActionsModule_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h" 

// Public Function Prototypes
void start2rotate(bool rotationDirection);
void rotate2beacon(void);
void drive(uint8_t DutyCycle, bool direction);
void stop(void);

#endif /* MotorActionsModule_H */
