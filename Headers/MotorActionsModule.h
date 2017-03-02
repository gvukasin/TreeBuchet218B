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
void start2rotate(bool rotationDirection, uint8_t DutyCycle);
void rotate2beacon(void);
void drive(uint8_t DutyCycle, bool direction);
void stop(void);
void driveSeperate(uint8_t LeftDutyCycle, uint8_t RightDutyCycle, bool direction);

#endif /* MotorActionsModule_H */
