/****************************************************************************
 Header file for PWMmodule
   PWMmodule.c
	 
 Elena Galbally
*****************************************************************************/

#ifndef PWMmodule_H
#define PWMmodule_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void InitializePWM(void);
void SetPWMDutyCycle(uint8_t DutyCycle, bool direction, bool wheelSide);
void SetPWMPeriodUS(uint16_t Period);
uint16_t GetPWMPeriodUS(void);

#endif /* PWMmodule_H */
