/****************************************************************************
 Header file for LEDModule
   LEDModule.c
	 
 Team 16
*****************************************************************************/

#ifndef LEDmodule_H
#define LEDmodule_H

//Includes
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void TurnOnYellowLEDs(void);
void TurnOnBlueLEDs(void);
void TurnOnTeamColorLEDs(bool TeamColor);

#endif /* LEDmodule_H */
