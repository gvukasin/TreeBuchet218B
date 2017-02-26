/****************************************************************************
 Header file for DrivingModule
   DrivingModule.c
	 
 Team 16
*****************************************************************************/

#ifndef DrivingModule_H
#define DrivingModule_H

//Includes
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void DriveOnWire(void);
void Drive2Wire(void);
void Drive2Reload(void);

#endif /* DrivingModule_H */
