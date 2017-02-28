/****************************************************************************
 
  Header file for Magnetic Module
 ****************************************************************************/

#ifndef RLCSensing_H
#define RLCSensing_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void InitRLCSensor( void );
void ReadRLCSensor(int RLCReading[2]);

#endif 
