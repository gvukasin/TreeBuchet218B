/****************************************************************************
 
  Header file for Magnetic Module
 ****************************************************************************/

#ifndef MagneticModule_H
#define MagneticModule_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void InitRLCSensor( void );
int CheckWirePosition(void);

#endif 
