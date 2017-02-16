/****************************************************************************
 
  Header file for Tape Module
 ****************************************************************************/

#ifndef TapeModule_H
#define TapeModule_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
void InitTapeInterrupt (void);
void TapeInterruptResponse(void);
uint32_t GetTapeSensedTime(void);
void EnableTapeInterrupt(void);

// Module Function Prototypes

#endif 

