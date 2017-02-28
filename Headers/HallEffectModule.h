/****************************************************************************
 
  Header file for Magnetic Module
 ****************************************************************************/

#ifndef HallEffectModule_H
#define HallEffectModule_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Public Function Prototypes
bool InitHallEffectModule ( uint8_t Priority );
void InitStagingAreaISR( void );
uint16_t GetStagingAreaCode( void );

#endif /* HallEffectModule_H */
