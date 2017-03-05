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
void EnableStagingAreaISR( void );
void StagingAreaISR( void );
// uint16_t GetStagingAreaCode( void );
uint8_t GetStagingAreaCodeSingle( uint16_t thePeriod );
uint8_t GetStagingAreaCodeArray(void);

#endif /* HallEffectModule_H */
