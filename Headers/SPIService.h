/****************************************************************************
  Header file for SPIService
  based on the Gen 2 Events and Services Framework
 ****************************************************************************/

#ifndef SPIService_H
#define SPIService_H

#include "ES_Configure.h"
#include "ES_Types.h"
#include "ES_Events.h" 


// typedefs for the states
// State definitions for use with the query function
typedef enum {WAITING2TRANSMIT,WAITING4TIMEOUT} SPIState_t ;

// Public Function Prototypes
bool InitSPIService ( uint8_t );
ES_Event RunSPIService( ES_Event );
bool PostSPIService( ES_Event );
uint16_t getCommand(void);


#endif /* SPIService_H */

