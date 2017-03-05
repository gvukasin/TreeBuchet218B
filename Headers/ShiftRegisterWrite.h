/***************************************************************************
 Header
   ShiftRegisterWrite.h

 Implementation
	 Elena Galbally
***************************************************************************/
#ifndef ShiftRegisterWrite_H //protects against multiple inclusions
#define ShiftRegisterWrite_H
#include <stdint.h>

void SR_Init(void);
uint8_t SR_GetCurrentRegister(void);
void SR_Write(uint8_t NewValue);

#endif
