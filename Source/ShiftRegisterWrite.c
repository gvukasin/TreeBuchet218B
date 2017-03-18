/****************************************************************************
 Module
   ShiftRegisterWrite.c

 Revision
   1.0.1

 Description
   This module acts as the low level interface to a write only shift register.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/11/15 19:55 jec     first pass
 10/22/15 20:36 Elena   second pass
 
****************************************************************************/
// the common headers for C99 types 
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "BITDEFS.H"

// readability defines
#define DATA GPIO_PIN_0

#define SCLK GPIO_PIN_1
#define SCLK_HI BIT1HI
#define SCLK_LO BIT1LO

#define RCLK GPIO_PIN_3
#define RCLK_LO BIT3LO
#define RCLK_HI BIT3HI

#define GET_MSB_IN_LSB(x) ((x & 0x80)>>7)
#define ALL_BITS (0xff<<2)

// an image of the last 8 bits written to the shift register
static uint8_t LocalRegisterImage=0;

/********************** SR_Init ********************************************
 Function
   SR_Init

 Parameters
   nothing

 Returns
   nothing

 Description
   Initializes the shift register
   
 Author
   Elena Galbally
****************************************************************************/
void SR_Init(void)
{
  // set up port D by enabling the peripheral clock
	HWREG(SYSCTL_RCGCGPIO) |= BIT3HI;
	while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3) != SYSCTL_PRGPIO_R3);
	
	// Set the direction of PD0, PD1 & PD2 to output and digital pins
	HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) |= (DATA | SCLK | RCLK);	
	HWREG(GPIO_PORTD_BASE+GPIO_O_DIR) |= (DATA | SCLK | RCLK);
  
  // Start with the data & sclk lines low and the RCLK line high
	HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= SCLK_LO;
	HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT0LO;
	HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) |= RCLK_HI;	
}

/********************* SR_GetCurrentRegister ***************************************
 Function
   SR_Write

 Parameters
   none

 Returns
   uint8_t LocalRegisterImage; current register value

 Description
   Returns the current shift register value
   
 Author
   Elena Galbally
****************************************************************************/
uint8_t SR_GetCurrentRegister(void)
{
	// Return the value for LocalRegisterImage
  return LocalRegisterImage;
}

/************************ SR_Write ****************************************
 Function
   SR_Write

 Parameters
   uint8_t NewValue

 Returns
   nothing

 Description
   Writes a new 8 bit value to the shift register
   
 Author
   Elena Galbally
****************************************************************************/
void SR_Write(uint8_t NewValue)
{
// Create a local variable to count bits
  uint8_t BitCounter;
  // save a local copy of NewValue
  LocalRegisterImage = NewValue; 
	
	// For 0 through 8 
	for(BitCounter = 0; BitCounter < 8; BitCounter++)
	{
		// Isolate the MSB of NewValue, put it into the LSB position and output
		if((NewValue & BIT7HI) == BIT7HI) 
			HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) |= DATA; 
		else
			HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= (~DATA); 
		
		// Pulse SCLK
		HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) |= SCLK_HI; 
		HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= SCLK_LO; 
		
		// Shift data
		NewValue = (NewValue << 1);
	}
		
	// Pulse the register clock to latch the new data
	HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= RCLK_LO;
	HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) |= RCLK_HI;  
}
