/****************************************************************************
 Module
   MotorActionsModule.c

 Description
	Stablish what the pins connected to the motor should do
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MotorActionsModule.h"
#include "PWMModule.h"

#include <stdio.h>
#include <termio.h>

// the common headers for C99 types 
#include <stdint.h>
#include <stdbool.h>

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_pwm.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "BITDEFS.H"
/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define CW 1
#define CCW 0

#define FORWARD 1
#define BACKWARD 0

#define LEFT 1
#define RIGHT 0

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static bool wheelSide;
static bool direction;

/*------------------------------ Module Code ------------------------------*/
/***************************************************************************
  start2rotate
		rotates the wheels so that the robot is spinning clockwise or counter clockwise at a specific duty cycle
 ***************************************************************************/
void start2rotate(bool rotationDirection, uint8_t DutyCycle)
{
	// If rotationDirection is clockwise
	if (rotationDirection == CW)
	{
		// pick the left wheel and rotate it forward to make robot spin CW
		wheelSide = LEFT;
		direction = FORWARD;
		SetPWMDutyCycle(DutyCycle, direction, wheelSide);
		
		// pick the right wheel and rotate it backward to make robot spin CW
		wheelSide = RIGHT;
		direction = BACKWARD;
		SetPWMDutyCycle(DutyCycle, direction, wheelSide);
	}
	// If rotationDirection is counterclockwise
	else 
	{
		// pick the left wheel and rotate it backward to make robot spin CCW
		wheelSide = LEFT;
		direction = BACKWARD;
		SetPWMDutyCycle(DutyCycle, direction, wheelSide);
		
		// pick the right wheel and rotate it forward to make robot spin CCW
		wheelSide = RIGHT;
		direction = FORWARD;
		SetPWMDutyCycle(DutyCycle, direction, wheelSide);		
	}
}

/***************************************************************************
  driveSeperate
		rotates the wheels so that the robot drives in direction, "direction," with separate duty cycles for left/right wheels
 ***************************************************************************/
void driveSeperate(uint8_t LeftDutyCycle, uint8_t RightDutyCycle, bool direction)
{
	// drive left motor at specified LeftDutyCycle and direction
	wheelSide = LEFT;
	SetPWMDutyCycle(LeftDutyCycle, direction, wheelSide);
	
	// drive right motor at specified RightDutyCycle and direction
	wheelSide = RIGHT;
	SetPWMDutyCycle(RightDutyCycle, direction, wheelSide);
}

/***************************************************************************
  stop
		stop the motors for both wheels
 ***************************************************************************/
void stop(void)
{
	uint8_t DutyCycle = 0; // to stop motor
	direction = FORWARD; 
	
	// stop the left motor
	wheelSide = LEFT;
	SetPWMDutyCycle(DutyCycle, direction, wheelSide);
	
	// stop the right motor
	wheelSide = RIGHT;
	SetPWMDutyCycle(DutyCycle, direction, wheelSide);	
}
