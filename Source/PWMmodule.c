/****************************************************************************
 Module
   PWMmodule.c

 Description
	 Generate PWM waveforms
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

//#define TEST 

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "PWMmodule.h"

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
#define TEST_MODE

#define PeriodInUS 500
#define MotorPeriodInUS 2000 // to get 500Hz
#define IRPeriodInUS 40000
#define PWMTicksPerUS 40000/(1000*32) //System clock (40MHz) / 32
#define PWMTickPerMS 40000/32 //System clock divided by 32
#define DutyCycle25 25
#define BitsPerNibble 4

#define PWM0_GenA_Normal (PWM_0_GENA_ACTCMPAU_ONE | PWM_0_GENA_ACTCMPAD_ZERO )
#define PWM0_GenB_Normal (PWM_0_GENB_ACTCMPBU_ONE | PWM_0_GENB_ACTCMPBD_ZERO )
#define PWM1_GenA_Normal (PWM_1_GENA_ACTCMPAU_ONE | PWM_1_GENA_ACTCMPAD_ZERO )
#define PWM1_GenB_Normal (PWM_1_GENB_ACTCMPBU_ONE | PWM_1_GENB_ACTCMPBD_ZERO )
#define PWM2_GenA_Normal (PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ZERO )
#define PWM2_GenB_Normal (PWM_2_GENB_ACTCMPBU_ONE | PWM_2_GENB_ACTCMPBD_ZERO )


#define L_CCW_MOTOR_PIN BIT4HI // alt function 4
#define L_CW_MOTOR_PIN BIT5HI
#define R_CW_MOTOR_PIN BIT6HI
#define R_CCW_MOTOR_PIN BIT7HI
#define ServoMotorPin BIT4HI //PE4, alt function 5
#define FlyWheelMotorPin BIT5HI //PE5
#define IRPin BIT1HI //PF1
#define ExtraPWMPin BIT0HI //PF0
#define FlyWheelEnablePin BIT7HI //PD7

#define FORWARD 1
#define BACKWARD 0

#define LEFT 1
#define RIGHT 0

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service*/
static void Set100DC(uint8_t SelectedPin);
static void Set0DC(uint8_t SelectedPin);
static void RestoreDC(uint8_t SelectedPin);
static void EnableFlyWheel( bool );

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
//static uint16_t PeriodInUSVal = PeriodInUS;

/*------------------------------ Module Code ------------------------------*/

/***************************************************************************
  SetPWMDutyCycle
		Set the duty cycle of one of the wheels corresponding to inputs: DutyCycle, direction (0=cw,1=ccw), and wheelSide (0=right,1=left)
 ***************************************************************************/
void InitializePWM(void)
{
	// Enable the clock to the PWM Module (PWM0)
	HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
	
	// Enable the clock to Port B
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
	
	// Select the PWM clock as System Clock/32
	HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) | (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
	
	// Make sure that the PWM module clock has gotten going
	while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0);
	
	// Disable the PWM while initializing
	HWREG(PWM0_BASE+PWM_O_0_CTL) = 0;
	HWREG(PWM0_BASE+PWM_O_1_CTL) = 0;
	
	// program generators to go to 1 at rising compare A/B, 0 on falling compare A/B
	HWREG(PWM0_BASE + PWM_O_0_GENA) = PWM0_GenA_Normal; 
	HWREG(PWM0_BASE + PWM_O_0_GENB) = PWM0_GenB_Normal;
	HWREG(PWM0_BASE + PWM_O_1_GENA) = PWM1_GenA_Normal;
	HWREG(PWM0_BASE + PWM_O_1_GENB) = PWM1_GenB_Normal;
	
	
	// Set the PWM period
	HWREG(PWM0_BASE + PWM_O_0_LOAD) = ((PeriodInUS * PWMTicksPerUS))>>1;
	HWREG(PWM0_BASE + PWM_O_1_LOAD) = ((PeriodInUS * PWMTicksPerUS))>>1;
	
	// Set the initial Duty cycle on A and B to 0 
	HWREG(PWM0_BASE + PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ZERO;
	HWREG(PWM0_BASE + PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ZERO;
	HWREG(PWM0_BASE + PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ZERO;
	HWREG(PWM0_BASE + PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ZERO;
	
	// Enable the PWM outputs 0, 1, 2, 3
	HWREG(PWM0_BASE + PWM_O_ENABLE) |= (PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN);

	// Configure the Port B pins 4,5,6,7 to be PWM outputs -- alternate function
	HWREG(GPIO_PORTB_BASE + GPIO_O_AFSEL) |= (L_CCW_MOTOR_PIN | L_CW_MOTOR_PIN | R_CCW_MOTOR_PIN | R_CW_MOTOR_PIN);
	HWREG(GPIO_PORTB_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) & 0x0000ffff) + (4<<(7*BitsPerNibble)) + (4<<(6*BitsPerNibble))
																																													 + (4<<(5*BitsPerNibble)) + (4<<(4*BitsPerNibble));
	
	// Enable pins 4,5,6,7 on Port B for digital I/O
	HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (L_CCW_MOTOR_PIN | L_CW_MOTOR_PIN | R_CCW_MOTOR_PIN | R_CW_MOTOR_PIN);
	
	// make pins 4,5,6,7 on Port B into outputs
	HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (L_CCW_MOTOR_PIN | L_CW_MOTOR_PIN | R_CCW_MOTOR_PIN | R_CW_MOTOR_PIN);
	
	// set the up/down count mode, enable the PWM generator and make
	// both generator updates locally synchronized to zero count
	HWREG(PWM0_BASE+ PWM_O_0_CTL) = (PWM_0_CTL_MODE | PWM_0_CTL_ENABLE | PWM_0_CTL_GENAUPD_LS | PWM_0_CTL_GENBUPD_LS);
	HWREG(PWM0_BASE+ PWM_O_1_CTL) = (PWM_1_CTL_MODE | PWM_1_CTL_ENABLE | PWM_1_CTL_GENAUPD_LS | PWM_1_CTL_GENBUPD_LS);
}

void InitializeAltPWM(void){
	// Enable the clock to the PWM Module (PWM1)
	HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R1;
	
	// Enable the clock to Port E and F
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5;
	
	// Select the PWM clock as System Clock/32
	HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) | (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
	
	// Make sure that the PWM module clock has gotten going
	while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R1) != SYSCTL_PRPWM_R1);

	// Disable the PWM while initializing
	HWREG(PWM1_BASE+PWM_O_1_CTL) = 0;
	HWREG(PWM1_BASE+PWM_O_2_CTL) = 0;
	
	// program generators to go to 1 at rising compare A/B, 0 on falling compare A/B
	HWREG(PWM1_BASE + PWM_O_1_GENA) = PWM1_GenA_Normal; 
	HWREG(PWM1_BASE + PWM_O_1_GENB) = PWM1_GenB_Normal;
	HWREG(PWM1_BASE + PWM_O_2_GENA) = PWM2_GenA_Normal;
	HWREG(PWM1_BASE + PWM_O_2_GENB) = PWM2_GenB_Normal;
	
	// Set the PWM period
	HWREG(PWM1_BASE + PWM_O_1_LOAD) = ((MotorPeriodInUS * PWMTicksPerUS))>>1;
	HWREG(PWM1_BASE + PWM_O_2_LOAD) = ((IRPeriodInUS * PWMTicksPerUS))>>1;
	
	// Set the initial Duty cycle on A and B to 0 
	HWREG(PWM1_BASE + PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ZERO;
	HWREG(PWM1_BASE + PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ZERO;
	HWREG(PWM1_BASE + PWM_O_2_GENA) = PWM_2_GENA_ACTZERO_ZERO;
	HWREG(PWM1_BASE + PWM_O_2_GENB) = PWM_2_GENB_ACTZERO_ZERO;
	
	// Enable the PWM outputs  2, 3, 4, 5
	HWREG(PWM1_BASE + PWM_O_ENABLE) |= (PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN);

	// Configure the Port E pins 4,5 to be PWM outputs -- alternate function 5
	HWREG(GPIO_PORTE_BASE + GPIO_O_AFSEL) |= (ServoMotorPin|FlyWheelMotorPin);
	HWREG(GPIO_PORTE_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTE_BASE+GPIO_O_PCTL) & 0xff00ffff) + (5<<(4*BitsPerNibble)) + (5<<(5*BitsPerNibble));
	
	// Configure the Port F pins 0,1 to be PWM outputs -- alternate function 5
	HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= (IRPin | ExtraPWMPin);
	HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTF_BASE+GPIO_O_PCTL) & 0xffffff00) + (5) + (5<<(BitsPerNibble));
	
	// Enable pins 4,5 on Port E for digital I/O
	HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= (ServoMotorPin|FlyWheelMotorPin);
	
	// Enable pins 0,1 on Port F for digital I/O
	HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= (IRPin | ExtraPWMPin);
	
	// make pins 4,5 on Port E into outputs
	HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) |= (ServoMotorPin|FlyWheelMotorPin);
	
		// make pins 0,1 on Port F into outputs
	HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) |= (IRPin | ExtraPWMPin);
	
	// set the up/down count mode, enable the PWM generator and make
	// both generator updates locally synchronized to zero count
	HWREG(PWM1_BASE+ PWM_O_1_CTL) = (PWM_1_CTL_MODE | PWM_1_CTL_ENABLE | PWM_1_CTL_GENAUPD_LS | PWM_1_CTL_GENBUPD_LS);
	HWREG(PWM1_BASE+ PWM_O_2_CTL) = (PWM_2_CTL_MODE | PWM_2_CTL_ENABLE | PWM_2_CTL_GENAUPD_LS | PWM_2_CTL_GENBUPD_LS);
	
	// initialize Fly Wheel Enable Pin
		//Initialize Port D
		HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
		
		// Wait for Port to be ready
		while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3) != SYSCTL_PRGPIO_R3);
		
		//Enable pin 2 on Port D for digital I/O
		HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= FlyWheelEnablePin;
		
		//make pin 2 on Port D into an output
		HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) |= FlyWheelEnablePin;
}

/***************************************************************************
  SetPWMDutyCycle
		Set the duty cycle of one of the driving wheel motors corresponding to inputs: DutyCycle, direction (0=backwards,1=forward), and wheelSide (0=right,1=left)
 ***************************************************************************/

void SetPWMDutyCycle(uint8_t DutyCycle, bool direction, bool wheelSide)
{
	// If wheelSide is left and the direction is forward
	if (wheelSide == LEFT && direction == FORWARD)
	{
		// If the DutyCycle is 0
		if (DutyCycle == 0)
		{
			// Set output of both left pins to 0
			Set0DC(L_CCW_MOTOR_PIN); //PB4 to 0
			Set0DC(L_CW_MOTOR_PIN);  //PB5 to 0
		}
		
		//Else If the DutyCycle is 100
		else if (DutyCycle == 100)
		{
			// Set output of left, counterclockwise pin to 100 and left clockwise pin to 0
			Set100DC(L_CCW_MOTOR_PIN);//PB4 to 100
			Set0DC(L_CW_MOTOR_PIN);		//PB5 to 0
		}
		// Else 
		else
		{
			// Restore pwm to left, counterclockwise pin 
			RestoreDC(L_CCW_MOTOR_PIN);
			
			// Set left, counterclockwise pin to DutyCycle
			HWREG( PWM0_BASE + PWM_O_1_CMPA) = (HWREG( PWM0_BASE + PWM_O_1_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);
			
			// Set left, clockwise pin to 0
			Set0DC(L_CW_MOTOR_PIN);
		}
	}
	
	// Else If wheelSide is left and direction is backwards
	else if (wheelSide == LEFT && direction == BACKWARD)
	{
		// If the DutyCycle is 0
		if (DutyCycle == 0)
		{
			// Set the left, counterclockwise and clockwise pins to 0
			Set0DC(L_CCW_MOTOR_PIN); //PB4 to 0
			Set0DC(L_CW_MOTOR_PIN);  //PB5 to 0
		}
		// Else If the DutyCycle is 100
		else if (DutyCycle == 100)
		{
			// Set the left, counterclockwise pin to 0 and the left, clockwise pin to 100
			Set0DC(L_CCW_MOTOR_PIN); //PB4 to 0
			Set100DC(L_CW_MOTOR_PIN);//PB5 to 100
		}
		// Else 
		else
		{
			// Restore pwm to the left, clockwise pin 
			RestoreDC(L_CW_MOTOR_PIN);
			
			//Set the duty cycle of the left counterclockwise pin to 0
			Set0DC(L_CCW_MOTOR_PIN);
			
			//Set the duty cycle of the left clockwise pin to the DutyCycle
			HWREG( PWM0_BASE + PWM_O_1_CMPB) = (HWREG( PWM0_BASE + PWM_O_1_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);	
		}
	}
	
	// Else If wheelSide is right and direction is forward 
	else if (wheelSide == RIGHT && direction == FORWARD)
	{
		// If the DutyCycle is 0
		if (DutyCycle == 0)
		{
			// Set the right, clockwise and counterclockwise pins to 0
			Set0DC(R_CW_MOTOR_PIN); //PB6 to 0
			Set0DC(R_CCW_MOTOR_PIN);//PB7 to 0
		}
		// Else If the DutyCycle is 100
		else if (DutyCycle == 100)
		{
			// Set the right cw pin to 100 and the right ccw pin to 0
			Set100DC(R_CW_MOTOR_PIN);//PB6 to 100
			Set0DC(R_CCW_MOTOR_PIN); //PB7 to 0
		}
		// Else 
		else
		{
			// Restore pwm to the right cw pin 
			RestoreDC(R_CW_MOTOR_PIN);
			
			// Set the duty cycle of the right cw pin to DutyCycle 
			HWREG( PWM0_BASE + PWM_O_0_CMPA) = (HWREG( PWM0_BASE + PWM_O_0_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);
			
			// Set the duty cycle of the right ccw pin to 0
			Set0DC(R_CCW_MOTOR_PIN);
		}
	}
	
	else // right wheel selected, backward direction selected
	{
		// If the DutyCycle is 0
		if (DutyCycle == 0)
		{
			// Set the right cw and ccw pins to 0
			Set0DC(R_CW_MOTOR_PIN); //PB6 to 0
			Set0DC(R_CCW_MOTOR_PIN);//PB7 to 0
		}
		// Else If the DutyCycle is 100
		else if (DutyCycle == 100)
		{
			// Set the right cw pin to 0 and the right ccw pin to 100
			Set0DC(R_CW_MOTOR_PIN);   //PB6 to 0
			Set100DC(R_CCW_MOTOR_PIN);//PB7 to 100
		}
		// Else 
		else
		{
			// Restore pwm to the right ccw pin
			RestoreDC(R_CCW_MOTOR_PIN);
			
			// Set the output of the right cw pin to 0
			Set0DC(R_CW_MOTOR_PIN);
			
			// Set the duty cycle of the right ccw pin to DutyCycle 
			HWREG( PWM0_BASE + PWM_O_0_CMPB) = (HWREG( PWM0_BASE + PWM_O_0_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);
		}
	}
}
/***************************************************************************
  SetPWMPeriodUS
		Set the loads to the period/2 that is the input of this function (in us)
***************************************************************************/	
void SetPWMPeriodUS(uint16_t Period)
{
		HWREG( PWM0_BASE + PWM_O_0_LOAD) = ((Period * PWMTicksPerUS))>>1;
	  HWREG( PWM0_BASE + PWM_O_1_LOAD) = ((Period * PWMTicksPerUS))>>1;
}

/***************************************************************************
  EmitIR
		Enable/Disable PWM at 25% duty cycle to IR LED based on input boolean OnOrOff
***************************************************************************/	
void EmitIR( bool OnOrOff ){
	// If OnOrOff is true
	if(OnOrOff == 1){
		// Restore PWM to IR LED Pin
		HWREG( PWM1_BASE + PWM_O_2_GENB) = PWM2_GenB_Normal;
		 
		// Set duty cycle to 25%
		HWREG( PWM1_BASE + PWM_O_2_CMPB) = (HWREG(PWM1_BASE + PWM_O_2_LOAD)) - ((DutyCycle25*(IRPeriodInUS * PWMTicksPerUS)/100)>>1);
	
	// Else If OnOrOff is false
	} else {
		// Disable PWM to IR pin, set pwm to 0% duty cycle
		HWREG(PWM1_BASE + PWM_O_2_GENB) = PWM_2_GENB_ACTZERO_ZERO;
	}
}
/***************************************************************************
  SetServoDuty
		Set the duty cycle, which is the input of function (TheDooty), of the pin associated with the servo 
***************************************************************************/	
void SetServoDuty( uint16_t TheDooty ){
	// If TheDooty is 0% duty cycle
	if (TheDooty == 0)
		{
			// Set duty cycle to 0V or 0% duty cycle
			HWREG(PWM1_BASE + PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ZERO;

		}
		// Else If TheDooty is 100% duty cycle
		else if (TheDooty == 100)
		{
			// Set output of Servo Pin to 3.3V (duty cycle to 100)
			HWREG( PWM1_BASE+PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ONE;
		}
		// Else If TheDooty is between 0% and 100% duty cycle
		else if ((TheDooty > 0) && (TheDooty < 100))
		{
			// Restore PWM function to the Servo Pin
			HWREG( PWM1_BASE + PWM_O_1_GENA) = PWM1_GenA_Normal;
		 
			// Set duty cycle to TheDooty
			HWREG( PWM1_BASE + PWM_O_1_CMPA) = (HWREG(PWM1_BASE + PWM_O_1_LOAD)) - ((TheDooty*(MotorPeriodInUS * PWMTicksPerUS)/100)>>1);
		}
	
}
/***************************************************************************
  SetFlyWheel
		Set the duty cycle, which is the input of function (DatDooty), of the pin associated with the flywheel 
 ***************************************************************************/
void SetFlyDuty( uint16_t DatDooty ){
	// If DatDooty is 0% duty cycle
	if (DatDooty == 0)
		{
			// disable H-bridge
			EnableFlyWheel(0);
			
			// set duty cycle to 0
			HWREG(PWM1_BASE + PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ZERO;

		}
		// Else If DatDooty 100% duty cycle
		else if (DatDooty == 100)
		{
			// enable H-bridge
			EnableFlyWheel(1);
			
			// set duty cycle to 100
			HWREG( PWM1_BASE + PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ONE;
		}
		// Else If DatDooty is between 0% and 100% duty cycle
		else if ((DatDooty > 0) && (DatDooty < 100))
		{
			// enable H-bridge
			EnableFlyWheel(1);
			
			// restore pwm 
			HWREG( PWM1_BASE + PWM_O_1_GENB) = PWM1_GenB_Normal;
		 
			// set duty cycle to DatDooty
			HWREG( PWM1_BASE + PWM_O_1_CMPB) = (HWREG(PWM1_BASE + PWM_O_1_LOAD)) - ((DatDooty*(MotorPeriodInUS * PWMTicksPerUS)/100)>>1);
		}
	
}


/***************************************************************************
 private functions
 ***************************************************************************/
/***************************************************************************
  EnableFlyWheel
		Enable/Disable the HBridge attached to the flywheel 
 ***************************************************************************/

static void EnableFlyWheel( bool OnOrOff ){
	// If input boolean is true
	if(OnOrOff){
		// enable H-bridge by setting enable pin Hi
		HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) |= FlyWheelEnablePin;
	
		// Else If input boolean is false
	} else {
		// disable H-bridge by setting enable pin Lo
		HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= (~FlyWheelEnablePin);
		
	}
}

/***************************************************************************
  Set100DC
		Program 100% duty cycle (3.3V output) on selected pin, input of function
 ***************************************************************************/
static void Set100DC(uint8_t SelectedPin){
	
	// If the SelectedPin is the left, counterclockwise motor pin
	if (SelectedPin == L_CCW_MOTOR_PIN)
	{
		// PB4 to 100%
		HWREG( PWM0_BASE+PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ONE;
	}
	
	// Else If the SelectedPin is the left, clockwise motor pin
	else if (SelectedPin == L_CW_MOTOR_PIN)
	{
		// PB5 to 100%
		HWREG( PWM0_BASE+PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ONE;
	}
	
	// Else If the SelectedPin is the right, clockwise motor pin
	else if (SelectedPin == R_CW_MOTOR_PIN)
	{
		// PB6 to 100%
		HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ONE;
	}
	
	else // Else If the SelectedPin is the right, counterclockwise motor pin
	{
		// PB7 to 100%
		HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ONE;
	}
}

/***************************************************************************
Set0DC
	Set the output on the respective pin, input of function to zero
 ***************************************************************************/

static void Set0DC(uint8_t SelectedPin){
	
	// If the SelectedPin is the left, counterclockwise motor pin
	if (SelectedPin == L_CCW_MOTOR_PIN)
	{
		// PB4 to 0%
		HWREG( PWM0_BASE+PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ZERO;
	}
	
	// Else if the SelectedPin is the left, clockwise motor pin
	else if (SelectedPin == L_CW_MOTOR_PIN)
	{
		// PB5 to 0%
		HWREG( PWM0_BASE+PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ZERO;
	}
	
	// Else If the SelectedPin is the right, clockwise motor pin
	else if (SelectedPin == R_CW_MOTOR_PIN)
	{
		// PB6 to 0%
		HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ZERO;
	}
	
	else // Else If the SelectedPin in the right, counterclockwise motor pin
	{
		// PB7 to 0%
		HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ZERO;
	}
}

/***************************************************************************
  RestoreDC
		Reprogram generators to go to 1 at rising compare A/B, 0 on falling compare A/B
 ***************************************************************************/
static void RestoreDC(uint8_t SelectedPin){
		
	// If the SelectedPin is the left, counterclockwise motor pin
	if (SelectedPin == L_CCW_MOTOR_PIN)
	{
		// Restore PB4
		HWREG( PWM0_BASE+PWM_O_1_GENA) = PWM1_GenA_Normal;
	}
	
	// Else If the SelectedPin is the left, clockwise motor pin
	else if (SelectedPin == L_CW_MOTOR_PIN)
	{
		// Restore PB5
		HWREG( PWM0_BASE+PWM_O_1_GENB) = PWM1_GenB_Normal;
	}
	
	// Else If the SelectedPin is the right, clockwise motor pin
	else if (SelectedPin == R_CW_MOTOR_PIN)
	{
		// Restore PB6
		HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM0_GenA_Normal;
	}
	
	else // Else If the SelectedPin is the right, counterclockwise motor pin
	{
		// Restore PB7
		HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM0_GenB_Normal;
	}
}

