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

#define TEST 

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
	
	// SEE ME: this is already set in the other initialization function so I do not want to reset it
	// Select the PWM clock as System Clock/32
	HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) | (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
	
	// Make sure that the PWM module clock has gotten going
	while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R1) != SYSCTL_PRPWM_R1);
	
	// SEE ME: is this the correct for using M0PWM2,3,4?
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
	printf("\r\n SEE ME6");
	
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
		printf("\r\n got thru alt pwm init");
}

void SetPWMDutyCycle(uint8_t DutyCycle, bool direction, bool wheelSide)
{
	if (wheelSide == LEFT && direction == FORWARD)
	{
		
		if (DutyCycle == 0)
		{
			Set0DC(L_CCW_MOTOR_PIN); //PB4 to 0
			Set0DC(L_CW_MOTOR_PIN);  //PB5 to 0
		}
		else if (DutyCycle == 100)
		{
			Set100DC(L_CCW_MOTOR_PIN);//PB4 to 100
			Set0DC(L_CW_MOTOR_PIN);		//PB5 to 0
		}
		else
		{
			RestoreDC(L_CCW_MOTOR_PIN);
			
			// PB4 set to DutyCycle
			HWREG( PWM0_BASE + PWM_O_1_CMPA) = (HWREG( PWM0_BASE + PWM_O_1_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);
			
			// PB5 set to 0
			Set0DC(L_CW_MOTOR_PIN);
		}
	}
	
	else if (wheelSide == LEFT && direction == BACKWARD)
	{
		if (DutyCycle == 0)
		{
			Set0DC(L_CCW_MOTOR_PIN); //PB4 to 0
			Set0DC(L_CW_MOTOR_PIN);  //PB5 to 0
		}
		else if (DutyCycle == 100)
		{
			Set0DC(L_CCW_MOTOR_PIN); //PB4 to 0
			Set100DC(L_CW_MOTOR_PIN);//PB5 to 100
		}
		else
		{
			RestoreDC(L_CW_MOTOR_PIN);
			
			// PB4 set to 0
			Set0DC(L_CCW_MOTOR_PIN);
			
			// PB5 commands motor CW
			HWREG( PWM0_BASE + PWM_O_1_CMPB) = (HWREG( PWM0_BASE + PWM_O_1_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);	
		}
	}
	
	else if (wheelSide == RIGHT && direction == FORWARD)
	{
	
		if (DutyCycle == 0)
		{
			Set0DC(R_CW_MOTOR_PIN); //PB6 to 0
			Set0DC(R_CCW_MOTOR_PIN);//PB7 to 0
		}
		else if (DutyCycle == 100)
		{
			Set100DC(R_CW_MOTOR_PIN);//PB6 to 100
			Set0DC(R_CCW_MOTOR_PIN); //PB7 to 0
		}
		else
		{
			RestoreDC(R_CW_MOTOR_PIN);
			
			// PB6 commands motor CW
			HWREG( PWM0_BASE + PWM_O_0_CMPA) = (HWREG( PWM0_BASE + PWM_O_0_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);
			
			// PB7 set to 0
			Set0DC(R_CCW_MOTOR_PIN);
		}
	}
	
	else // right wheel selected, backward direction selected
	{
	
		if (DutyCycle == 0)
		{
			Set0DC(R_CW_MOTOR_PIN); //PB6 to 0
			Set0DC(R_CCW_MOTOR_PIN);//PB7 to 0
		}
		else if (DutyCycle == 100)
		{
			Set0DC(R_CW_MOTOR_PIN);   //PB6 to 0
			Set100DC(R_CCW_MOTOR_PIN);//PB7 to 100
		}
		else
		{
			RestoreDC(R_CCW_MOTOR_PIN);
			
			// PB6 set to 0
			Set0DC(R_CW_MOTOR_PIN);
			
			// PB7 commands motor CCW
			HWREG( PWM0_BASE + PWM_O_0_CMPB) = (HWREG( PWM0_BASE + PWM_O_0_LOAD)) - ((DutyCycle*(PeriodInUS * PWMTicksPerUS)/100)>>1);
		}
	}
}

void SetPWMPeriodUS(uint16_t Period)
{
		HWREG( PWM0_BASE + PWM_O_0_LOAD) = ((Period * PWMTicksPerUS))>>1;
	  HWREG( PWM0_BASE + PWM_O_1_LOAD) = ((Period * PWMTicksPerUS))>>1;
}

uint16_t GetPWMPeriodUS(void)
{
	return PeriodInUS;
}

void EmitIR( bool OnOrOff ){
	if(OnOrOff == 1){
		// turn on PWM to IR 
		// restore pwm 
		HWREG( PWM1_BASE + PWM_O_2_GENB) = PWM2_GenB_Normal;
		 
		// set duty cycle to 25%
		HWREG( PWM1_BASE + PWM_O_2_CMPB) = (HWREG(PWM1_BASE + PWM_O_2_LOAD)) - ((DutyCycle25*(IRPeriodInUS * PWMTicksPerUS)/100)>>1);
		
	} else {
		// disable PWM to IR pin
		HWREG(PWM1_BASE + PWM_O_2_GENB) = PWM_2_GENB_ACTZERO_ZERO;
	}
}

void SetServoDuty( uint16_t TheDooty ){
	
	if (TheDooty == 0)
		{
			// set duty cycle to 0
			HWREG(PWM1_BASE + PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ZERO;

		}
		else if (TheDooty == 100)
		{
			// set duty cycle to 100
			HWREG( PWM1_BASE+PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ONE;
		}
		else if ((TheDooty > 0) && (TheDooty < 100))
		{
			// restore pwm 
			HWREG( PWM1_BASE + PWM_O_1_GENA) = PWM1_GenA_Normal;
		 
			// set duty cycle to 25%
			HWREG( PWM1_BASE + PWM_O_1_CMPA) = (HWREG(PWM1_BASE + PWM_O_1_LOAD)) - ((TheDooty*(MotorPeriodInUS * PWMTicksPerUS)/100)>>1);
		}
	
}
void SetFlyDuty( uint16_t DatDooty ){
	
	if (DatDooty == 0)
		{
			// disable H-bridge
			EnableFlyWheel(0);
			
			// set duty cycle to 0
			HWREG(PWM1_BASE + PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ZERO;

		}
		else if (DatDooty == 100)
		{
			// enable H-bridge
			EnableFlyWheel(1);
			
			// set duty cycle to 100
			HWREG( PWM1_BASE + PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ONE;
		}
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
static void EnableFlyWheel( bool OnOrOff ){
	
	if(OnOrOff){
		// enable H-bridge by setting enable pin Hi
		HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) |= FlyWheelEnablePin;
		
	} else {
		// disable H-bridge by setting enable pin Lo
		HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= (~FlyWheelEnablePin);
		
	}
}

static void Set100DC(uint8_t SelectedPin){

	// Program 100% DC - set the action on Zero to set the output to one
	if (SelectedPin == L_CCW_MOTOR_PIN)
	{
		// PB4 to 100%
		HWREG( PWM0_BASE+PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ONE;
	}
	
	else if (SelectedPin == L_CW_MOTOR_PIN)
	{
		// PB5 to 100%
		HWREG( PWM0_BASE+PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ONE;
	}
	
	else if (SelectedPin == R_CW_MOTOR_PIN)
	{
		// PB6 to 100%
		HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ONE;
	}
	
	else // PB7 selected
	{
		// PB7 to 100%
		HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ONE;
	}
}

static void Set0DC(uint8_t SelectedPin){

	// Program 0% DC - set the action on Zero to set the output to zero
	if (SelectedPin == L_CCW_MOTOR_PIN)
	{
		// PB4 to 0%
		HWREG( PWM0_BASE+PWM_O_1_GENA) = PWM_1_GENA_ACTZERO_ZERO;
	}
	
	else if (SelectedPin == L_CW_MOTOR_PIN)
	{
		// PB5 to 0%
		HWREG( PWM0_BASE+PWM_O_1_GENB) = PWM_1_GENB_ACTZERO_ZERO;
	}
	
	else if (SelectedPin == R_CW_MOTOR_PIN)
	{
		// PB6 to 0%
		HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ZERO;
	}
	
	else // PB7 selected
	{
		// PB7 to 0%
		HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ZERO;
	}
}

static void RestoreDC(uint8_t SelectedPin){

	// Restore the previous DC - set the action back to the normal actions
	if (SelectedPin == L_CCW_MOTOR_PIN)
	{
		// Restore PB4
		HWREG( PWM0_BASE+PWM_O_1_GENA) = PWM1_GenA_Normal;
	}
	
	else if (SelectedPin == L_CW_MOTOR_PIN)
	{
		// Restore PB5
		HWREG( PWM0_BASE+PWM_O_1_GENB) = PWM1_GenB_Normal;
	}
	
	else if (SelectedPin == R_CW_MOTOR_PIN)
	{
		// Restore PB6
		HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM0_GenA_Normal;
	}
	
	else // PB7 selected
	{
		// Restore PB7
		HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM0_GenB_Normal;
	}
}

// test harness for first check-off
#ifdef TEST
#include "termio.h"
#define clrScrn() 	printf("\x1b[2J")
int main(void){
	
// Set the clock to run at 40MhZ using the PLL and 16MHz external crystal
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
			| SYSCTL_XTAL_16MHZ);
	TERMIO_Init();
	clrScrn();
	printf("\r\n pwm test module \r\n");

//	//initialize pwm
//	InitializePWM();
//	
//	printf("\r\n pwm initialized \r\n");
//	
//	Set100DC(R_CW_MOTOR_PIN);
//	Set0DC(R_CCW_MOTOR_PIN);
	InitializeAltPWM();
	//EmitIR( 0 );
	SetFlyDuty(80);
	//HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA + ALL_BITS)) &= (~FlyWheelEnablePin);
	SetServoDuty(60); 
	//EnableFlyWheel(0);
	
}
#endif
