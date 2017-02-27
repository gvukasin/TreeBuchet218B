  /****************************************************************************
 Module
   SPIService.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the 
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 11/02/13 17:21 jec      added exercise of the event deferral/recall module
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for the framework and this service
*/
//#define TEST
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"	// Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"
#include "inc/hw_nvic.h"
#include "BITDEFS.H"
#include "inc/hw_ssi.h"
#include "SPIService.h"

// to print comments to the terminal
#include <stdio.h>
#include "termio.h" 
#define clrScrn() 	printf("\x1b[2J")


/*----------------------------- Module Defines ----------------------------*/
// pin definitions
#define RXPIN BIT4HI
#define TXPIN BIT5HI
#define SPIClock BIT2HI
#define SlaveSelect BIT3HI

// SSI Module definition
#define SSIModule BIT0HI
#define SSI_NVIC_HI BIT7HI

// CPSR divisor for SSI clock rate (40)
#define CPSDVSR 0x28

// SCR divisor for SSI clock rate (99)
#define SCR 0x63

// SPI period (4ms b/c need a minimum of 2ms)
// these times assume a 1.000mS/tick timing
#define TicksPerSec 976
#define SPIPeriod (TicksPerSec/25)

// defining ALL_BITS
#define ALL_BITS (0xff<<2)

// number of bits per nibble in Tiva registers
#define BitsPerNibble 4

// number of bytes sent/received to/from LOC with each interaction with the LOC
#define numReceivedBytes 4

// Commands to send to the LOC
// Status command
#define StatusCommand (BIT7HI|BIT6HI)

// Report staging area frequency command
#define FreqCommand BIT7HI

// Query new response ready command
#define QueryCommand (BIT6HI|BIT5HI|BIT4HI)

// Number of bits to shift 8 bit number to the top of a 16 bit number
#define NumResponseBits 8

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitSerialHardware( void );
static uint16_t Transmit2LOC( uint8_t );

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;

// current state of SPI state machine
static SPIState_t CurrentState;

// received data from data register
static uint8_t ReceivedData;
static uint8_t ReceivedLOCData[5];

// ISR events
static ES_Event ISREvent; 
static ES_Event LastEvent;

// Last command byte to LOC
static uint8_t LastCommand;

// Last response byte from the LOC
static uint8_t LastResponse;

// variable defining red or green team
static uint8_t RED = 0;
static uint8_t GREEN = 1;
static uint8_t TeamColor;

// variable defining byte count of each transmission
static uint8_t ByteCount = 0;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSPIService

 Parameters
     void

 Returns
     void

 Description
     Initializes this service to be the top level state of the HSM
 Notes

 Author
     Team 16, 02/04/17, 16:00
****************************************************************************/
bool InitSPIService ( uint8_t Priority )
{
	 MyPriority = Priority;

	 // set LastEvent parameter to 0
	 LastEvent.EventParam = 0x00;
	 
	 // Initialize hardware
	 InitSerialHardware();
	 
	// Initialize shorttimer 
	ES_Timer_InitTimer(SPI_TIMER,SPIPeriod);
	
	// Initialize team color as red
	TeamColor = RED;
	
	// Initialize current state
	CurrentState = WAITING2TRANSMIT;
	
	 return true;
}

/****************************************************************************
 Function
     RunSPIService

 Parameters
     void

 Returns
     ES_Event ThisEvent

 Description
     Xmits info
 Notes

 Author
     Team 16, 02/04/17, 16:00
****************************************************************************/
ES_Event RunSPIService ( ES_Event CurrentEvent )
{	
	printf("\r\n Run SPI: %i \r\n",CurrentEvent.EventType);
  // define return event if no errors persist 
  ES_Event ReturnEvent;
	ReturnEvent.EventType = ES_NO_EVENT;

  // WAITING2TRANSMIT State
  if(CurrentState == WAITING2TRANSMIT)
  {
		if(CurrentEvent.EventType == TEAM_COLOR){
			
			// set team color from the parameter of the TEAM_COLOR event
			TeamColor = CurrentEvent.EventParam;
			
		} else {
			// change state to TRANSMITTING
			CurrentState = TRANSMITTING;

			
			// define event to post to RobotSM 
			ES_Event PostEvent;
			PostEvent.EventType = ES_NO_EVENT;

			// transmit to the LOC the bytes corresponding to the type of command event 	
			if(CurrentEvent.EventType == ROBOT_QUERY)
			{printf("\r\n ROBOT_QUERY \r\n");
				// set Query event type and send data to LOC, and set event param to response
				PostEvent.EventType = COM_QUERY_RESPONSE;
				PostEvent.EventParam = Transmit2LOC( QueryCommand );
				
			} else if (CurrentEvent.EventType == ROBOT_FREQ_RESPONSE) {
				printf("\r\n ROBOT_FREQ_RESPONSE \r\n");
				
			// set last four bits to the frequency the RobotSM has determined
				printf("\r\n FREQ:%x \r\n",CurrentEvent.EventParam);
			uint8_t FCommand = (FreqCommand|CurrentEvent.EventParam);
				
			// set Freq event type and send data to LOC, and set event param to response
			PostEvent.EventType = COM_FREQ_REPORT;
			PostEvent.EventParam = Transmit2LOC( FCommand );

			} else if (CurrentEvent.EventType == ROBOT_STATUS){
				printf("\r\n ROBOT_STATUS \r\n");
			// set Query event type and send data to LOC, and set event param to response
			PostEvent.EventType = COM_STATUS;
			PostEvent.EventParam = Transmit2LOC( StatusCommand );
			}

			// Initialize shorttimer to wait for 2 ms 
			ES_Timer_InitTimer(SPI_TIMER,SPIPeriod);
			
			// Post event to RobotTopSM
			//PostRobotTopSM(PostEvent);
		}

	// TRANSMITTING State
	} else if (CurrentState == TRANSMITTING) {

		if(CurrentEvent.EventType == ES_TIMEOUT) {

			if(ByteCount <= numReceivedBytes) {

				// transmit a 0x00 byte to the LOC
				HWREG(SSI0_BASE+SSI_O_DR) = 0;

				// Increment ByteCount
				ByteCount = ByteCount + 1;

				// reset timer 
				ES_Timer_InitTimer(SPI_TIMER,SPIPeriod);

			} else {

				// clear ByteCount
				ByteCount = 0;

				// Change state to 
				CurrentState = WAITING2TRANSMIT;
			}
		}
	}	
		
	return ReturnEvent;
}

/****************************************************************************
 Function
     PostSPIService

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
		 bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Team 16, 02/04/17, 16:00
****************************************************************************/
bool PostSPIService( ES_Event ThisEvent )
{
	 return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
     SPI_InterruptResponse

 Parameters
     void

 Returns
     void

 Description
     Initializes hardware in the Tiva to create an SPI master

 Author
     Team 16, 02/04/17, 16:00
****************************************************************************/
void SPI_InterruptResponse( void )
{	
	// clear interrupt
	HWREG(SSI0_BASE + SSI_O_IM) &= (~SSI_IM_TXIM);

	// read command 
	ReceivedData = HWREG(SSI0_BASE+SSI_O_DR);	
	
	// post command to action service
	ISREvent.EventType = ISR_COMMAND;
	ISREvent.EventParam = ReceivedData;
	//PostActionService(ISREvent);
	
	// set current state to WAITING2TRANSMIT
	CurrentState = WAITING2TRANSMIT;
}


/*----------------------------------------------------------------------------
private functions
-----------------------------------------------------------------------------*/

/****************************************************************************
 Function
     Transmit2LOC

 Parameters
     first 8 bits to write to data register

 Returns
     void

 Description
     writes the first 8 bits on Tx line, then 0 four more times

 Author
     Team 16, 02/04/17, 23:00
****************************************************************************/
static uint16_t Transmit2LOC( uint8_t Command )
{		
	// define return data variable 
	uint16_t ReturnedData = 0;
	
	//Enable the NVIC interrupt for the SSI
	HWREG(SSI0_BASE + SSI_O_IM) |= SSI_IM_TXIM;
	
	// write command to the data register
	HWREG(SSI0_BASE+SSI_O_DR) = Command;
	
	// write 0 to the data register four more times
//	HWREG(SSI0_BASE+SSI_O_DR) = 0;
//	HWREG(SSI0_BASE+SSI_O_DR) = 0;
//	HWREG(SSI0_BASE+SSI_O_DR) = 0;
//	HWREG(SSI0_BASE+SSI_O_DR) = 0;
	
	// read data from the data
//	for(int i=0;i<numReceivedBytes;i++){
//		ReceivedLOCData[i] = HWREG(SSI0_BASE+SSI_O_DR);
//	}
	ReceivedLOCData[0] = HWREG(SSI0_BASE+SSI_O_DR);
	// return the correct data for the corresponding Command into this function
	if(Command == QueryCommand){
		// set ReturnedData to Response Ready byte (shifted by 8) and Report Status Byte
		ReturnedData = ((ReceivedLOCData[1]<<NumResponseBits)|ReceivedLOCData[2]);
		
	} else if (Command == FreqCommand) {
		// set ReturnedData to the first two bytes of the total response from the LOC
		
		ReturnedData = ((ReceivedLOCData[0]<<NumResponseBits)|ReceivedLOCData[1]);
		
		
	} else if (Command == StatusCommand){
		// set ReturnedData to SB1 byte (bit shifted by 8) and SB2 or SB3 depending on red or green team
		if(TeamColor == RED){	
			ReturnedData = ((ReceivedLOCData[1]<<NumResponseBits)|ReceivedLOCData[3]);
		} else {
			ReturnedData = ((ReceivedLOCData[1]<<NumResponseBits)|ReceivedLOCData[2]);
		}
		
	}
	
	// return the relevant data from the LOC
	printf("\r\n Returned data:%x \r\n",ReturnedData);
	return ReturnedData;
}

/****************************************************************************
 Function
     InitSerialHardware

 Parameters
     void

 Returns
     void

 Description
     keeps the service init more readable

 Author
     Team 16, 02/04/17, 16:00
****************************************************************************/
static void InitSerialHardware(void)
{

	//Enable the clock to the GPIO port
	HWREG(SYSCTL_RCGCGPIO)|= SYSCTL_RCGCGPIO_R0;		
	
	// Enable the clock to SSI module
	HWREG(SYSCTL_RCGCSSI) = SSIModule;	
	
	// Wait for the GPIO port to be ready
	while((HWREG(SYSCTL_RCGCGPIO) & SYSCTL_PRGPIO_R0) != SYSCTL_PRGPIO_R0){};
		
	// Program the GPIO to use the alternate functions on the SSI pins
	HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= (RXPIN|TXPIN|SPIClock|SlaveSelect);	
		
	//Set mux position in GPIOPCTL to select the SSI use of the pins 
	// map bit RXPIN's alt function to SSI0Rx (2), by clearing nibble then shifting 2 to 4th nibble 
	HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) & 0xfff0ffff) + (2<<(4*BitsPerNibble));
		
	// map bit TXPIN's alt function to SSI0Tx (2), by clearing nibble then shifting 2 to 5th nibble 
	HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) & 0xff0fffff) + (2<<(5*BitsPerNibble));		
		
	// map bit SPIClock's alt function to SSI0Clk (2), by clearing nibble then shifting 2 to 2nd nibble 
	HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) & 0xfffff0ff) + (2<<(2*BitsPerNibble));	
		
	// map bit SlaveSelect's alt function to SSI0Fss (2), by clearing nibble then shifting 2 to 3rd nibble 
	HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) = (HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) & 0xffff0fff) + (2<<(3*BitsPerNibble));	
		
	// Program the port lines for digital I/O
	HWREG(GPIO_PORTA_BASE + GPIO_O_DEN)|= (RXPIN|TXPIN|SPIClock|SlaveSelect);
		
	// Program the required data directions on the port lines
	HWREG(GPIO_PORTA_BASE + GPIO_O_DIR)|= (RXPIN|TXPIN|SPIClock|SlaveSelect);
		
	// If using SPI mode 3, program the pull-up on the clock line
	HWREG(GPIO_PORTA_BASE + GPIO_O_PUR) |= SPIClock;		
	
	// Wait for the SSI0 to be ready  CHECK
	while ((HWREG(SYSCTL_RCGCSSI)& SYSCTL_RCGCSSI_R0)!= SYSCTL_RCGCSSI_R0);			
	
	// Make sure that the SSI is disabled before programming mode bits
	HWREG(SSI0_BASE + SSI_O_CR1) &= (~SSI_CR1_SSE);	
	
	// Select master mode (MS) & TXRIS indicating End of Transmit (EOT)
	HWREG(SSI0_BASE + SSI_O_CR1) &= (~SSI_CR1_MS);
	HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
	
	// Configure the SSI clock source to the system clock
	HWREG(SSI0_BASE + SSI_O_CC) = SSI_CC_CS_SYSPLL; 
	
	// Configure the clock pre-scaler
	HWREG(SSI0_BASE + SSI_O_CPSR) = CPSDVSR;
	
	// Configure clock rate (SCR) by clearing with a mask and then shifting SCR two nibbles
	HWREG(SSI0_BASE + SSI_O_CR0) = (HWREG(SSI0_BASE + SSI_O_CR0) & 0xffff00ff)+(SCR<<(2*BitsPerNibble));	
	//HWREG(SSI0_BASE + SSI_O_CR0) |= (SCR<<(2*BitsPerNibble));	
	
	// Configure phase & polarity (SPH, SPO), data size (DSS)
	HWREG(SSI0_BASE + SSI_O_CR0) |= (SSI_CR0_SPH |SSI_CR0_SPO|SSI_CR0_DSS_8);		
	
	// Configure mode(FRR) using mask to select Freescale SPI Frame Format as FRF mode by clearing
	HWREG(SSI0_BASE + SSI_O_CR0) &= (~SSI_CR0_FRF_M);

	// Locally enable interrupts (TXIM in SSIIM) 
	//unmasking -tiva DS pg.977
	HWREG(SSI0_BASE + SSI_O_IM) |= SSI_IM_TXIM;
	
	// Make sure that the SSI is enabled for operation
	HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_SSE;

	//Enable the NVIC interrupt for the SSI when starting to transmit
	//Interrupt number -tiva DS pg.104
	HWREG(NVIC_EN0) |= SSI_NVIC_HI;
	
	printf("\r\nGot thru SPI interrupt init\n");
}

#ifdef TEST
int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
			| SYSCTL_XTAL_16MHZ);
	TERMIO_Init();
	clrScrn();
	printf("\r\n Starting SPI Test \r\n");
	
	InitSerialHardware();
	Transmit2LOC();
	
	return 0;
}
#endif
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
