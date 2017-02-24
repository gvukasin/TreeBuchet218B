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
#include "ActionService.h"

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

// CPSR divisor for SSI clock rate (2)
#define CPSDVSR 0x18

// SCR divisor for SSI clock rate (24)
#define SCR 0x01

// SPI period (4ms b/c need a minimum of 2ms)
// these times assume a 1.000mS/tick timing
#define TicksPerSec 976
#define SPIPeriod (TicksPerSec/25)

// defining ALL_BITS
#define ALL_BITS (0xff<<2)

#define BitsPerNibble 4

#define QueryBits 0xaa

#define numTransimittedBytes 5

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitSerialHardware( void );
static void QuerySPI( uint32_t Command[numTransimittedBytes]  );
static ES_Event DuringW2T( ES_Event );
static ES_Event DuringW4E( ES_Event );
static ES_Event DuringW4T( ES_Event );

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;

// current state of SPI state machine
static SPIState_t CurrentState;

// received data from data register
static uint8_t ReceivedData;

// ISR events
static ES_Event ISREvent;
static ES_Event LastEvent;

// Last command byte to LOC
static uint8_t LastCommand;

// Last response byte from the LOC
static uint8_t LastResponse;

// Commands to send to the LOC
// Status command
static uint32_t StatusCommand[5] = {0x0b,0x11,0x00,0x00,0x00};

// Report staging area frequency command
static uint32_t ReportFreqCommand[5] = {0x0b,0x10,0x00,0x00,0x00};

// Query new response ready command
static uint32_t QueryCommand[5] = {0x0b,0x01,0x11,0x00,0x00};


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

	// set ThisEvent for entry into this service's start function
	ES_Event ThisEvent;
	ThisEvent.EventType = ES_ENTRY_HISTORY;

	// start this state machine
	StartSPIService( ThisEvent );

	printf("\r\nGot through SPI init\r\n");
	
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
		bool MakeTransition = false;/* are we making a state transition? */
   SPIState_t NextState = CurrentState;
   ES_Event EntryEventKind = { ES_ENTRY, 0 };// default to normal entry to new state
   ES_Event ReturnEvent = { ES_NO_EVENT, 0 }; // assume no error

    switch ( CurrentState )
   {
       case WAITING2TRANSMIT :       // If current state is state one
         // Execute During function for state one. ES_ENTRY & ES_EXIT are
         // processed here allow the lowere level state machines to re-map
         // or consume the event
         CurrentEvent = DuringStateOne(CurrentEvent);
         //process any events
         if ( CurrentEvent.EventType != ES_NO_EVENT ) //If an event is active
         {
            switch (CurrentEvent.EventType)
            {
               case ES_LOCK : //If event is event one
                  // Execute action function for state one : event one
                  NextState = STATE_TWO;//Decide what the next state will be
                  // for internal transitions, skip changing MakeTransition
                  MakeTransition = true; //mark that we are taking a transition
                  // if transitioning to a state with history change kind of entry
                  EntryEventKind.EventType = ES_ENTRY_HISTORY;
                  break;
                // repeat cases as required for relevant events
            }
         }
         break;
				 
				 case WAITING4EOT :       // If current state is state one
         // Execute During function for state one. ES_ENTRY & ES_EXIT are
         // processed here allow the lowere level state machines to re-map
         // or consume the event
         CurrentEvent = DuringStateOne(CurrentEvent);
         //process any events
         if ( CurrentEvent.EventType != ES_NO_EVENT ) //If an event is active
         {
            switch (CurrentEvent.EventType)
            {
               case ES_LOCK : //If event is event one
                  // Execute action function for state one : event one
                  NextState = STATE_TWO;//Decide what the next state will be
                  // for internal transitions, skip changing MakeTransition
                  MakeTransition = true; //mark that we are taking a transition
                  // if transitioning to a state with history change kind of entry
                  EntryEventKind.EventType = ES_ENTRY_HISTORY;
                  break;
                // repeat cases as required for relevant events
            }
         }
         break;
				 
				 case WAITING4TIMEOUT :       // If current state is state one
         // Execute During function for state one. ES_ENTRY & ES_EXIT are
         // processed here allow the lowere level state machines to re-map
         // or consume the event
         CurrentEvent = DuringStateOne(CurrentEvent);
         //process any events
         if ( CurrentEvent.EventType != ES_NO_EVENT ) //If an event is active
         {
            switch (CurrentEvent.EventType)
            {
               case ES_LOCK : //If event is event one
                  // Execute action function for state one : event one
                  NextState = STATE_TWO;//Decide what the next state will be
                  // for internal transitions, skip changing MakeTransition
                  MakeTransition = true; //mark that we are taking a transition
                  // if transitioning to a state with history change kind of entry
                  EntryEventKind.EventType = ES_ENTRY_HISTORY;
                  break;
                // repeat cases as required for relevant events
            }
         }
         break;
    }
    //   If we are making a state transition
    if (MakeTransition == true)
    {
       //   Execute exit function for current state
       CurrentEvent.EventType = ES_EXIT;
       RunSPIService(CurrentEvent);

       CurrentState = NextState; //Modify state variable

       // Execute entry function for new state
       // this defaults to ES_ENTRY
       RunSPIService(EntryEventKind);
     }
   // in the absence of an error the top level state machine should
   // always return ES_NO_EVENT, which we initialized at the top of func
   return(ReturnEvent);
	
	
//	//if(ThisEvent.EventType == NEXT_COMMAND)
//	if(ThisEvent.EventType == ES_TIMEOUT)
//	{		
//		// Initialize shorttimer 
//		ES_Timer_InitTimer(SPI_TIMER,SPIPeriod);
//		
//		// Idling State
//		if(CurrentState == Idling)
//		{		
//			// change state to Busy
//			CurrentState = Busy;
//			
//			// query the Command Generator
//			QuerySPI( QueryCommand );
//		} 
//	}
//	return ReturnEvent;
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
     StartSPIService

 Parameters
     ES_Event CurrentEvent

 Returns
     nothing

 Description
     Does any required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:15
****************************************************************************/
void StartMasterSM ( ES_Event CurrentEvent )
{
  // initialize the state variable if CurrentEvent is not enter history event
	if(CurrentEvent.EventType != ES_ENTRY_HISTORY ){
		
		// start in initial state of WAITING2TRANSMIT
		CurrentState = WAITING2TRANSMIT;
	}

  // now we need to let the Run function init the lower level state machines
  RunSPIService(CurrentEvent);
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
	PostActionService(ISREvent);
	
	// set current state to idling
	CurrentState = WAITING2TRANSMIT;
}

/****************************************************************************
 Function
     QuerySPI

 Parameters
     8 bits to write to data register

 Returns
     void

 Description
     writes 8 bits on Tx line 

 Author
     Team 16, 02/04/17, 23:00
****************************************************************************/
static void QuerySPI( uint32_t Command[numTransimittedBytes]  )
{		
	//Enable the NVIC interrupt for the SSI
	HWREG(SSI0_BASE + SSI_O_IM) |= SSI_IM_TXIM;
	
	// chop up 5 command bytes and write them to the data register sequentially
	HWREG(SSI0_BASE+SSI_O_DR) = QueryBits;
}

/*----------------------------------------------------------------------------
private functions
-----------------------------------------------------------------------------*/
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

/****************************************************************************
 Function
     DuringW2T

 Parameters
     ES_Event CurrentEvent

 Returns
     ES_Event ReturnEvent

 Description
     During function for Waiting2Transmit state 
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:15
****************************************************************************/
static ES_Event DuringW2T( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assme no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // implement any entry actions required for this state machine
        
        // after that start any lower level machines that run in this state
        //StartLowerLevelSM( Event );
        // repeat the StartxxxSM() functions for concurrent state machines
        // on the lower level
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // on exit, give the lower levels a chance to clean up first
        //RunLowerLevelSM(Event);
        // repeat for any concurrently running state machines
        // now do any local exit functionality
      
    }else
    // do the 'during' function for this state
    {
        // run any lower level state machine
        // ReturnEvent = RunLowerLevelSM(Event);
      
        // repeat for any concurrent lower level machines
      
        // do any activity that is repeated as long as we are in this state
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/****************************************************************************
 Function
     DuringW4E

 Parameters
     ES_Event CurrentEvent

 Returns
     ES_Event ReturnEvent

 Description
     During function for Waiting4EOT state 
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:15
****************************************************************************/
static ES_Event DuringW4E( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assme no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // implement any entry actions required for this state machine
        
        // after that start any lower level machines that run in this state
        //StartLowerLevelSM( Event );
        // repeat the StartxxxSM() functions for concurrent state machines
        // on the lower level
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // on exit, give the lower levels a chance to clean up first
        //RunLowerLevelSM(Event);
        // repeat for any concurrently running state machines
        // now do any local exit functionality
      
    }else
    // do the 'during' function for this state
    {
        // run any lower level state machine
        // ReturnEvent = RunLowerLevelSM(Event);
      
        // repeat for any concurrent lower level machines
      
        // do any activity that is repeated as long as we are in this state
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
}

/****************************************************************************
 Function
     DuringW2T

 Parameters
     ES_Event CurrentEvent

 Returns
     ES_Event ReturnEvent

 Description
     During function for Waiting2Timeout state 
 Notes

 Author
     J. Edward Carryer, 02/06/12, 22:15
****************************************************************************/
static ES_Event DuringW4T( ES_Event Event)
{
    ES_Event ReturnEvent = Event; // assme no re-mapping or comsumption

    // process ES_ENTRY, ES_ENTRY_HISTORY & ES_EXIT events
    if ( (Event.EventType == ES_ENTRY) ||
         (Event.EventType == ES_ENTRY_HISTORY) )
    {
        // implement any entry actions required for this state machine
        
        // after that start any lower level machines that run in this state
        //StartLowerLevelSM( Event );
        // repeat the StartxxxSM() functions for concurrent state machines
        // on the lower level
    }
    else if ( Event.EventType == ES_EXIT )
    {
        // on exit, give the lower levels a chance to clean up first
        //RunLowerLevelSM(Event);
        // repeat for any concurrently running state machines
        // now do any local exit functionality
      
    }else
    // do the 'during' function for this state
    {
        // run any lower level state machine
        // ReturnEvent = RunLowerLevelSM(Event);
      
        // repeat for any concurrent lower level machines
      
        // do any activity that is repeated as long as we are in this state
    }
    // return either Event, if you don't want to allow the lower level machine
    // to remap the current event, or ReturnEvent if you do want to allow it.
    return(ReturnEvent);
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
	QuerySPI();
	
	return 0;
}
#endif
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
