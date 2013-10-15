/* Quang Nguyen
 * GCRobotics
 *
 * Created on October 214, 2013.
 *
 * Compatible with PIC16F887
 *
 *
 * This program is used to get a rough estimate of the encoder counts per rev.
 *
 ******************  Useage instruction  *************************************
 *
 * Once the program start up, there will be a 1 second delay and PWM duty cycle
 * 0%. The 1 second delay is used to ensure that the motor is definitely not moving.
 * After this 1 second period, you can determine the counts per rev of the motor by 
 * spinning the wheel by one rotation with you hand. You should now pause the program 
 * and bring up the TMR1 register in the WATCH window. This register should contain
 * the motor's counts per rev.
 *
 *****************************************************************************  
 *
 */

#include <pic16f887.h>          // 16F887 header file, for all
                                //  processor-specific declarations
#include <htc.h>                // htc.h necessary for PIC16F917 Configuration
                                //  Bit Settings
#include "pwmlib887.h"             // allows for use of PWM module
                                //  (use double quotes for user-defined)
#include "enclib887.h"             // allows for use of Encoder module
#include "i2cSlave887.h"             // allows for use of I2C module

// Configuration bit settings
// Use 20MHz external oscillator --> FOSC_HS
// Turn on external oscillator --> FOSC_HS
  // NOTE:  MCLRE must be set to ON because of this family of PIC (887.
  //  Must be used in conjunction with the MCLR pin via weak pullup.
  //  See section 14.2.2, page 211, of the 887 datasheet.
__CONFIG(FOSC_HS & WDTE_OFF & PWRTE_ON & MCLRE_ON &
        CP_OFF & CPD_OFF & BOREN_OFF & IESO_OFF & FCMEN_OFF & LVP_OFF);
__CONFIG(BOR4V_BOR40V & WRT_OFF);

#define FLAG_ADDRESS 0xAA       // Address for user defined flag register
                                //  According to datasheet, 0xAA is free

int COUNTS = 0;                 // TMR1 encoder counts --> passed to CPU

// Function Prototypes
void Initialise();              // contains all initializing functions
void interrupt isr();           // general interrupt vector


//Test Functions
void delay1sec();

// Register that holds flags that are set in software upon determination of
//  the cause of an interrupt.  These flags are continuously checked in the
//  main program.  Some may be unimplemented.
unsigned char FLAG @ FLAG_ADDRESS;
bit I2C     @ ((unsigned)&FLAG*8)+0;    // I2C flag
bit T0      @ ((unsigned)&FLAG*8)+1;    // TMR0 flag
bit T1      @ ((unsigned)&FLAG*8)+2;    // TMR1 flag
bit DIR     @ ((unsigned)&FLAG*8)+3;    // Direction flag
bit ERR     @ ((unsigned)&FLAG*8)+4;    // H-Bridge Error flag
union {
    struct {
        unsigned    I2C     : 1;
        unsigned    T0      : 1;
        unsigned    T1      : 1;
        unsigned    DIR     : 1;
        unsigned    ERR     : 1;
        unsigned            : 3;
    };
}F @ FLAG_ADDRESS;


//-----------------------------------------------------------------------------
// MAIN STARTS HERE -----------------------------------------------------------
int main()
{
    // BEGIN
    Initialise();
    while(1)
    {
		;						// Thers is no instructions in this loop because TMR1 count by it self and we only need to stop the program 
								// look at the register TMR1 through the watch screen. 
    } // end while(1)

    return 1;                   // standard ending for an "int main"
}   // END MAIN ---------------------------------------------------------------
//-----------------------------------------------------------------------------


// Any PIC initialization that is necessary goes here
void Initialise()
{
    FLAG = 0;
	BeginPWM();             // initialize PWM associated registers
	SetPulse(0);			// Set PWM to 0 until Arduino say otherwise
	PEIE = 1;               // generic peripheral interrupts enabled
    GIE = 1;				// Enable all interrupts
    BeginEncoder();         // initialize encoder registers (TMR0 & TMR1)

	delay1sec();			// Delay 1 sec so motor can stop spinning

	// Clear all TIMER registers
	TMR1 = 0;
	TMR0 = 0;

    // Configure interrupts
    RBIE = 1;               // PORTB interrupts enabled
	TMR1IE = 1;				// TMR1 interrupts enabled

    // Clear flags
    RBIF = 0;
    T0IF = 0;
    TMR1IF = 0;
    // Enable all interrupts


    TRISB = 0b11110111;
    PORTBbits.RB3 =		1;    // default to forward

    TRISD = 0;
    PORTCbits.RC1 = 0;
}


// Primary Interrupt vector
void interrupt isr()
{
	if (TMR1IF == 1)     // overflow of counts; probably never happens
    {		// Only 2 bytes of data are being sent through I2C so therefore, the 
			// overflown is not taken into account right now
        TMR1IF = 0;
    } else if (RBIF == 1)       // PORTB change
    {
        F.DIR = 1;          	// set direction flag bit
        RBIF = 0;
    } else                      // Any other interrupt error
    {
        TRISD = 0;
        PORTD = 0xFF;
    }
}       // end interrupt function

void delay1sec()
{
    int i;
    // Loop 76 times for 1 second delay (20MHz, 256 prescale)
    for(i = 0; i <= 76; i++)
    {
        while (T0IF == 0)
            asm("nop");
        T0IF = 0;
    }
}


