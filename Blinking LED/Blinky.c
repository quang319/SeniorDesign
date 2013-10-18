/* Quang Nguyen
 * GCRobotics
 * Created 10/14/2013
 * 
 *			 ````` Use for Prototype robot `````````
 * The function of this code is to receive i2cSpeed and i2cDirection from the Arduino
 * and execute it. PID and encoder counts will not be taken into account for this code
 *
 *
 *			/////// Future Work /////////
 * Need to work on error code for PORTD. Right now, LEDs from PORTD doesn't 
 * have any meaning. 
 * 
 */

#include <pic16f887.h>          // 16F887 header file, for all
                                //  processor-specific declarations
#include <htc.h>                // htc.h necessary for PIC16F917 Configuration
                                //  Bit Settings
	#ifndef _XTAL_FREQ
	#define _XTAL_FREQ 	20000000
	#endif


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

__CONFIG(FOSC_HS & WDTE_OFF & PWRTE_OFF & MCLRE_ON &
        CP_OFF & CPD_OFF & BOREN_OFF & IESO_OFF & FCMEN_OFF & LVP_OFF);
__CONFIG(BOR4V_BOR40V & WRT_OFF);

int COUNTS = 0;

#define ON 	0xFF
#define OFF	0x00
// Function Prototypes
void Initialise();              // contains all initializing functions

//-----------------------------------------------------------------------------
// MAIN STARTS HERE -----------------------------------------------------------
int main()
{
    // BEGIN
    Initialise();
    while(1)
    {
		PORTD = 0xFF;
		__delay_ms(500);
		PORTD = 0x00;
		__delay_ms(500);
    }   

    return 1;                   // standard ending for an "int main"
}   // END MAIN ---------------------------------------------------------------
//-----------------------------------------------------------------------------


// Any PIC initialization that is necessary goes here
void Initialise()
{
    TRISD = 0;
}

