/* Quang Nguyen
 * GCRobotics
 * Created 10/14/2013
 * 
 *			 ````` Use for Prototype robot `````````
 * The function of this code is to receive i2cTarget and i2cDirection from the Arduino
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
#include "i2cSlave887.h"            // allows for use of I2C module

// Configuration bit settings
// Use 20MHz external oscillator --> FOSC_HS
// Turn on external oscillator --> FOSC_HS
  // NOTE:  MCLRE must be set to ON because of this family of PIC (887.
  //  Must be used in conjunction with the MCLR pin via weak pullup.
  //  See section 14.2.2, page 211, of the 887 datasheet.

__CONFIG(FOSC_HS & WDTE_OFF & PWRTE_OFF & MCLRE_ON &
        CP_OFF & CPD_OFF & BOREN_OFF & IESO_OFF & FCMEN_OFF & LVP_OFF);
__CONFIG(BOR4V_BOR40V & WRT_OFF);


// Pre-processor definitions that specify an individual PIC.  That is, all
//  the main code can be exactly the same, from PIC to PIC for the robot,
//  depending on the motor position (front left motor, for example), only
//  these definitions will need to change

/***************** I2C address; unique to specific PIC ******************/
//#define I2C_ADDRESS 0x02        // FRONT RIGHT motor address
//#define I2C_ADDRESS 0x04        // BACK RIGHT motor address
#define I2C_ADDRESS 0x06        // BACK LEFT motor address
//#define I2C_ADDRESS 0x08        // FRONT LEFT motor address
/************************************************************************/
/****************  Motor direction: This changes depends on the side of the motor ****/
//#define MOTOR_DIRECTION i2cDirection
#define MOTOR_DIRECTION !i2cDirection
/*************************************************************************/
#define FORWARD     1               // PIC specific depending on wheel orientation
#define BACKWARD    !FORWARD       // ^
#define PWM_OFFSET  40           // Motor won't spin until a certain voltage is applied
                                // to it.
#define DT          0.1         // Delta Time

#define FLAG_ADDRESS 0xAA       // Address for user defined flag register
                                //  According sto datasheet, 0xAA is free
//#define KP 2.0                  // PID P coefficient
//#define KI 0.8                  // PID I coefficient
//#define KD 0                    // PID D coefficient
//#define KPID 1.0                // PID cycle/s to PWM dampening factor

double   KP        = 3.5;
double   KI        = 1.7;
double   KD        = 2;

// Function Prototypes
void Initialise();              // contains all initializing functions
void interrupt isr();           // general interrupt vector
void UpdateData(int c);         // takes in most recent count measurements and
                                //  adds them to current total
void setDirection(int dir);     // Sets the direction bit (PORTB bit 3)


//Global Variables
int Target              = 0;                // target speed passed down from CPU
int Direction           = FORWARD;       	// target derection passed down from CPU
int DirectionRead       = FORWARD;          // value read from encoder flip-flop used
                                            //  to keep track of current direction
int TMR0OverflowCounter   = 0;                // This will keep track of the number of times that
                                            // TMR0 has overflowed.
                                            // TMR0 is set to overflow at 10 ms so
                                            // 5 times of TMR0 overflow equal to 100 ms or 0.1 second.
                                            // This is how often our PID will run

int OdometryCounts          = 0;           // TMR1 encoder counts --> passed to CPU
                                           
int PID                 = 0;                // sum of P, I, and D values
int EncoderCounts       = 0;                // number of counts since last PID loop
int Error               = 0;                // error variable
int AccumulatedError    = 0;                // integral variable
int DeltaError          = 0;
int PreviousError       =0;
int CurrentPwm          = 0;                // current pulse width pushed to PWM

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
		PORTD = 0xFF;               // Have Flag bits on just to indicate
                                            // that PIC is running
        if (F.I2C == 1)
        {
            // Perform some I2C operation
            // If read, reset Data and PID information
            // Perform operation for Target

            Target = i2cTarget;
            setDirection(MOTOR_DIRECTION);
			// Reset the variables for PID and odometry //
            OdometryCounts = 0;
//            AccumulatedError = 0;

            // Clear Flag
            F.I2C = 0;
        }
        if (F.DIR == 1)
        {
            // Update counts before updating direction
            EncUpdate(&EncoderCounts);				//This will put the value of TMR1 into counts and then clear TMR0
            UpdateData(EncoderCounts);			// This will add counts to OdometryCounts (which is the total distanced traveled so far.)

            // Update direction
            DirectionRead = RB5;

            // Clear Flag
            F.DIR = 0;
        }

        //  PID Loop occurs at regular time intervals
        if (F.T0 == 1)
        {
            // Update to most recent encoder counts
            EncUpdate(&EncoderCounts);
            UpdateData(EncoderCounts);

            // Perform PID
            Error               = Target - EncoderCounts;
            AccumulatedError    = AccumulatedError + Error;
            DeltaError          = Error - PreviousError;
            PreviousError       = Error;

			// Set bounds for the accumulated error ///
            if (AccumulatedError > 10000)
                AccumulatedError = 10000;
            else if (AccumulatedError < -10000)
                AccumulatedError = -10000;

            PID = (Error * KP) + (AccumulatedError * KI) + (DeltaError * KD );     // Performing PID calculation
                                                        //Note: Even through the variable types are
                                                        // different, the result should be the correct
                                                        // value of with the type of int
            if (Error != 0)         // If no error
            {
                CurrentPwm = PID + PWM_OFFSET;
                if (CurrentPwm >= 255)
                    CurrentPwm = 255;
                SetPulse(CurrentPwm);       // set new PWM
 
            }
            F.T0 = 0;                   // reset TMR0 flag
        } // end PID Loop               */						// Clear error flag

    } // end while(1)

    return 1;                   // standard ending for an "int main"
}   // END MAIN ---------------------------------------------------------------
//-----------------------------------------------------------------------------


// Any PIC initialization that is necessary goes here
void Initialise()
{
    FLAG    = 0;
    BeginPWM();             // initialize PWM associated registers
    SetPulse(0);			// Set PWM to 0 until Arduino say otherwise
    i2cInit(I2C_ADDRESS);   // initialize I2C
    PEIE    = 1;               // generic peripheral interrupts enabled
    PIE1    = 0b00001000;      // I2C interrupts enabled
    SSPIF   = 0;				// Clear I2C flag
    GIE     = 1;				// Enable all interrupts

    BeginEncoder();         // initialize encoder registers (TMR0 & TMR1)
    PIE2    = 0;               // other peripherals disabled

// Clear and Initate TIMER registers
    TMR1    = 0;
    TMR0    = 61;
// Configure interrupts
    RBIE    = 1;               // PORTB interrupts enabled
    T0IE    = 1;               // TMR0 interrupts enabled
                                    // Need to re-enable
    TMR1IE  = 1;				// TMR1 interrupts enabled

    // Clear flags
    RBIF    = 0;
    T0IF    = 0;
    TMR1IF  = 0;
    // Enable all interrupts


    TRISB   = 0b11110111;
    PORTBbits.RB3 = FORWARD;    // default to forward

    TRISD   = 0;
    PORTCbits.RC1 = 0;
}


// Primary Interrupt vector
void interrupt isr()
{
    if (SSPIF == 1)             // interrupt is I2C related
    {	
	PORTD = 0x01;			// I2C error indicator: Got stuck in interrupt isr
	
        F.I2C = 1;              // set i2c flag bit
        i2cIsrHandler();		// interrupt flag was cleared in this function
    } else if (T0IF == 1)       // overflow of timer 0
    {                           // TMR0 overflows every 10 ms
        TMR0OverflowCounter++;
        if (TMR0OverflowCounter == 5){   // 10 times of TMR0 overflow = 100 ms or 0.1 second
            F.T0 = 1;				// set T0 flag bit
            TMR0OverflowCounter = 0;
        }
        T0IF = 0;
    } else if (TMR1IF == 1)     // overflow of counts; probably never happens
    {		// Only 2 bytes of data are being sent through I2C so therefore, the 
			// overflown is not taken into account right now
        TMR1IF = 0;
    } else if (RBIF == 1)       // PORTB change
    {
        if (RB5 != DirectionRead)    // verify that it was a direction change
        {
            F.DIR = 1;          // set direction flag bit
        } else                  // Error, probably H-Bridge related
        {
;
        }
        RBIF = 0;
    } else                      // Any other interrupt error
    {
;
    }
}       // end interrupt function


// Takes in variables holding the most recent time read and count read and adds
//  (or subtracsts, depending on the direction) those to the current totals
void UpdateData(int c)
{
    // Assuming no overflow occurs in longs

    // Add counts if going forward, subtract if going backwards
    if (DirectionRead == FORWARD)
    {
        OdometryCounts += c;
    } else
    {
        OdometryCounts -= c;
    }
}


// Sets the direction according to the direction value
void setDirection(int dir)
{
    if (dir == 0)          // if I2C passes down direction of 0, it is forward
        PORTBbits.RB3 = FORWARD;        // forward
    else if (dir == 1)
        PORTBbits.RB3 = BACKWARD;       // reverse
    else
        PORTBbits.RB3 = FORWARD;        // default to motor forward
}