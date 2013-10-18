/* Quang Nguyen
 * GCRobotics
 * Created 10/14/2013
 * 
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

__CONFIG(FOSC_HS & WDTE_OFF & PWRTE_ON & MCLRE_ON &
        CP_OFF & CPD_OFF & BOREN_OFF & IESO_OFF & FCMEN_OFF & LVP_OFF);
__CONFIG(BOR4V_BOR40V & WRT_OFF);


// Pre-processor definitions that specify an individual PIC.  That is, all
//  the main code can be exactly the same, from PIC to PIC for the robot,
//  depending on the motor position (front left motor, for example), only
//  these definitions will need to change

// Motor 1:  Front Right.  Address = 0x02.  Forward = 1.
// Motor 2:  Back Right.  Address = 0x04.  Forward = 1.
// Motor 3:  Back Left.  Address = 0x06.  Forward = 0.
// Motor 4:  Front Left.  Address = 0x08.  Forward = 0.

#define I2C_ADDRESS 0x08        // I2C address; unique to specific PIC
#define FORWARD 1               // PIC specific depending on wheel orientation
#define BACKWARD !FORWARD       // ^

#define FLAG_ADDRESS 0xAA       // Address for user defined flag register
                                //  According to datasheet, 0xAA is free
#define KP 3.0                  // PID P coefficient
#define KI 0.4                  // PID I coefficient
#define KD 0                    // PID D coefficient
#define KPID 1.0                // PID cycle/s to PWM dampening factor



// Function Prototypes
void Initialise();              // contains all initializing functions
void interrupt isr();           // general interrupt vector
void UpdateData(int c);         // takes in most recent count measurements and
                                //  adds them to current total
void setDirection(int dir);     // Sets the direction bit (PORTB bit 3)
int abs(int a, int b);          // Returns the absolute value of the difference
                                //  between two numbers

//Test Functions
void delay(int length);
void delay1sec();



//Global Variables
int TARGET = 0;                 // target speed passed down from CPU
int DIRECTION = FORWARD;       	// target derection passed down from CPU
int DIR_READ = FORWARD;         // value read from encoder flip-flop used
                                //  to keep track of current direction

int COUNTS = 0;                 // TMR1 encoder counts --> passed to CPU
									// Note: As you are reading through the code,
									// know that COUNTS is different from counts.
									// COUNTS is what will be passing to the CPU 
									// and is used for the odometry.
									// counts is the variable used to store
									// encoder over a long period of time.


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
	
	// Variables for PID
    int 	counts 	= 0;                	// number of counts since last PID loop
    int    	ERROR 	= 0;                	// error variable
    int		ACC_ERROR = 0;                	// integral variable
    int		D;                         	 	// differential variable
    int		PREV_ERROR= 0;                  // old error variable
    int		PID;                        	// sum of P, I, and D values
	
    int currentPWM = 0;             		// current pulse width pushed to PWM

    // BEGIN
    Initialise();
    while(1)
    {
		PORTD = 0xFF;							// Clear error flag
        if (F.I2C == 1)
        {
            // Perform some I2C operation
            // If read, reset Data and PID information
            // Perform operation for TARGET

            TARGET = i2cSpeed;
            setDirection(!i2cDirection);
            SetPulse(i2cSpeed);

			// Reset the variables for PID and odometry //
            COUNTS = 0;
            ACC_ERROR = 0;
            D = 0;
            PREV_ERROR = 0;

            // Clear Flag
            F.I2C = 0;
        }

        if (F.DIR == 1)
        {
            // Update counts before updating direction
            EncUpdate(&counts);				//This will put the value of TMR1 into counts and then clear TMR1
			COUNTS = currentPWM;			//Quang: Send back PID data for graphing and tuning
			
//            UpdateData(counts);			// This will add counts to COUNTS (which is the total distanced traveled so far.

            // Update direction
            DIR_READ = RB5;

            // Clear Flag
            F.DIR = 0;
        }

        //  PID Loop occurs at regular time intervals
        if (F.T0 == 1)
        {
            // Update to most recent encoder counts
            EncUpdate(&counts);
			COUNTS = currentPWM;			// Quang: Send back PID data for graphing and tuning
 //           UpdateData(counts);		

            // Perform PID
           	ERROR = TARGET - counts;
            ACC_ERROR = ACC_ERROR + ERROR;

			// Set bounds for the accumulated error ///
            if (ACC_ERROR > 200)
                ACC_ERROR = 200;
            else if (ACC_ERROR < -200)
                ACC_ERROR = -200;

			// determining PID
//            D = abs(P, P_old);              // calculate differential error
            PID = (ERROR * KP) + (ACC_ERROR * KI);// + (D * KD);   // calculate new output
            PREV_ERROR = ERROR;                      // save error for next time

            if (ERROR != 0)					// If no error   
            {
//                currentPWM = PID + 65;
//                SetPulse(currentPWM);       // set new PWM
            }


            F.T0 = 0;                   // reset TMR0 flag
        } // end PID Loop               */

		SetPulse(80);
		__delay_ms(2000);
		SetPulse(140);
		__delay_ms(2000);
		SetPulse(200);
		__delay_ms(2000);
		SetPulse(240);
		__delay_ms(2000);

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
	
	__delay_ms(10);			// let motor settle

	PEIE = 1;               // generic peripheral interrupts enabled
    PIE1 = 0b00000000;      // I2C interrupts disable
 	SSPIF = 0;				// Clear I2C flag
    GIE = 1;				// Enable all interrupts

    BeginEncoder();         // initialize encoder registers (TMR0 & TMR1)
    PIE2 = 0;               // other peripherals disabled

	// Clear all TIMER registers
	TMR1 = 0;
	TMR0 = 0;
    // Configure interrupts
    RBIE = 1;               // PORTB interrupts enabled
    T0IE = 1;               // TMR0 interrupts enabled
	TMR1IE = 1;				// TMR1 interrupts enabled

    // Clear flags
    RBIF = 0;
    T0IF = 0;
    TMR1IF = 0;
    // Enable all interrupts


    TRISB = 0b11111111;
    PORTBbits.RB3 = FORWARD;    // default to forward

    TRISD = 0;
    PORTCbits.RC1 = 0;
}


// Primary Interrupt vector
void interrupt isr()
{
    if (SSPIF == 1)             // interrupt is I2C related
    {	
		PORTD = 0x01;			// I2C error indicator: Got stuck in interrupt isr
	
		__delay_ms(1000);		// We should never enter this

        F.I2C = 1;              // set i2c flag bit
        i2cIsrHandler();		// interrupt flag was cleared in this function
    } else if (T0IF == 1)       // overflow of timer 0
    {
        // TMR0 overflows every 13.11 ms
        F.T0 = 1;				// set T0 flag bit
        T0IF = 0;
    } else if (TMR1IF == 1)     // overflow of counts; probably never happens
    {		// Only 2 bytes of data are being sent through I2C so therefore, the 
			// overflown is not taken into account right now
        TMR1IF = 0;
    } else if (RBIF == 1)       // PORTB change
    {
        if (RB5 != DIR_READ)    // verify that it was a direction change
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
    if (DIR_READ == FORWARD)
    {
        COUNTS += c;
    } else
    {
        COUNTS -= c;
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


// Returns the absolute value of the difference between two numbers
int abs(int a, int b)
{
    int temp;
    temp = a - b;
    if (temp < 0)
        temp = temp * -1;
    return temp;
}


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


