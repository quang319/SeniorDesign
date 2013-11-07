/* PWM Library
 *
 * Function implementation
 *
 * Alan, 2012/12/08
 *
 * COPIED FROM THE PIC16F917 DATASHEET
 * 
15.3.7 SETUP FOR PWM OPERATION
The following steps should be taken when configuring
the CCP module for PWM operation:
1. Disable the PWM pin (CCPx) output drivers by
setting the associated TRIS bit.
2. Set the PWM period by loading the PR2 register.
3. Configure the CCP module for the PWM mode
by loading the CCPxCON register with the
appropriate values.
4. Set the PWM duty cycle by loading the CCPRxL
register and CCPx bits of the CCPxCON register.
5. Configure and start Timer2:
? Clear the TMR2IF interrupt flag bit of the
PIR1 register.
? Set the Timer2 prescale value by loading the
T2CKPS bits of the T2CON register.
? Enable Timer2 by setting the TMR2ON bit of
the T2CON register.
6. Enable PWM output after a new PWM cycle has
started:
? Wait until Timer2 overflows (TMR2IF bit of
the PIR1 register is set).
? Enable the CCPx pin output driver by
clearing the associated TRIS bit.
 */

#include "pwmlib887.h"
#include <pic16f887.h>

//Function that configures the CCP module for PWM mode,
//configure Timer2 to run
void BeginPWM()
{
    //Disable PWM pin (CCP2) by setting associated TRIS bit;
    //  in this case, set RC1 as input, or TRISCbits.TRISC1 = 1
    TRISCbits.TRISC1 = 1;

    //Disable TMR2 interrupts by clearing PIE1 bit 1
    PIE1bits.TMR2IE = 0;

    //Set PWM period by loading PR2 register
    //  provide random value to start, can change later
    PR2 = 0xFF;

    //To configure CCP2 for PWM mode, bits 2 and 3 of CCP2CON
    //  must be set;  leave 2 LSBs of duty cycle clear for now
    CCP2CON = 0b00001100;  //sets CCP2 as PWM mode

    //Set the PWM duty cycle by loading CCPR2L register and
    //  CCP2 bits of the CCP2CON register; set duty cycle = 0
    CCPR2L = 0;
    CCP2CONbits.CCP2X = 0;
    CCP2CONbits.CCP2Y = 0;

    //Need to configure and start Timer2
    //  First, Clear the TMR2IF interrupt flag bit of the
    //  PIR1 register
    PIR1bits.TMR2IF = 0;
    //  Next, Set the Timer2 prescale value by loading the
    //  T2CKPS bits of the T2CON register; turn TMR2 on
    //  Postscale of 1:1, TMR2 on, Prescale of 1:1
    T2CON = 0b00000100;

    //Enable PWM output after a new PWM cycle has started:
    //  Wait until Timer2 overflows (TMR2IF bit of the PIR1 register is set)
    //  Enable the CCPx pin output driver by clearing the associated TRIS bit.
    while(1)  //waits until TMR2IF = 1
    {
        if(PIR1bits.TMR2IF == 1)    //repeatedly test overflow flag
        {
            TMR2IF = 0;             //clear flag
            TRISCbits.TRISC1 = 0;   //set PWM output to start
            break;                  //exit loop
        }
    }
}


//Function that sets the PWM period by loading PR2 with value
void SetPR2(int length)
{
    int temp = length;  //set intermediate variable to length value
    if (temp > 255)     //error test length
        temp = 255;     //if greater than 255 (1 byte's worth), set to 255
    PR2 = temp;         //load PR2 with new length value
}


//Function that sets PWM duty cycle by loading value into
//CCPR2L and CCP2 bits (of CCP2CON register); 10-bit resolution
//  NOTE:  for our purposes, we will only be using the 8 MSBs of the
//   CCPR2L register and will always leave the 2 LSBs clear.
//   If this does not give us the resolution we need, we will have to
//   add a function that takes "length" and parses the 2 LSBs off in
//   order to set them.  In other words, it would be harder to implement.
void setPWM(int length)
{
    int temp = length;  //set intermediate variable to length value
    if (temp > 255)     //error test length
        temp = 255;     //if greater than 255 (1 byte's worth), set to 255
    CCPR2L = temp;      //load CCPR2L with new length value

    //NOTE:  if CCPR2L > PR2 (duty cycle greater than period), it is the
    //  same as the duty cycle being 100%
}


//Function that stops PWM by turning off Timer2
void StopPWM()
{
    //Disable TMR2
    T2CONbits.TMR2ON = 0;
}
