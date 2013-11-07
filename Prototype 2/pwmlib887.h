/* 
 * File:   pwmlib.h
 * Author: Alan
 *
 * Created on December 8, 2012, 6:43 PM
 */

#ifndef PWMLIB_H
#define	PWMLIB_H

#ifdef	__cplusplus
extern "C" {
#endif


    //Function that configures the CCP module for PWM mode,
    //configure Timer2 to run
    void BeginPWM();
    
    //Function that sets the PWM period by loading PR2 with value
    void SetPR2(int length);

    //Function that sets PWM duty cycle by loading value into
    //CCPR1L and CCP1 bits (of CCP1CON register); 10-bit resolution
    void setPWM(int length);

    //Function that stops PWM by turning off Timer2
    void StopPWM();



#ifdef	__cplusplus
}
#endif

#endif	/* PWMLIB_H */

