/*
 * File:   enclib.h
 * Author: Alan
 *
 * Created on February 6, 2013, 9:12 PM
 */

#ifndef ENCLIB_H
#define	ENCLIB_H

#ifdef	__cplusplus
extern "C" {
#endif


    // Function that initialises Timer0 as a timer and Timer1 as a counter
    void beginEncoder();


    // Function that takes in two pointers and sets one equal to TMR0 and
    //  the other equal to TMR1.  The values of TMR0 and TMR1 are then reset.
    void encUpdate(int *t1);







#ifdef	__cplusplus
}
#endif

#endif	/* ENCLIB_H */

