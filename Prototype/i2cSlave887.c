/* Alan Hale III
 * Josh Galicic
 * GCRobotics
 *
 * 2013/3/18
 */



#include "i2cSlave887.h"
#include <pic16f887.h>

extern int COUNTS;          // load from primary887main.c



void i2cInit(char address){

    TRISC3 = 1;
    TRISC4 = 1;
    SSPEN = 1;
    CKE = 0;
    SMP = 0;
    CKP = 1;

    SSPM0 = 0;
    SSPM1 = 1;
    SSPM2 = 1;
    SSPM3 = 0;

//    SSPIE = 1;

    SSPADD = address;
//    PEIE = 1;
//    GIE = 1;
//    INTE = 1;
//    i2cBuffer[1] = 0;

    i2cBufferVal = 0;
    i2cWriteInt = 0;  // start with lowest byte
}

void i2cIsrHandler(){

//SSP_Handler

   // setting up for 3 byte messages as follows:
   // | type byte | msg data |
   // Address is handled by the pic, so is ignored by our isr code.
   
    int i = 0;
    i++;

    if ((SSPSTAT & 0b00100100) == 0b00100000){ // D_A high, R_W low, write w/ data in buffer
        i2cBuffer[i2cBufferVal] = SSPBUF;
        i2cBufferVal++;

    } else if ((SSPSTAT & 0b00100100) == 0b00000000){ // D_A low,R_W low, write w/ addr in buffer
        // Empty the buffer and carry on, this shouldn't happen since the pic does addr matching.
        SSPBUF = 0;

    } else if ((SSPSTAT & 0b00100100) == 0b0000100){ // D_A low,R_W high, read w/ addr in buffer
        // Again shoulnt happen, just here in case we ever need it.
	//SSPBUF = 20;
	i2cSend(i2cSpeed*2);
	
    } else if ((SSPSTAT & 0b00001100) == 0b00001100){ // D_A high,R_W high, read w/ data in buffer
	/*
        if (i2cWriteInt == 0)
        {
            i2cWriteInt = 1;
            i2cSend(COUNTS);
        } else
        {
            i2cWriteInt = 0;
            i2cSend(COUNTS >> 8);
        }
	*/
       
        i2cRequest = SSPBUF;

        if (i2cRequest == 1){ // requesting velocity       // SETUP THIS STUFF
            i2cSend(20);
        } else if (i2cRequest == 2){ // requesting errors
            i2cSend(20);
        }
	else
	{
	    ;
	}
    }
        

    SSPIF = 0;

    if (i2cBufferVal == 3){
        i2cBufferVal = 0;
        i2cDataUpdate();
    }
    return;
}


void i2cDataUpdate(){
    switch (i2cBuffer[0]) { // Check message type
        default : // Velocity
            i2cSpeed = (i2cBuffer[1]);
            i2cDirection = (i2cBuffer[2]);
            break;
    }
    return;
}

// See Data Sheet page 208 for Slave Send instructions
void i2cSend(char msg){
  //  int i = 0; Does this do anything??
  //  i++;
    SSPBUF = msg;
    SSPCONbits.CKP = 0;
    SSPCONbits.CKP = 1;

    return;
}