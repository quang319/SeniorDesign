/* Alan Hale III	2013/3/18
 * Josh Galicic		2013/3/18
 * GCRobotics
 * Creator:
 *              Alan Hale III   3/18/2013
 *              Josh Galicic    3/18/2013
 * Revise by:
 *              Quang Nguyen    10/9/2013
 *
 *  This libary is for setting up and using the PIC I2C protocol
 *	The 2 main functions in this libary are:
 *	 	
 *.		1. i2cInit(char address)
 * 			This function just setup the bits needed to run I2C
 *			*NOTE* You still need to turnon PEIE and I2C bit of PIE1
 *
 *		2. i2cIsrHandler()
 *			This is function is handle everything else for the I2C.		
 *		
 *			To receive data from master, the master should send 3 bytes of data.
 *                      The first byte contain the type of the message.
 *                          There is only 1 type right now. We can definitely expand on this
 *                          and incorporate other types.
 *			The last 2 bytes will be stored in i2cTarget and i2cDirection.
 *			
 *			To send data to master, the master should request 2 bytes of data.
 *			OdometryCounts will be the data that will be sent to the master. It is up
 *			to the master to piece 2 bytes back to 1 interger.
 *
 * Timing infomation:
 *	  It takes roughly 400 us to read from master or write to master. 
 *			This timing is for the entire operation, not just timing for sending
 *			or receiving just one byte of data.
 *
 */



#include "i2cSlave887.h"
#include <pic16f887.h>

extern   int OdometryCounts;          // load from primary887main.c

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


    SSPADD = address;
    i2cBufferVal = 0;
    i2cWriteInt = 0;  // start with lowest byte
}

void i2cIsrHandler(){

//SSP_Handler

   // setting up for 3 byte messages as follows:
   // | type byte | msg data |
			//
			// 3 bytes are received when slave is reading from master.
			// Hence this is the reason why the arduino is sending sending out 3 bytes
			//
			// Only 2 bytes are sent when slave is writing to master
			//
   // Address is handled by the pic, so is ignored by our isr code.
   
   // Bit 3 of SSPSTAT = R/W bit
   // Bit 5 of SSPSTAT = D/A bit

    if ((SSPSTAT & 0b00100100) == 0b00100000){ // D_A high, R_W low, write w/ data in buffer
        i2cBuffer[i2cBufferVal] = SSPBUF;
        i2cBufferVal++;

    } else if ((SSPSTAT & 0b00100100) == 0b00000000){ // D_A low,R_W low, write w/ addr in buffer
        // Empty the buffer and carry on, this shouldn't happen since the pic does addr matching.
        SSPBUF = 0;

    } else if ((SSPSTAT & 0b00100100) == 0b00000100){ // D_A low,R_W high, read w/ addr in buffer
		// Not sure why, but the first char of data is being sent in this statement
        if (i2cWriteInt == 0)
        {
            i2cWriteInt = 1;
            i2cSend(OdometryCounts);
        }
	
    } else if ((SSPSTAT & 0b00100100) == 0b00100100){ // D_A high,R_W high, read w/ data in buffer
		// The second char of the message is being sent though
        if (i2cWriteInt == 1)
        {
            i2cWriteInt = 0;
            i2cSend(OdometryCounts >> 8);
        }
    }else
	{
            ;
	}
       
    SSPIF = 0;

    if (i2cBufferVal >= 3){
        i2cBufferVal = 0;
        i2cDataUpdate();
    }
}

// This is where the different type can be utilize.
void i2cDataUpdate(){
    if (i2cBuffer[0] == 0)      // There is only one type so far
    {
        i2cTarget       = (i2cBuffer[1]);
        i2cDirection    = (i2cBuffer[2]);
    }
}

// See Data Sheet page 208 for Slave Send instructions
void i2cSend(char msg){

    SSPBUF = msg;
    SSPCONbits.CKP = 1;
}