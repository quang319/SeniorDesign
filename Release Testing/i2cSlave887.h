/* 
 * File:   i2cSlave.h
 * Author: Josh Galicic
 *
 * Created on December 27, 2012, 8:55 PM
 */

#ifndef I2CSLAVE_H
#define	I2CSLAVE_H


char i2cBufferVal;
char i2cBuffer[3];
char i2cRequest;
char i2cSpeed;
char i2cDirection = 0;
char i2cWriteInt;  // value that determines whether or not the upper or lower
                   //  byte of an integer will be passed to SSPBUF



void i2cInit(char address);
void i2cIsrHandler();
void i2cSend(char msg);
void i2cDataUpdate();



#endif	/* I2CSLAVE_H */

