
/*

		NAME 	 - Arduino Control Bridge 
		DESIGN - Josh Galicic
		PRIORITY - Required - Neccessary for communicating between PC and Pics.
	
\
*/

//#define PIC_ADDRESS   0x02
#define PIC_ADDRESS   0x04
//#define PIC_ADDRESS   0x06
//#define PIC_ADDRESS   0x08


#define Forward      0x00
#define Backward     0x01

#define SlowSpeed    40
#define MediumSpeed  90
#define FastSpeed    130
#define Stop         00

#include <ArduinoHardware.h>
#include <Wire.h>
#include <TimerOne.h>

void ReadOne(char address);

int SerialInput = 0;    // This variable is used to stored the value of the keyboard button that was press
                                    // if SerialInput = 119 , W , that mean go forward
                                    
int RobotSpeed = FastSpeed;
int COUNTS = 0;         // This will contain the number of counts per loop. NOT the total distance
int PID    = 0;
int ACC_ERR= 0;
int i2cDirection = 0, i2cSpeed = 0;   // for incoming serial data
int ODOM1 = 0, ODOM2 = 0, ODOM3 = 0, ODOM4 = 0;

void setup(){
  Wire.begin(); // join i2c bus
  Serial.begin(128000);
  pinMode(13, OUTPUT);
  
  while (1) {
    if (Serial.available() > 0){
      SerialInput = Serial.read();      // We don't care what button was pressed
      Serial.print("Target =    "); Serial.println(RobotSpeed); Serial.println("");
      i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
      break;
    }
  }
}

void loop()
{
  for (int f = 0; f <= 50; f++){
    Serial.println("I'm out of the loop");
    ReadOne(PIC_ADDRESS);
    Serial.print("PID =    "); Serial.print(PID); Serial.print("COUNTS :      "); Serial.println(COUNTS);
    delay(110);
  }
  while (1){
    if (Serial.available() > 0){
    SerialInput = Serial.read();     
    break;
    }
  }
}

void i2cWrite ( char Address, char Speed , char Direction)
{
    Wire.beginTransmission(Address >> 1 );
    Wire.write((byte)0);
    Wire.write((byte)Speed);
    Wire.write((byte)Direction);
    Wire.endTransmission();
}

void ReadOne(char address) {                               // pass in the motor you want to read
  unsigned int encoder[2] = {0,0};
  Wire.requestFrom(address, 2);    // request 2 bytes from address
   int i = 0;
  while(Wire.available())   // slave may send less than requested
  { 
    encoder[i] = Wire.read();   // receive a byte as character
    i++;
  }
  PID     = encoder[0];
  COUNTS  = encoder[1];
}


