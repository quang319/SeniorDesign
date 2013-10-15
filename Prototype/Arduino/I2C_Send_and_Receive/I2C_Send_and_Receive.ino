
/*

		NAME 	 - Arduino Control Bridge 
		DESIGN - Josh Galicic
		PRIORITY - Required - Neccessary for communicating between PC and Pics.
	
		--------OVERVIEW--------
This code allow the arduino to subscribe to the i2cSend topic (as well as others) and send these
i2c requests to the specified pics. The i2c message type allows for 2 bytes of data and an address.
Filling these fields out will let the arduino handle the rest in the i2c callback.

This code also allows the arduino to request encoder data from the pics and send it back up on the
/encoderData topic. This happens on a timer1 interrupt (line 80) currently set to 2 seconds.
There is also code and hardware for the arduino to read the battery voltage and send it back to insure
that the battery is not drained too low.


		--------FUTURE WORK--------
the 2 second request rate for encoder data is waaaaaaaaaay too low to provide useful position data.
Ideally, we should be able to request this data every ~100ms. testing will have to be done to see
if this is possible (make sure the arduino can leave the interrupt before it occurs again).

The battery voltage code is not currently implemented, but it has been verified as working correctly.
It is connected to a voltage divider on the regulator board to limit it to the 5 volt range. Refine
the conversion factor from analog input to battery voltage, because it isnt super accurate, maybe
look into a better voltage and/or current sensor to can provide more usable data.

Sending code has been verified as stable, and should give no issues, the pin 13 led will go high
whenever the arduino is sending or receiving i2c data. If you every see that light being constantly on,
it means that something on the bus is probably holding the clk line low, blocking all communication.
This can happen when reprogramming the PICs, so probably just reset power to the boards and hit the reset
button on the arduino. the serial bridge will survive an arduino reset, so that can keep going without
issue. 

*/

#define Front_Left   0x02
#define Front_Right  0x08
#define Back_Left    0x06
#define Back_Right   0x05

#define Forward      0x01
#define Backward     0x00

#define SlowSpeed    100
#define MediumSpeed  175
#define FastSpeed    225
#define Stop         00

#include <ArduinoHardware.h>
#include <Wire.h>
#include <TimerOne.h>

int ReadOne(char address);

int SerialInput = 0;    // This variable is used to stored the value of the keyboard button that was press
                                    // if SerialInput = 119 , that mean go forward
                                    // if SerialInput = 115 , that mean go backward
                                    // if SerialInput = 97 , that mean strafe left
                                    // if SerialInput = 100 , that mean strafe right
                                    // if SerialInput = 120 , that mean STOP!!
int COUNTS = 0;         // This will contain the number of counts per loop. NOT the total distance
int i2cDirection = 0, i2cSpeed = 0;   // for incoming serial data

void setup(){
  Wire.begin(); // join i2c bus
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  Serial.print ("Press W to go forward.               "); Serial.println ("Press S to go backward");
  Serial.print ("Press A to strafe left.              "); Serial.println ("Press D to strafe right");
}

void loop()
{
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    SerialInput = Serial.read();
    if (SerialInput == 119)                    // W was pressed. Go forward
    {
      Serial.print ("You just pressed: W      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   MediumSpeed,   Forward  );
      i2cWrite(Front_Right,  MediumSpeed,   Forward  );
      i2cWrite(Back_Left,    MediumSpeed,   Forward  );
      i2cWrite(Back_Right,   MediumSpeed,   Forward  );
      
    }
    else if (SerialInput == 115)               // S was pressed. Go backward
    {
      Serial.print ("You just pressed: S      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   MediumSpeed,   Backward  );
      i2cWrite(Front_Right,  MediumSpeed,   Backward  );
      i2cWrite(Back_Left,    MediumSpeed,   Backward  );
      i2cWrite(Back_Right,   MediumSpeed,   Backward  );
    }
    else if (SerialInput == 97)               // A was pressed. Go left
    {
      Serial.print ("You just pressed: A      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   MediumSpeed,   Backward  );
      i2cWrite(Front_Right,  MediumSpeed,   Forward   );
      i2cWrite(Back_Left,    MediumSpeed,   Forward   );
      i2cWrite(Back_Right,   MediumSpeed,   Backward  );
    }
    else if (SerialInput == 100)               // D was pressed. Go right
    {
      Serial.print ("You just pressed: D      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   MediumSpeed,   Forward   );
      i2cWrite(Front_Right,  MediumSpeed,   Backward  );
      i2cWrite(Back_Left,    MediumSpeed,   Backward  );
      i2cWrite(Back_Right,   MediumSpeed,   Forward   );
    }
    else if (SerialInput == 120)               // X was pressed. STOP!
    {
      Serial.print ("You just pressed: X      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   Stop,   Forward   );
      i2cWrite(Front_Right,  Stop,   Forward  );
      i2cWrite(Back_Left,    Stop,   Forward  );
      i2cWrite(Back_Right,   Stop,   Forward   );
    }
    else {
      Serial.println ("Invalid Input!!");
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

int ReadOne(char address) {                               // pass in the motor you want to read
  unsigned int encoder[2] = {0,0};
  int encoderCountTotal = 0;
  Wire.requestFrom(address, 2);    // request 2 bytes from address
   int i = 0;
  while(Wire.available())   // slave may send less than requested
  { 
    encoder[i] = Wire.read();   // receive a byte as character
    i++;
  }
  
  encoder[1] = encoder[1] << 8; // Combine the two bytes into one value, lower byte is sent first, upper second.
  
  return encoder[1] + encoder[0];
}


