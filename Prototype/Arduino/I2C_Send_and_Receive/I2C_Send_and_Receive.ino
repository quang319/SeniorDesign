
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

#define PIC_Address 0x02

#include <ArduinoHardware.h>
#include <Wire.h>
#include <TimerOne.h>

//int ReadOne(char address);

int COUNTS = 0;         // This will contain the number of counts per loop. NOT the total distance
int i2cDirection = 0, i2cSpeed = 0;   // for incoming serial data

void setup(){
  Wire.begin(); // join i2c bus
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  
    while (1) {
    if (Serial.available() > 0) {                //PIC will be stuck in a while loop until a 1 is sent to
                // read the incoming byte:
      i2cDirection = Serial.read();
      i2cDirection -= 48;
                // say what you got:
      Serial.print("I received: ");
      Serial.println(i2cDirection, DEC);
      break;
    }
  }
  
//  delay(10000);
  Wire.beginTransmission(PIC_Address >> 1 );
  Wire.write((byte)0);
  Wire.write((byte)150);
  Wire.write((byte)i2cDirection);
  Wire.endTransmission();
}

void loop()
{
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    i2cSpeed = Serial.parseInt(); 
    // do it again:
    i2cDirection = Serial.parseInt();
    Serial.print("i2cSpeed:  "); Serial.print(i2cSpeed, DEC);  Serial.print("       i2cDirection:    ");
    Serial.println(i2cDirection, DEC); 
    Wire.beginTransmission(PIC_Address >> 1 );
    Wire.write((byte)0);
    Wire.write((byte)i2cSpeed);
    Wire.write((byte)i2cDirection);
    Wire.endTransmission();
  }
  delay(14);
  COUNTS = ReadOne(PIC_Address >> 1);
  Serial.print("COUNTS =      ");   Serial.print(COUNTS);  Serial.println(",");
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


