
/*

		NAME 	 - Arduino Control Bridge 
		DESIGN - Josh Galicic
		PRIORITY - Required - Neccessary for communicating between PC and Pics.
	
\
*/

#define Front_Left   0x08
#define Front_Right  0x02
#define Back_Right   0x04
#define Back_Left    0x06


#define Forward      0x00
#define Backward     0x01

#define SlowSpeed    30
#define MediumSpeed  50
#define FastSpeed    60
#define Stop         00

#include <ArduinoHardware.h>
#include <Wire.h>
#include <TimerOne.h>

int ReadOne(char address);

int SerialInput = 0;    // This variable is used to stored the value of the keyboard button that was press
                                    // if SerialInput = 119 , W , that mean go forward
                                    // if SerialInput = 115 , S , that mean go backward
                                    // if SerialInput = 97  , A , that mean strafe left
                                    // if SerialInput = 100 , D , that mean strafe right
                                    // if SerialInput = 113 , Q , that mean left diagonal forward
                                    // if SerialInput = 101 , E , that mean right diagonal forward
                                    // if SerialInput = 114 , R , that mean rotate left
                                    // if SerialInput = 116 , T , that mean rotate right
                                    // if SerialInput = 120 , X , that mean STOP!!
                                    // if SerialInput = 49  , 1 , that mean SlowSpeed
                                    // if SerialInput = 50  , 2 , that mean MediumSpeed
                                    // if SerialInput = 51  , 3 , that mean FastSpeed
                                    // if SerialInput = 99  , C , that mean RECORD ODEMETRY
                                    
int RobotSpeed = MediumSpeed;
int COUNTS = 0;         // This will contain the number of counts per loop. NOT the total distance
int i2cDirection = 0, i2cSpeed = 0;   // for incoming serial data
int ODOM1 = 0, ODOM2 = 0, ODOM3 = 0, ODOM4 = 0;

void setup(){
  Wire.begin(); // join i2c bus
  Serial.begin(4800);
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
      i2cWrite(Front_Left,   RobotSpeed,   Forward  );
      i2cWrite(Front_Right,  RobotSpeed,   Forward  );
      i2cWrite(Back_Left,    RobotSpeed,   Forward  );
      i2cWrite(Back_Right,   RobotSpeed,   Forward  );
      
    }
    else if (SerialInput == 115)               // S was pressed. Go backward
    {
      Serial.print ("You just pressed: S      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   RobotSpeed,   Backward  );
      i2cWrite(Front_Right,  RobotSpeed,   Backward  );
      i2cWrite(Back_Left,    RobotSpeed,   Backward  );
      i2cWrite(Back_Right,   RobotSpeed,   Backward  );
    }
    else if (SerialInput == 97)               // A was pressed. Go left
    {
      Serial.print ("You just pressed: A      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   RobotSpeed,   Backward  );
      i2cWrite(Front_Right,  RobotSpeed,   Forward   );
      i2cWrite(Back_Left,    RobotSpeed,   Forward   );
      i2cWrite(Back_Right,   RobotSpeed,   Backward  );
    }
    else if (SerialInput == 100)               // D was pressed. Go right
    {
      Serial.print ("You just pressed: D      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   RobotSpeed,   Forward   );
      i2cWrite(Front_Right,  RobotSpeed,   Backward  );
      i2cWrite(Back_Left,    RobotSpeed,   Backward  );
      i2cWrite(Back_Right,   RobotSpeed,   Forward   );
    }
    else if (SerialInput == 113)               // Q was pressed. Left Diagonal!
    {
      Serial.print ("You just pressed: Q      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   Stop,   Forward   );
      i2cWrite(Front_Right,  RobotSpeed,   Forward  );
      i2cWrite(Back_Left,    RobotSpeed,   Forward  );
      i2cWrite(Back_Right,   Stop,   Forward   );
    }
    else if (SerialInput == 101)               // E was pressed. Right Diagonal!
    {
      Serial.print ("You just pressed: E      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   RobotSpeed,   Forward   );
      i2cWrite(Front_Right,  Stop,   Forward  );
      i2cWrite(Back_Left,    Stop,   Forward  );
      i2cWrite(Back_Right,   RobotSpeed,   Forward   );
    }
    else if (SerialInput == 114)               // R was pressed. Rotate Left!
    {
      Serial.print ("You just pressed: R      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   RobotSpeed,   Backward   );
      i2cWrite(Front_Right,  RobotSpeed,   Forward    );
      i2cWrite(Back_Left,    RobotSpeed,   Backward   );
      i2cWrite(Back_Right,   RobotSpeed,   Forward    );
    }
    else if (SerialInput == 116)               // T was pressed. Rotate Right!
    {
      Serial.print ("You just pressed: Q      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   RobotSpeed,   Forward   );
      i2cWrite(Front_Right,  RobotSpeed,   Backward  );
      i2cWrite(Back_Left,    RobotSpeed,   Forward   );
      i2cWrite(Back_Right,   RobotSpeed,   Backward  );
    }
    else if (SerialInput == 120)               // X was pressed. STOP!
    {
      Serial.print ("You just pressed: X      Value of SerialInput is   "); Serial.println(SerialInput);        //For debugging
      i2cWrite(Front_Left,   Stop,   Forward   );
      i2cWrite(Front_Right,  Stop,   Forward  );
      i2cWrite(Back_Left,    Stop,   Forward  );
      i2cWrite(Back_Right,   Stop,   Forward   );
    }
    else if (SerialInput == 49)              // 1 was pressed, SLOW
    {
      RobotSpeed = SlowSpeed;
    }
    else if (SerialInput == 50)              // 2 was pressed, MEDIUM
    {
      RobotSpeed = MediumSpeed;
    }
    else if (SerialInput == 51)              // 3 was pressed, FAST
    {
      RobotSpeed = FastSpeed;
    }
   else if (SerialInput == 99)
   {
     ODOM1 = ReadOne (Front_Left);
     ODOM2 = ReadOne (Front_Right);
     ODOM3 = ReadOne (Back_Right);
     ODOM4 = ReadOne (Back_Left);
     Serial.print("ODOM1:    "); Serial.print(ODOM1); Serial.print("              ODOM2:    "); Serial.println(ODOM2);
     Serial.print("ODOM3:    "); Serial.print(ODOM3); Serial.print("              ODOM4:    "); Serial.println(ODOM4);
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
  Wire.requestFrom(address >>1 , 2);    // request 2 bytes from address
   int i = 0;
  while(Wire.available())   // slave may send less than requested
  { 
    encoder[i] = Wire.read();   // receive a byte as character
    i++;
  }
  
  encoder[1] = encoder[1] << 8; // Combine the two bytes into one value, lower byte is sent first, upper second.
  
  return encoder[1] + encoder[0];
}


