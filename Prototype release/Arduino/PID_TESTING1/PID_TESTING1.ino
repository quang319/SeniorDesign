
//#define PIC_ADDRESS   0x02
#define PIC_ADDRESS   0x04
//#define PIC_ADDRESS   0x06
//#define PIC_ADDRESS   0x08


#define Forward      0x00
#define Backward     0x01

#define SlowSpeed    20
#define MediumSpeed  40
#define FastSpeed    60
#define Stop         00
#define TIME_LOOP    60

#include <ArduinoHardware.h>
#include <Wire.h>
#include <TimerOne.h>

int ReadOne(char address);

/************** PID VARIABLES **********************/
double KP_CURRENT = 3;
double KI_CURRENT = 1.8;
double KD_CURRENT = 2;

double KP_TARGET [3] = {3.5,3.5,3.5};
double KI_TARGET [3] = {1.8,1.7,1.7};
double KD_TARGET [3] = {2,2,2};

/****************************************************/

int SerialInput = 0;    // This variable is used to stored the value of the keyboard button that was press
                                    
int RobotSpeed = FastSpeed;
int COUNTS = 0;         // This will contain the number of counts per loop. NOT the total distance
int PID    = 0;
int ACC_ERR= 0;
int i2cDirection = 0, i2cSpeed = 0;   // for incoming serial data
int TIME = 0;

void setup(){
  Wire.begin(); // join i2c bus
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  delay(2000);      
}

void loop()
{ 
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
  RobotSpeed = FastSpeed;
  COUNTS = ReadOne(PIC_ADDRESS);
  Serial.print("COUNTS    "); Serial.println(COUNTS);
  delay(3000);
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
  RobotSpeed = SlowSpeed;
  COUNTS = ReadOne(PIC_ADDRESS);
  Serial.print("COUNTS    "); Serial.println(COUNTS);
  delay(3000);
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
  RobotSpeed = Stop;
  COUNTS = ReadOne (PIC_ADDRESS);
  Serial.print("COUNTS    "); Serial.println(COUNTS);
  delay(3000);
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Backward  );
  COUNTS = ReadOne (PIC_ADDRESS);
  Serial.print("COUNTS    "); Serial.println(COUNTS);
  while(1);
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

