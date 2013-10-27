/********************************************
* Author: Quang Nguyen 
*
*
*********************************************/
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
#define TIME_LOOP    110

#include <ArduinoHardware.h>
#include <Wire.h>
#include <TimerOne.h>

void ReadOne(char address);
void CLEAR(char address);
void KP_UPDATE (char address, double TARGET);
void KI_UPDATE (char address, double TARGET);
void EXCEL_PRINT(char address);

/************** PID VARIABLES **********************/
double KP_CURRENT = 0;
double KI_CURRENT = 0;

double KP_TARGET [3] = {0,0,0};
double KI_TARGET [5] = {0,0,0};

/****************************************************/

int SerialInput = 0;    // This variable is used to stored the value of the keyboard button that was press
                                    
int RobotSpeed = MediumSpeed;
int COUNTS = 0;         // This will contain the number of counts per loop. NOT the total distance
int PID    = 0;
int ACC_ERR= 0;
int i2cDirection = 0, i2cSpeed = 0;   // for incoming serial data
int TIME = 0;

void setup(){
  Wire.begin(); // join i2c bus
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  
  delay(4000);      
//  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
//  Serial.println("CLEARDATA"); Serial.println("LABEL,TIME, KP, KI, LOOP,COUNTS,PID,ACC_ERR");
}

void loop()
{ 

////////////////// Graph TARGET [0] //////////  
  KP_UPDATE(PIC_ADDRESS, KP_TARGET[0] );
  KI_UPDATE(PIC_ADDRESS, KI_TARGET[0] );
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
  Serial.println("CLEARDATA"); Serial.println("LABEL,TIME, KP, KI, LOOP,COUNTS,PID,ACC_ERR");
  for (int f = 0; f <= 39; f++){
    TIME = TIME_LOOP * f;
    EXCEL_PRINT(PIC_ADDRESS);
    delay(TIME_LOOP);
  }
  CLEAR(PIC_ADDRESS);
////////////////// Graph TARGET [1] /////////
  KP_UPDATE(PIC_ADDRESS, KP_TARGET[1] );
  KI_UPDATE(PIC_ADDRESS, KI_TARGET[1] );
  
    Serial.print("DATA,TIME,");                     // Indicate the start of a new set of data
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.println(",");
    
  for (int f = 0; f <= 39; f++){
    TIME = TIME_LOOP * f;
    EXCEL_PRINT(PIC_ADDRESS);
    delay(TIME_LOOP);
  }
  CLEAR(PIC_ADDRESS);
  
  ////////////////// Graph TARGET [2] /////////
  KP_UPDATE(PIC_ADDRESS, KP_TARGET[2] );
  KI_UPDATE(PIC_ADDRESS, KI_TARGET[2] );
  
    Serial.print("DATA,TIME,");                     // Indicate the start of a new set of data
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.println(",");
    
  for (int f = 0; f <= 39; f++){
    TIME = TIME_LOOP * f;
    EXCEL_PRINT(PIC_ADDRESS);
    delay(TIME_LOOP);
  }
  CLEAR(PIC_ADDRESS);
  
  while (1);          // Infinite loop
  
//      delay(4000);
//      i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
 //     Serial.println("CLEARDATA"); Serial.println("LABEL,TIME, LOOP,COUNTS,PID,ACC_ERR");
//      Serial.println("ROW,SET,2");
//      break;
//    }
//  }
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
  unsigned int encoder[4] = {0,0,0,0};
  Wire.requestFrom(PIC_ADDRESS >> 1, 4);    // request 2 bytes from address
  int i = 0;
  while(Wire.available())   // slave may send less than requested
  { 
    encoder[i] = Wire.read();   // receive a byte as character
    i++;
  }
  encoder[1] = encoder[1] << 8;
  ACC_ERR = encoder[1] + encoder[0];
  COUNTS  = encoder[2];
  PID     = encoder[3];
}

void KP_UPDATE(char address, double TARGET)
{
  while (TARGET != KP_CURRENT)
  {
    if (KI_CURRENT < TARGET){
    Wire.beginTransmission(address >> 1 );
    Wire.write((byte)1);
    Wire.write((byte)0);
    Wire.write((byte)0);
    Wire.endTransmission();
      KP_CURRENT += 0.1;
    }
    if (KP_CURRENT > TARGET) {
    Wire.beginTransmission(address >> 1 );
    Wire.write((byte)2);
    Wire.write((byte)0);
    Wire.write((byte)0);
    Wire.endTransmission();
      KP_CURRENT -= 0.1;
    }
  }
}

void KI_UPDATE(char address, double TARGET)
{
  while (TARGET != KI_CURRENT)
  {
    if (KI_CURRENT < TARGET){
    Wire.beginTransmission(address >> 1 );
    Wire.write((byte)3);
    Wire.write((byte)0);
    Wire.write((byte)0);
    Wire.endTransmission();
      KI_CURRENT += 0.1;
    }
    if (KI_CURRENT > TARGET) {
    Wire.beginTransmission(address >> 1 );
    Wire.write((byte)4);
    Wire.write((byte)0);
    Wire.write((byte)0);
    Wire.endTransmission();
      KI_CURRENT -= 0.1;
    }
  }
}

void CLEAR(char address)
{
  i2cWrite(address,   0,   Forward  );
  Wire.beginTransmission(address >> 1 ); // Clear ACC_ERR
  Wire.write((byte)5);
  Wire.write((byte)0);
  Wire.write((byte)0);
  Wire.endTransmission();
  delay(300);                            // Wait for motor to fully stop
  Wire.beginTransmission(address >> 1 ); // Clear ACC_ERR
  Wire.write((byte)5);
  Wire.write((byte)0);
  Wire.write((byte)0);
  Wire.endTransmission();
}
void EXCEL_PRINT(char address)
{
    ReadOne(address);
    Serial.print("DATA,TIME,");
    Serial.print(KP_CURRENT); Serial.print(",");
    Serial.print(KI_CURRENT); Serial.print(",");
    Serial.print(TIME)      ; Serial.print(","); 
    Serial.print(COUNTS)    ; Serial.print(","); 
    Serial.print(PID)       ; Serial.print(","); 
    Serial.print(ACC_ERR)   ; Serial.println(","); 
}
