/********************************************
* Author: Quang Nguyen 
* Date  : 10/27/2013
*
* Direction: 
*  This program was written to be used with PLX-DAQ for tuning the PID constants.
*  Inorder to use this program, you must first set the value of KP_CURRENT and 
*  KI_CURRENT to the current value of KP and KI that is being used in the PIC. 
*  You should then input the desirable testing values of KP and KI in 
*  KP_TARGET and KI_TARGET. The program will test and graph the system 
*  at each of the value for KP and KI. 
*  Upon given power to the Arduino, you have 4 seconds to connect to the 
*  PLX-DAQ and to turn on the robot. Assuming that you have done everything correctly,
*  your excel file should be populated with incoming data from the arduino. This is the
*  result of your PID. You can graph these data points to better understand the reaction
*  that KP and KI have on the system. 
*
*********************************************/
//#define PIC_ADDRESS   0x02
#define PIC_ADDRESS   0x04
//#define PIC_ADDRESS   0x06
//#define PIC_ADDRESS   0x08


#define Forward      0x00
#define Backward     0x01

#define SlowSpeed    20
#define MediumSpeed  50
#define FastSpeed    60
#define Stop         00
#define TIME_LOOP    60

#include <ArduinoHardware.h>
#include <Wire.h>
#include <TimerOne.h>

void ReadOne(char address);
void CLEAR(char address);
void CLEAR_ACC_ERROR (char address);
void KP_UPDATE (char address, double TARGET);
void KI_UPDATE (char address, double TARGET);
void KP_UPDATE (char address, double TARGET);
void EXCEL_PRINT(char address);

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
  
  delay(4000);      
//  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
//  Serial.println("CLEARDATA"); Serial.println("LABEL,TIME, KP, KI, LOOP,COUNTS,PID,ACC_ERR");
}

void loop()
{ 

//////////////// Graph TARGET [0] //////////  
//  KP_UPDATE(PIC_ADDRESS, KP_TARGET[0] );
//  KI_UPDATE(PIC_ADDRESS, KI_TARGET[0] );
//  KD_UPDATE(PIC_ADDRESS, KD_TARGET[0] );
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
  Serial.println("CLEARDATA"); Serial.println("LABEL,TIME, KP, KI, KD, LOOP,COUNTS,PID,ACC_ERR");
  for (int f = 0; f <= 35; f++){
    TIME = TIME_LOOP * f;
    EXCEL_PRINT(PIC_ADDRESS);
    delay(TIME_LOOP);
  }
  CLEAR(PIC_ADDRESS);
////////////////// Graph TARGET [1] /////////
  KP_UPDATE(PIC_ADDRESS, KP_TARGET[1] );
  KI_UPDATE(PIC_ADDRESS, KI_TARGET[1] );
  KD_UPDATE(PIC_ADDRESS, KD_TARGET[1] );
  
    Serial.print("DATA,TIME,");                     // Indicate the start of a new set of data
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.println(",");
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );  
  for (int f = 0; f <= 35; f++){
    TIME = TIME_LOOP * f;
    EXCEL_PRINT(PIC_ADDRESS);
    delay(TIME_LOOP);
  }
  CLEAR(PIC_ADDRESS);
  
  ////////////////// Graph TARGET [2] /////////
  KP_UPDATE(PIC_ADDRESS, KP_TARGET[2] );
  KI_UPDATE(PIC_ADDRESS, KI_TARGET[2] );
  KD_UPDATE(PIC_ADDRESS, KD_TARGET[2] );
  
    Serial.print("DATA,TIME,");                     // Indicate the start of a new set of data
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.print(","); 
    Serial.print(0); Serial.println(",");
  i2cWrite(PIC_ADDRESS,   RobotSpeed,   Forward  );
  for (int f = 0; f <= 35; f++){
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
  int temp = 0;
  if (TARGET > KP_CURRENT)
    temp = round((TARGET - KP_CURRENT)*10.0);
  else if (TARGET < KP_CURRENT)
    temp = round((KP_CURRENT - TARGET)*10.0);
  else 
    temp = 0;
    
  for (int j = 0; j < temp; j++)
  {
    if (KP_CURRENT < TARGET){
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
    delay(100);
  }
}

void KI_UPDATE(char address, double TARGET)
{
  int temp = 0;
  if (TARGET > KI_CURRENT)
    temp = round((TARGET - KI_CURRENT)*10.0);
  else if (TARGET < KI_CURRENT)
    temp = round((KI_CURRENT - TARGET)*10.0);
  else 
    temp = 0;
    
  for (int j = 0; j < temp; j++)
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

void KD_UPDATE(char address, double TARGET)
{
  int temp = 0;
  if (TARGET > KD_CURRENT)
    temp = round((TARGET - KD_CURRENT)*10.0);
  else if (TARGET < KD_CURRENT)
    temp = round((KD_CURRENT - TARGET)*10.0);
  else 
    temp = 0;
    
  for (int j = 0; j < temp; j++)
  {
    if (KD_CURRENT < TARGET){
    Wire.beginTransmission(address >> 1 );
    Wire.write((byte)5);
    Wire.write((byte)0);
    Wire.write((byte)0);
    Wire.endTransmission();
      KD_CURRENT += 0.1;
    }
    if (KD_CURRENT > TARGET) {
    Wire.beginTransmission(address >> 1 );
    Wire.write((byte)6);
    Wire.write((byte)0);
    Wire.write((byte)0);
    Wire.endTransmission();
      KD_CURRENT -= 0.1;
    }
  }
}

void CLEAR(char address)
{
  i2cWrite(address,   0,   Forward  );
  delay(2000);
  CLEAR_ACC_ERROR (address);
}
void CLEAR_ACC_ERROR (char address)
{
  Wire.beginTransmission(address >> 1 ); // Clear ACC_ERR
  Wire.write((byte)7);
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
    Serial.print(KD_CURRENT); Serial.print(",");
    Serial.print(TIME)      ; Serial.print(","); 
    Serial.print(COUNTS)    ; Serial.print(","); 
    Serial.print(PID)       ; Serial.print(","); 
    Serial.print(ACC_ERR)   ; Serial.println(","); 
}
