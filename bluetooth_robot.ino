/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    SlaveBluetoothBySoftSerialTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/09
 * @brief   Description: this file is sample code for Me bluetooth device.
 *          The bluetooth inherited the MeSerial class from SoftwareSerial.
 *          You also can use this sample code to reset the bluetooth. The
 *          AT command for the Me bluetooth can see 0n page 7 of
 *          doc\Me_Bluetooth\Datasheet\ELET114A_Datasheet_v1.5.5_CH.pdf
 *          You can use the command "AT+URATE=<baudrate> <CR> <LF>" to re-set 
 *          the baud rate.
 *
 * Function List:
 * 1. void MeBluetooth::begin()
 * 2. int16_t MeBluetooth::available()
 * 3. size_t MeSerial::write(uint8_t byte)
 * 4. int16_t MeSerial::read(void)
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2015/09/09    1.0.0          rebuild the old lib
 * </pre>
 */
 
#include "MeOrion.h"
#include <SoftwareSerial.h>

MeBluetooth bluetooth(PORT_5);

MeDCMotor motor3(M1);

MeDCMotor motor4(M2);
unsigned char command = 0;
uint8_t motorSpeed3 = 100;
uint8_t motorSpeed4 = 92;

void setup()
{
  Serial.begin(115200);
  bluetooth.begin(115200);    //The factory default baud rate is 115200
  Serial.println("Bluetooth Start!");
}

void loop()
{ 
  int readdata = 0,i = 0,count = 0;
  char outDat;
  if (bluetooth.available())
  {
    while((readdata = bluetooth.read()) != (int)-1)
    {
      command = readdata;
      delay(1);
    }
   
      Serial.write(command);
  }
  switch(command)
  {
  case('1'): 
    forward();
    break;
  case('2'):
    backward();
    break;
  case('3'):
    turnleft();
    break;
  case('4'):
  turnright();
  break;
  default:
    stop();
  }

}

void forward(){
  motor3.run(motorSpeed3);
  motor4.run(motorSpeed4);
  }

void backward(){
  motor3.run(-motorSpeed3);
  motor4.run(-motorSpeed4);
  }
void turnright(){
  motor3.run(motorSpeed3);
  motor4.stop();
  }

void turnleft(){
  motor3.stop();
  motor4.run(motorSpeed4);
  }

void stop(){
  motor3.stop();
  motor4.stop();
  }
