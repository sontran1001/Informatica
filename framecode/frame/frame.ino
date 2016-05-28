//***********************************************************
/**
 * \par Copyright (C), May 2016
 * @file    frame.ino
 * @author  SonTran, Viet Phuong Dao, Anup, Maci
 * @version V1.0.0
 * @date    2016/05/28
 * @brief   Description: this file is a frame code which collect all features for
 *          a simple mobile robot with two modes:
 *          1 - Automatic control
 *          2 - Controling by a smart phone via bluetooth
 */
#include "MeOrion.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>

#include "clockwork.h"
#include "finite_state_machine.h"
#include "pidcontroller.h"
#include "low_pass_1.h"


MeBluetooth bluetooth(PORT_5);
MeDCMotor motor3(M1);
MeDCMotor motor4(M2);

unsigned char command = 0;
uint8_t motorSpeed3 = 100;
uint8_t motorSpeed4 = 92;

// From PIDClass.ino
#define Kp 70 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 5 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define Ki 0
#define Time 50

#define rightMaxSpeed 250 // max speed of the robot
#define leftMaxSpeed 250 // max speed of the robot
#define rightBaseSpeed 200 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 200  // this is the speed at which the motors should spin when the robot is perfectly on the line

MeLineFollower left_lineFinder(PORT_3);
MeLineFollower right_lineFinder(PORT_4);

MeDCMotor MotorL(M1);  
MeDCMotor MotorR(M2);

float input = 0;
float output = 0;
float setpoint = 0;
pidController pid(input, output, setpoint, Kp, Ki, Kd,0.05);
Clockwork clockwork(Time);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  bluetooth.begin(115200);    //The factory default baud rate is 115200
  Serial.println("Bluetooth Start!");

}

//**************************************************************************
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
  case('5'):
    automaticFollow();
    break;
  default:
    stop();
  }

}

//**********************************************************************
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

  // functions for automatic control
  int readPosition(){
  static int old_pos = 0;
  int current_pos;
  int left_sensorState = left_lineFinder.readSensors();
  int right_sensorState = right_lineFinder.readSensors();
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_OUT && old_pos <0) current_pos = -3;
  if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_OUT_S2_OUT) current_pos =  -2;
  if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_OUT_S2_OUT) current_pos = -1;
  if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_IN_S2_OUT) current_pos = 0;
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_IN_S2_IN) current_pos = 1;
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_IN) current_pos = 2;
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_OUT_S2_OUT && old_pos >0) current_pos = 3;

  // impossible position => return the old position
  if(left_sensorState == S1_OUT_S2_OUT && right_sensorState == S1_IN_S2_OUT) current_pos =  old_pos;
  if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_OUT_S2_OUT) current_pos =  old_pos;
  if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_OUT_S2_IN) current_pos =  old_pos;
  if(left_sensorState == S1_OUT_S2_IN && right_sensorState == S1_IN_S2_IN) current_pos =  old_pos;  // posible but solving later
  if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_OUT_S2_IN) current_pos =  old_pos;
  if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_IN_S2_OUT) current_pos =  old_pos;
  if(left_sensorState == S1_IN_S2_OUT && right_sensorState == S1_IN_S2_IN) current_pos =  old_pos;
  if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_OUT_S2_IN) current_pos =  old_pos;
  if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_IN_S2_OUT) current_pos =  old_pos;  // posible but solving later
  if(left_sensorState == S1_IN_S2_IN && right_sensorState == S1_IN_S2_IN) current_pos =  old_pos;   // posible but solving later
  old_pos = current_pos;
  return current_pos;
}

void automaticFollow(){
  clockwork.start();
  for(int i = 0; i< 5; i++)
  {
      input += -(float)(readPosition());
      delay(1);
  }
  input = input/5;
  pid.setInput(input);
  pid.pidCompute();
  Serial.println(pid.pidCompute());
  pid.pidControl(rightBaseSpeed, leftBaseSpeed, -255, 255);
  MotorL.run(pid.getLeftMotorSpeed());
  MotorR.run(pid.getRightMotorSpeed());
  clockwork.stop();
}
