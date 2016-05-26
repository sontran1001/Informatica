#include "clockwork.h"
#include "pidcontroller.h"  
#include "MeOrion.h"
#include <SoftwareSerial.h>

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

void setup()
{
  // put your setup code here, to run once:
  delay(10);
  Serial.begin(9600);
  //pid.setLastError(0);
  // calibration code here
} 


void loop()
{

 //clockwork.start();
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
  //clockwork.stop();
}

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
