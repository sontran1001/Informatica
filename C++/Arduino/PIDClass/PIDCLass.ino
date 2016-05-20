#include "clockwork.h"
#include "pidcontroller.h"  
#include "MeOrion.h"
#include <SoftwareSerial.h>

#define Kp 100 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 5 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define Ki 15
#define Time 100

#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line

MeLineFollower left_lineFinder(PORT_3);
MeLineFollower right_lineFinder(PORT_4);

MeDCMotor MotorL(M1);  
MeDCMotor MotorR(M2);

float input = 0;
float output = 0;
float setpoint = 0;
pidController pid(&input, &output, &setpoint, Kp, Ki, Kd,0.1);
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
  input = -readPosition();
  pid.setInput(&input);
  Serial.print("left: ");
  Serial.println(pid.getLeftMotorSpeed());
  Serial.print("right: ");
  Serial.println(pid.getRightMotorSpeed());
  clockwork.start();
  pid.pidCompute();
  pid.pidControl(rightBaseSpeed, leftBaseSpeed, -255, 255);
  pid.pidControl(rightBaseSpeed, leftBaseSpeed, -255, 255);
  MotorL.run(pid.getLeftMotorSpeed());
  MotorR.run(pid.getRightMotorSpeed());
  clockwork.stop();
}


float readPosition(){
  int left_sensorState = left_lineFinder.readSensors();
  int right_sensorState = right_lineFinder.readSensors();
  if((left_sensorState == S1_OUT_S2_OUT) && (right_sensorState == S1_OUT_S2_OUT))     return 0;
  else if((left_sensorState == S1_IN_S2_OUT) && (right_sensorState == S1_OUT_S2_OUT)) return  -2;
  else if((left_sensorState == S1_IN_S2_IN) && (right_sensorState == S1_OUT_S2_OUT))  return  -1.0;
  else if((left_sensorState == S1_OUT_S2_IN) && (right_sensorState == S1_OUT_S2_OUT))  return  -0.5;
  else if((left_sensorState == S1_OUT_S2_IN) && (right_sensorState == S1_IN_S2_OUT))  return  0;
  else if((left_sensorState == S1_OUT_S2_OUT) && (right_sensorState == S1_IN_S2_OUT))   return  0.5;
  else if((left_sensorState == S1_OUT_S2_OUT) && (right_sensorState == S1_IN_S2_IN))   return  1.0;
  else if((left_sensorState == S1_OUT_S2_OUT) && (right_sensorState == S1_OUT_S2_IN)) return  2;
  else return 0;
  
}
