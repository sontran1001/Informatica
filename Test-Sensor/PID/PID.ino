  
#include "MeOrion.h"
#include <SoftwareSerial.h>
#include "SimpleTimer.h"

#define Kp 20 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line

MeLineFollower left_lineFinder(PORT_3);
MeLineFollower right_lineFinder(PORT_4);

MeDCMotor MotorL(M1);  
MeDCMotor MotorR(M2);

#define sample_time 100
SimpleTimer timer;
float lastError =0;
float error =0;
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); 
  // calibration code here
  timer.setInterval(50, control);
  
} 


void loop()
{
   timer.run();
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

void control(){

  float sum_pos = 0;
  float position = 0;
  for(int i = 0; i < 5; i++){
    sum_pos += (float)(readPosition());
    delay(1);
    }
  position = sum_pos/5;
  error =  -position;
//  sum_pos = 0;
  Serial.println(position);
  float motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  float rightMotorSpeed = rightBaseSpeed + (int)motorSpeed;
  float leftMotorSpeed = leftBaseSpeed - (int)motorSpeed;
  Serial.println(motorSpeed);
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  {
      MotorL.run(leftMotorSpeed);
      MotorR.run(rightMotorSpeed);
      Serial.println(leftMotorSpeed);
      Serial.println(rightMotorSpeed);
      Serial.println(" ");
  }
  }
