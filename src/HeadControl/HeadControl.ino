//ROS stuff
#include <ros.h>
#include <std_msgs/Int8.h>

//Servo control
#include <Servo.h>

//Defining possible commands
#define GRIP 1

Servo gripperServo;
int gripperPin = 9;
int gripperPos = 0;

ros::NodeHandle edwin_head;
int cmd = 0;
const int ledPin = 13;
boolean light = false;

void grip_object(){
  for(gripperPos = 0; gripperPos <= 180; gripperPos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    gripperServo.write(gripperPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(gripperPos = 180; gripperPos>=0; gripperPos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    gripperServo.write(gripperPos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}

void callback( const std_msgs::Int8& cmd_raw){
    cmd = cmd_raw.data;
}

ros::Subscriber<std_msgs::Int8> cmd_sub("head_cmd", &callback);

void setup(){
  edwin_head.getHardware() -> setBaud(9600);
  edwin_head.initNode();
  edwin_head.subscribe(cmd_sub);
  
  //Set up gripper servo
  gripperServo.attach(gripperPin);
  
  //Set up status LED
  pinMode(ledPin, OUTPUT);
  //Blinks once so we know LED is working
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
}

void loop(){
  edwin_head.spinOnce();
  
  if (cmd != 0){
    digitalWrite(ledPin, HIGH);
    switch (cmd) {
      case GRIP:
        grip_object();
        break;        
    }    
    cmd = 0;
    digitalWrite(ledPin, LOW);
  }
  
  delay(1);
}  

