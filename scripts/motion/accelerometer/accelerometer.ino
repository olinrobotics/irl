

/*
This code is for the Arbotix-M accelerometer. It is designed 
to be powered and grounded by the Arbotix-M, and has its Xo, Yo, 
and Zo pins connected to data pins on the Arbotix-M. This program 
will then read the values and then output a boolean when the
accelerometer detects that there is a sudden surge in movement,
i.e. when I tap it. It must be able to recognize tapping
even when the accelerator moves at a constant speed.
*/


#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher accel("accel", &str_msg);


//Pin A5 is the Z output from the accelerometer, Pin A4 is the Y 
// output, and Pin A3 is the X output.
int pointX = 3;
int pointY = 4;
int pointZ = 5;


//Boolean
boolean big_change = true;

//Positions
int current_x = 0;
int current_y = 0;
int current_z = 0;

int old_x = 0;
int old_y = 0;
int old_z = 0;

//Differences in positions
int diff_z = 0;
int diff_y = 0;
int diff_x = 0;

//Increment
int i = 0;

void setup(){
  
  nh.initNode();
  nh.advertise(accel);
  
  
  pinMode(pointZ, INPUT);
  pinMode(pointY, INPUT);
  pinMode(pointX, INPUT);
     
  Serial.begin(9600);
  
  
}  

void loop(){
  if(i == 0){
  
    old_x = analogRead(pointX);
    old_y = analogRead(pointY);
    old_z = analogRead(pointZ); 
    i++;
  
  }
  else{
    current_x = analogRead(pointX);
    current_y = analogRead(pointY);
    current_z = analogRead(pointZ);
    
    diff_z = current_z - old_z;
    diff_y = current_y - old_y;
    diff_x = current_x - old_x;
  
    if((abs(diff_z) > 20) || (abs(diff_y) > 20) || (abs(diff_x) > 20)){
      Serial.println(big_change);
      str_msg.data = "true";
      accel.publish( &str_msg );
      nh.spinOnce();
      
    }
    
    old_z = current_z;
    old_y = current_y;
    old_x = current_x;
  
  }  
  
  
  /*
  Serial.print("X = ");
  Serial.println(x);
  Serial.print("Y = ");
  Serial.println(y);
  Serial.print("Z = ");
  Serial.println(z);
  Serial.println(" ");
 */

}  
  
