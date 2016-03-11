

/*
This code is for the Arbotix-M accelerometer. It is designed 
to be powered and grounded by the Arbotix-M, and has its Xo, Yo, 
and Zo pins connected to data pins on the Arbotix-M. This program 
will then read the values and then output a boolean when the
accelerometer detects that there is a sudden surge in movement,
i.e. when I tap it. It must be able to recognize tapping
even when the accelerator moves at a constant speed.
It also utilizes RosSerial as a Publisher to tell Edwin whether it moves or was touched.
*/


#include <ros.h>
#include <std_msgs/String.h>
#include <math.h>

ros::NodeHandle edwin_head;

std_msgs::String str_msg;
ros::Publisher accel("edwin_imu", &str_msg);


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

//Differences in positions
int diff_z = 0;
int diff_y = 0;
int diff_x = 0;

//Records of the last five numbers
int old5_x = 0;
int old5_y = 0;
int old5_z = 0;

int old4_x = 0;
int old4_y = 0;
int old4_z = 0;

int old3_x = 0;
int old3_y = 0;
int old3_z = 0;

int old2_x = 0;
int old2_y = 0;
int old2_z = 0;

int old1_x = 0;
int old1_y = 0;
int old1_z = 0;


// The average

int last_avg_x = 0;
int last_avg_y = 0;
int last_avg_z = 0;

//Increment for initialization of first data set
int i = 0;

void setup(){
  edwin_head.getHardware() -> setBaud(9600);
  edwin_head.initNode();
  edwin_head.advertise(accel);
  
  
  pinMode(pointZ, INPUT);
  pinMode(pointY, INPUT);
  pinMode(pointX, INPUT);
     
  Serial.begin(9600);
  
  
}  

void loop(){

    
  
  if(i < 6){
  
    if(i == 1){
      
      old1_x = analogRead(pointX);
      old1_y = analogRead(pointY);
      old1_z = analogRead(pointZ);
    }
    else if(i == 2){
      
       old2_x = analogRead(pointX);
       old2_y = analogRead(pointY);
       old2_z = analogRead(pointZ);
    }    
    else if(i == 3){
      
       old3_x = analogRead(pointX);
       old3_y = analogRead(pointY);
       old3_z = analogRead(pointZ);
    }
    else if(i == 4){
      
       old4_x = analogRead(pointX);
       old4_y = analogRead(pointY);
       old4_z = analogRead(pointZ);
    }
    else if(i == 5){
      
       old5_x = analogRead(pointX);
       old5_y = analogRead(pointY);
       old5_z = analogRead(pointZ);
    }    
    i++;
  
  }
  else{  

    
      
    current_x = analogRead(pointX);
    current_y = analogRead(pointY);
    current_z = analogRead(pointZ);
    
    last_avg_x = (old1_x + old2_x + old3_x + old4_x + old5_x)/5.0;
    last_avg_y = (old1_y + old2_y + old3_y + old4_y + old5_y)/5.0;
    last_avg_z = (old1_z + old2_z + old3_z + old4_z + old5_z)/5.0;
    
    diff_z = current_z - last_avg_z;
    diff_y = current_y - last_avg_y;
    diff_x = current_x - last_avg_x;
  
    if((abs(diff_z) <= 70 && abs(diff_z) > 25) || (abs(diff_y) <= 70 && abs(diff_y) > 25) ||
    (abs(diff_x) <= 70 && abs(diff_x) > 25)){
     
      str_msg.data = "IMU: pat";
      accel.publish( &str_msg );
      old5_x = 0;
      old5_y = 0;
      old5_z = 0;
      
      old4_x = 0;
      old4_y = 0;
      old4_z = 0;
      
      old3_x = 0;
      old3_y = 0;
      old3_z = 0;
      
      old2_x = 0;
      old2_y = 0;
      old2_z = 0;
      
      old1_x = 0;
      old1_y = 0;
      old1_z = 0;
      
      i = 0;
      delay(2000);

      
    }
    else if((abs(diff_z) > 70) || (abs(diff_y) > 70) || (abs(diff_x) > 70)){
     
      str_msg.data = "IMU: slap";
      accel.publish( &str_msg );
      old5_x = 0;
      old5_y = 0;
      old5_z = 0;
      
      old4_x = 0;
      old4_y = 0;
      old4_z = 0;
      
      old3_x = 0;
      old3_y = 0;
      old3_z = 0;
      
      old2_x = 0;
      old2_y = 0;
      old2_z = 0;
      
      old1_x = 0;
      old1_y = 0;
      old1_z = 0;
      
      i = 0;
      
      delay(2000);

      
    }
    else{
      str_msg.data = "IMU: notouch";
      accel.publish( &str_msg );

    }
    
    
    edwin_head.spinOnce();
    delay(200);


    
    old1_z = old2_z;
    old1_y = old2_y;
    old1_x = old2_x;
    
    old2_z = old3_z;
    old2_y = old3_y;
    old2_x = old3_x;
    
    old3_z = old4_z;
    old3_y = old4_y;
    old3_x = old4_x;
    
    old4_z = old5_z;
    old4_y = old5_y;
    old4_x = old5_x;
    
    old5_z = current_z;
    old5_y = current_y;
    old5_x = current_x;
  
  }  
  
  
  
  

}  
  
