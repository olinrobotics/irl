

/*
This code is for the Arbotix-M accelerometer. It is designed 
to be powered and grounded by the Arbotix-M, and has its Xo, Yo, 
and Zo pins connected to data pins on the Arbotix-M. This program 
will then read the values and then output a boolean when the
accelerometer detects that there is a sudden surge in movement,
i.e. when I tap it. It must be able to recognize tapping
even when the accelerator moves at a constant speed.
*/

//Pin 0 is the Z output from the accelerometer, Pin 1 is the Y 
// output, and Pin 2 is the X output.
int pointZ = 1;
int pointY = 2;
int pointX = 3;



int test = 6;

//Positions
int x = 0;
int y = 0;
int z = 0;

void setup(){
  
  pinMode(pointZ, INPUT);
  pinMode(pointY, INPUT);
  pinMode(pointX, INPUT);
  
  pinMode(test,OUTPUT);
    
  Serial.begin(9600);
  
  
}  

void loop(){
  
  x = digitalRead(pointX);
  y = digitalRead(pointY);
  z = digitalRead(pointZ);
  
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  
  Serial.println(" ");
 

}  
  
