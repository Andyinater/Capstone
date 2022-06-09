#include <Servo.h>
#include "AndrewsBot.h"
Servo servoLeft;
Servo servoRight;


// Use this script to test values in the table and fill out

double setVel = 100;

void setup() {
  // put your setup code here, to run once:
  // prepareServos(servoLeft, servoRight);
//  for(int i = 0; i < 4; i++){
//    delay(1000);
//    turn(servoLeft, servoRight, 360, 0);
//  }
  turn(servoLeft, servoRight, 180, 100);
  delay(1000);
  turn(servoLeft, servoRight, 90, 100);

//  delay(1000);
//  turn(servoLeft, servoRight, 180, 0);
//  delay(1000);
//  turn(servoLeft, servoRight, 360, 0);
  //goForward(servoLeft, servoRight, setVel, 10000);
  
  
  


}

void loop() {
  // put your main code here, to run repeatedly:
  
  
}
