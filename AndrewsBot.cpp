#include "AndrewsBot.h"
#include <Servo.h>
double accelRate = 0.01;
int midVal = 1500;
double batteryMod = 1.01;

int surfaceType = 1; // 0 = desk top, 1 = lab floor

double getAngleMod(double angle){
  angle = abs(angle);
  double aMod;
  if (surfaceType == 1){
    if(0 < angle <= 180){
      aMod = map(angle, 0, 180, 8, 7.1);
    } else if( 180 < angle <= 360){
      aMod = map(angle, 180, 360, 6.4, 6.2);
    }
  }

  return aMod*batteryMod;
}


void turn(Servo sLeft, Servo sRight, double angle, double radius){
  // right is positive
  sLeft.attach(13);
  sRight.attach(12);
  int startTime = millis();
  double carVel = 0;
  double minVal = midVal - 200;
  double maxVal = midVal + 200;

  double angleMod = getAngleMod(angle);

  while(millis() - startTime < angleMod*abs(angle)){
    if (angle < 0){ // left hand turn
      carVel -= accelRate;
      if (midVal + carVel > minVal){
        sLeft.writeMicroseconds(midVal + carVel);
      } else{
        sLeft.writeMicroseconds(minVal);
      }
  
      if (midVal + carVel > minVal){
        sRight.writeMicroseconds(midVal + carVel);
      } else{
        sRight.writeMicroseconds(minVal);
      }

      
    } else{  // right hand turn
      carVel += accelRate;
      int leftSpeed = midVal + carVel + radius;
      int rightSpeed = midVal + carVel - radius;
      if (leftSpeed < maxVal){
        sLeft.writeMicroseconds(leftSpeed);
      } else{
        sLeft.writeMicroseconds(maxVal);
      }
  
      if (rightSpeed < maxVal){
        sRight.writeMicroseconds(rightSpeed);
      } else{
        sRight.writeMicroseconds(maxVal - 2* radius);
      }
    }
  }

  sLeft.detach();
  sRight.detach();
  
}
void goForward(Servo sLeft, Servo sRight, double setVel, int t){
  int startTime = millis();
  sLeft.attach(13);
  sRight.attach(12);
  double minVal = midVal - setVel;
  double maxVal = midVal + setVel;
  double carVel = 0;
  while(millis()-startTime < t){
  
    carVel += accelRate;
    
    
    if(midVal + carVel < maxVal){
      sLeft.writeMicroseconds(int(midVal + carVel));
    } else{
      sLeft.writeMicroseconds(int(maxVal));
    }
    if(midVal - carVel > minVal){
      sRight.writeMicroseconds(int(midVal - carVel));
    } else{
      sRight.writeMicroseconds(int(minVal));
    }
    
    // delay(1);
    
  }

  sLeft.detach();
  sRight.detach();
}

void prepareServos(Servo sLeft, Servo sRight){
  sLeft.attach(13);
  sRight.attach(12);
  sLeft.writeMicroseconds(1500);
  sRight.writeMicroseconds(1500);
  tone(4, 3000, 1000);
  delay(3000);
  sLeft.detach();
  sRight.detach();
}
