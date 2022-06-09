#include <ESP32Servo.h>

// pins 25 and 26 seem to be available on logger for pwm operation

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int servoPin = 25;
//int servoPin = 26;

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
//  myservo.setTimerWidth(20);
  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

void loop() {

  //  myservo.writeMicroseconds(1500);

  delay(3000);
  myservo.write(0);
  delay(1000);
  myservo.write(10);
  delay(500);
  myservo.write(20);
  delay(3000);

  myservo.detach();
  myservo.attach(servoPin, 1000, 2000);

  myservo.write(20);
  delay(1000);
  myservo.write(30);
  delay(500);
  myservo.write(40);


}
