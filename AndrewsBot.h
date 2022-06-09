#ifndef ANDREWSBOT_H
#define ANDREWSBOT_H
#include <Servo.h>

#include <Arduino.h>

void goForward(Servo sLeft, Servo sRight, double carVel, int t);

void prepareServos(Servo sLeft, Servo sRight);

void turn(Servo sLeft, Servo sRight, double angle, double radius);

double getAngleMod(double angle);

#endif
