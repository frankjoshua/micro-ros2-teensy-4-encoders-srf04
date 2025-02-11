#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
public:
  Motor(int motorNumber, int dir);
  void adjust(double delta);

private:
  double power = 0;
  int direction = 1;
  String motorCommand;
};

#endif