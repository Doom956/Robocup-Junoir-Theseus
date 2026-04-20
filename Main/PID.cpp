
#include <Arduino.h>
#include "PID.h"

PID::PID(double _kp, double _ki, double _kd){
  
  kp = _kp; // public kp = inputted _kp
  ki = _ki;
  kd = _kd;
  previousTime = micros();

}
double PID::getPID(double _error){
  error = _error;
  currentTime = micros()-(end-start); // functions are slower! you need to use micros
  delta = (error-prevError)/(currentTime - previousTime);
  cumError += error;
  if (cumError >  1000.0) cumError =  1000.0;
  if (cumError < -1000.0) cumError = -1000.0;
  double output = kp*error + ki*cumError + kd*delta;
  previousTime = currentTime;
  prevError = error; // update previousTime and prevError
  return output;
  
}
void PID::pausePID(int on){
  if(on == 1) start = micros();
  if(on == 2) end = micros();
}

