#include "gyro.h"
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <rtos.h>

extern Adafruit_BNO055 bno;
extern rtos::Mutex i2cMutex;

gyro::gyro(){
  
}
void gyro::init_Gyro(){
  i2cMutex.lock();
  bool found = bno.begin();
  i2cMutex.unlock();
  if(!found) Serial.println("can't find gyro");
  else        Serial.println("gyro found");
}
int gyro::inverse(int val, bool inv){
  if(inv == true) {
    if(val == 0) return val; 
    val = val - 360;
  }
  else{val = val%360;}
  return val;
}
int gyro::modulus(int val){
  if(val<0) return 360 - val;
  if(val> 180) {val = val - 360;}
  else{val = val%360;}
  return val;
}
double gyro::heading(){
  sensors_event_t event;
  i2cMutex.lock();
  bno.getEvent(&event);
  i2cMutex.unlock();
  float heading = (double)event.orientation.x;
  if (abs(360-heading)< 5||abs(heading)<5) heading = 0; // wraparound
  return heading;
}
double gyro::yaw_heading(){
  sensors_event_t event;
  i2cMutex.lock();
  bno.getEvent(&event);
  i2cMutex.unlock();
  float yaw_heading = (double)event.orientation.z;
  if (abs(360-yaw_heading)< 5||abs(yaw_heading)<5) yaw_heading = 0;
  return yaw_heading;
}
void gyro::reset_accel_filter(){
  accelFilterInitialized = false;
  accelFiltered = 0.0;
}
double gyro::opposite_heading(double h) {
  h += 180.0;
  if (h >= 360.0) h -= 360.0;
  return h;
}
int gyro::headingToCardinal(double heading){
    // normalize angle
    if (heading < 0) heading += 360;
    if (heading >= 360) heading -= 360;

    if (heading >= 315 || heading < 45)
        return 0; // North
    else if (heading >= 45 && heading < 135)
        return 1; // East
    else if (heading >= 135 && heading < 225)
        return 2; // South
    else
        return 3; // West
}