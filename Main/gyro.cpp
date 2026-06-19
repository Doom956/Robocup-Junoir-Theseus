#include "gyro.h"
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern Adafruit_BNO055 bno;

gyro::gyro(){
  
}
void gyro::init_Gyro(){
  if(!bno.begin()){
    Serial.println("can't find gyro");
  }
  else{
    Serial.println("gyro found");
  }
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
  sensors_event_t event; bno.getEvent(&event);
  float heading = (double)event.orientation.x;
  if (abs(360-heading)< 5||abs(heading)<5) heading = 0; // wraparound
  return heading;
}
double gyro::yaw_heading(){
  sensors_event_t event; bno.getEvent(&event);
  float yaw_heading = (double)event.orientation.z;
  if (abs(360-yaw_heading)< 5||abs(yaw_heading)<5) yaw_heading = 0;
  return yaw_heading;
}
double gyro::get_acceleration(){
  sensors_event_t event; bno.getEvent(&event,  Adafruit_BNO055::VECTOR_LINEARACCEL);
  double accel = (double) event.acceleration.x;
  return accel;
}
double gyro::get_filtered_acceleration(){
  const double alpha = 0.15;  // 0..1, lower = smoother
  double raw = get_acceleration();

  if (!accelFilterInitialized) {
    accelFiltered = raw;
    accelFilterInitialized = true;
  } else {
    accelFiltered = alpha * raw + (1.0 - alpha) * accelFiltered;
  }
  return accelFiltered;
}

void gyro::reset_accel_filter(){
  accelFilterInitialized = false;
  accelFiltered = 0.0;
}
double gyro::get_velocity(){
  
  double a = get_filtered_acceleration();

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;   // seconds
  lastTime = now;
  if(abs(a)<0.02) a = 0.0; 
  v += a * dt;
  return v;
  
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