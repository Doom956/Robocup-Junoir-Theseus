
void init_drive(){
  drivetrain.init_drive();
  // initialize gyro
  myGyro.init_Gyro();
}


// full stop

void fwd(double dist){ // in mm
  double pulses = dist/(wheel_diameter*M_PI)*wheel_cpr*gear_ratio; // easier to make a variable.
  double pulses156 = pulses*1.25;
  bool black = false; // toggle for black tile
  bool climbtoggle = false; // toggle for climbing
  int cnt = 0; // tiles traversed while climbing.
  double difference = 0; // centering distance
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  double init_heading = myGyro.heading();
  PID myPID(0.30,0,0.2); // pid for centering
  PID gyroPID(1.8,0,0.1);
  PID Scale_PID(0.006,0,0.0008); // pid for encoder 0.0008
  Serial.println("forwarding");
  // allow the camera RTOS thread to flag victims for this move
  motionActive = true;
  isVictim = false;
  victimPending = false;
  int init_yaw = myGyro.modulus((int)myGyro.yaw_heading());
  int front_left_current=measure(7); int front_right_current=measure(1);
  int front_left_last=measure(7); int front_right_last=measure(1);
  timer myTime;
  myTime.reset_delta_time();
  while((climbtoggle==true||(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3<=pulses*1.1)&&black!=true){
    if(Pausemaze==true) {drivetrain.fullstop(); break;}
    // Service a camera victim flagged by the RTOS thread: stop, pause PID +
    // timer, identify + dispense, then resume. (claude version 6/16/2026)
    if(victimPending){
      myPID.pausePID(1);
      Scale_PID.pausePID(1);
      myTime.pause(1);
      serviceCameraVictim();
      myPID.pausePID(2);
      Scale_PID.pausePID(2);
      myTime.pause(2);
    }
    // color: detect black (stop + back off) and blue (swamp) tiles ahead.
    int color = read_color(); // also marks silver checkpoints internally
    if(color == 1){ // blue tile ahead
      bluetoggle = true;
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(BLUE);
    }
    else if(color == -1){ // black tile ahead -> stop, mark next tile, back off
      drivetrain.fullstop();
      delay(100);
      Serial.println("black");
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(BLACK);
      blacktoggle = true;
      while(drivetrain.encoderCountA >= 0 && drivetrain.encoderCountB >= 0 && drivetrain.encoderCountD >= 0){
        drivetrain.backward(200);
      }
      black = true;
    }
    // PID centering
    difference = center();
    
    //Serial.println(center());
    double adjustment = myPID.getPID(difference);
    double Scale = Scale_PID.getPID(pulses-(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    
    // emergency stop
    
    front_left_current = measure(7);
    front_right_current = measure(1);
    /*
    if((front_left_current<=50&&front_left_current!=-1)||(front_right_current<=50&&front_right_current!=-1)){
      Serial.println("stopping");
      drivetrain.fullstop();
      delay(50);
      break;
    }
    */
    // check yaw heading
    // if it is greater than 25, the robot is going up a slope, so the encoder is turned off.
     if(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 15){
      Serial.println("climbing");
      int _encoderCountA = drivetrain.encoderCountA; // save values before ramp
      int _encoderCountB = drivetrain.encoderCountB;
      int _encoderCountD = drivetrain.encoderCountD;
      climbtoggle = true; // prevent outer loop from exiting on encoder count
      Serial.println(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw));
      
      //drivetrain.reset_encoderCount(true,true,true);
      while(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 15){
        // PID centering
        double yaw = myGyro.heading();
        if(yaw>180) yaw = yaw-360;
        //Serial.println(center());
        double adjustment = gyroPID.getPID(yaw);
        Serial.println("climbing");
        Serial.println(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw));
        // center during climbing
        drivetrain.drive(150-adjustment,150-adjustment,150+adjustment,150+adjustment);
        //Serial.println((drivetrain.encoderCountD+drivetrain.encoderCountA+drivetrain.encoderCountB)/3);
        if((drivetrain.encoderCountD+drivetrain.encoderCountA+drivetrain.encoderCountB)/3 >= pulses/cos(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw)*(M_PI/180))){
          Serial.println("1 section of the ramp climbed");
          cnt++;
          drivetrain.reset_encoderCount(true,true,true);
        } // track tiles
      }
      // ramp crested: restore pre-ramp encoder values so outer loop finishes the tile
      drivetrain.set_encoderCountA(_encoderCountA);
      drivetrain.set_encoderCountB(_encoderCountB);
      drivetrain.set_encoderCountD(_encoderCountD);
      climbtoggle = false; // re-enable encoder-based exit in outer loop
    }
    //Serial.print("adjustment: ");
    //Serial.println(adjustment);
    drivetrain.drive(constrain(Scale*(150+adjustment),20,150),constrain(Scale*(150+adjustment),20,150)*1.25,constrain(Scale*(150-adjustment),20,150)*1.25,constrain(Scale*(150-adjustment),20,150));
    //drivetrain.drive(150+adjustment,(150+adjustment)*1.25,(150-adjustment)*1.25,150+adjustment);
  }
  // sometimes it barely makes it over the slope
  if(climbtoggle == true){
    for(int i = 0; i<cnt;i++){
      Serial.println("adding ramp to map");
      markEdgeBothWays(x_pos, y_pos, currentDir);
      stepForward(currentDir, x_pos, y_pos);
      writeWallsToCurrentTile(0, 1, 0, 1);
      updateFullyExploredAt(x_pos, y_pos);
    }
    Serial.println("compensating");
    drivetrain.fw(200);
    delay(300);
  }
  Serial.println("stop- end of fwd");
  motionActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true);

}
// absolute turning

void absoluteturn(double angle){
  // create PID instance.
  PID myPID(10,0,0.3);
  double MOTORSPEED = 0;
  double current_angle=myGyro.heading();
  bool fasterway = false;
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  // allow the camera RTOS thread to flag victims during the turn
  motionActive = true;
  isVictim = false;
  victimPending = false;
  if(abs(angle-current_angle)> abs(angle-(360-current_angle))){
    current_angle = myGyro.inverse(current_angle,true); // make sure the robot turns the least amount
    fasterway = true;
  } 
  double init_angle = current_angle;
  Serial.println("current angle");
  Serial.println(current_angle);
   // create timer to cut of turning
  timer myTimer;

  if(myGyro.inverse(angle,fasterway) - current_angle > 0){
    while(true){
      if(Pausemaze==true) {drivetrain.fullstop(); break;}
      if(victimPending){ // service camera victim mid-turn (claude version 6/16/2026)
        myPID.pausePID(1); myTimer.pause(1);
        serviceCameraVictim();
        myPID.pausePID(2); myTimer.pause(2);
      }
      i2cMutex.lock();
      motorB->run(BACKWARD);
      motorD->run(BACKWARD);
      i2cMutex.unlock();
      if(myGyro.inverse(angle,fasterway)-current_angle<=0 && current_angle < 190) break;
      
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break; // turning limit
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      
      MOTORSPEED = myPID.getPID(myGyro.inverse(angle,fasterway)-current_angle);
      
      drivetrain.turnright(constrain(MOTORSPEED,20,100));
    }
  }

  else if(myGyro.inverse(angle,fasterway)-current_angle<0) {
    while(true){
      if(Pausemaze==true) {drivetrain.fullstop(); break;}
      if(victimPending){ // service camera victim mid-turn (claude version 6/16/2026)
        myPID.pausePID(1); myTimer.pause(1);
        serviceCameraVictim();
        myPID.pausePID(2); myTimer.pause(2);
      }
      i2cMutex.lock();
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
      i2cMutex.unlock();
      if(myGyro.inverse(angle,fasterway)-current_angle>=0 && current_angle > 170) break;
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break;
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      MOTORSPEED = myPID.getPID(current_angle-myGyro.inverse(angle,fasterway));
      
      drivetrain.turnleft(constrain(MOTORSPEED,20,100));
    }
  }
  
  if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
  Serial.println("finished turning");
  motionActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true); // reset encoder counters.
}
// full stop function
