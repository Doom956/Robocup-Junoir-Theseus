
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
  bool climbed = false; // if climbing occured.
  bool upwards = false; // up/ down for elevation
  int cnt = 0; // tiles traversed while climbing.
  double difference = 0; // centering distance
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  PID climbPID(10,0,0.1); // pid for centering
  PID gyroPID(3.5,0,0.1);
  PID Scale_PID(0.007,0,0.0008); // pid for encoder 0.0008
  Serial.println("forwarding");
  // allow the camera RTOS thread to flag victims for this move
  obstacleright = false;
  obstacleleft = false;
  fwdActive = true;
  isVictim = false;
  victimPending = false;
  int init_pitch = myGyro.modulus((int)myGyro.pitch_heading());
  int init_yaw = turnNeededDeg(myGyro.headingToCardinal(myGyro.heading()));
  int front_left_current=measure(7); int front_right_current=measure(1);
  int front_left_last=measure(7); int front_right_last=measure(1);
  timer myTime;
  myTime.reset_delta_time();
  int front_left = measure(7);int front_right = measure(1);
  if(front_left<=MIN_DIST&&front_left!=-1&&!(front_right<=MIN_DIST&&front_right!=-1)){
      obstacleavoidance(1);
      drivetrain.fullstop();
      delay(50);
      return;
    }
    else if(front_right<=MIN_DIST&&front_right!=-1&&!(front_left<=MIN_DIST&&front_left!=-1)){
      obstacleavoidance(0);
      drivetrain.fullstop();
      delay(50);
      return;
    }
  while((climbtoggle==true||(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3<=pulses*1.2)&&black!=true){
    //Serial.println((drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    if(Pausemaze==true) {drivetrain.fullstop(); break;}
    // Service a camera victim flagged by the RTOS thread: stop, pause PID +
    // timer, identify + dispense, then resume. (claude version 6/16/2026)
    if(victimPending){
      drivetrain.fullstop(); // does not overide the thread
      climbPID.pausePID(1);
      gyroPID.pausePID(1);
      Scale_PID.pausePID(1);
      myTime.pause(1);
      while(victimPending == true){
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
      }
      gyroPID.pausePID(2);
      climbPID.pausePID(2);
      Scale_PID.pausePID(2);
      myTime.pause(2);
    }
    
    // color: detect black (stop + back off) and blue (swamp) tiles ahead.
    int color = read_color(); // also marks silver checkpoints internally
    if(color == 1){ // blue tile ahead
      bluetoggle = true;
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
    //difference = center();
    
    //Serial.println(center());
    //double adjustment = myPID.getPID(difference);
    double yaw = myGyro.heading()-init_yaw;
    if(yaw>180) yaw = yaw-360;
    if(yaw<-180) yaw+= 360;
    
    double adjustment = gyroPID.getPID(yaw);
    double Scale = Scale_PID.getPID(pulses-(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    
    // emergency stop
    
    front_left_current = measure(7);
    front_right_current = measure(1);
    
    if((front_left_current<=50&&front_left_current!=-1)&&(front_right_current<=50&&front_right_current!=-1)){
      Serial.println("stopping");
      // if the robot doesn't make it halfway across the tile, fwd failed.
      
      drivetrain.fullstop();
      delay(50);
      break;
    }
    
    // check pitch: if it is greater than 25, the robot is going up a slope, so the encoder is turned off.
    Serial.println(adjustment);
    if(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) > 20){
      Serial.println("climbing");
      int _encoderCountA = drivetrain.encoderCountA; // save values before ramp
      int _encoderCountB = drivetrain.encoderCountB;
      int _encoderCountD = drivetrain.encoderCountD;
      climbtoggle = true; // prevent outer loop from exiting on encoder count
      climbed = true;
      Serial.println(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch));
      if(myGyro.modulus(myGyro.pitch_heading())-init_pitch>20) upwards = true; // distinguish between moving up and moving down.
      //drivetrain.reset_encoderCount(true,true,true);
      while(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) > 20){
        // PID centering
        double yaw = myGyro.heading()-init_yaw;
        if(yaw>180) yaw = yaw-360;
        if(yaw<-180) yaw+= 360;
    
        double adjustment = climbPID.getPID(yaw);
        
        Serial.println("climbing");
        //Serial.println(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch));
        Serial.println(adjustment);
        // center during climbing
        if(upwards == true) drivetrain.drive(180-adjustment,180-adjustment,180+adjustment,180+adjustment);
        if(upwards == false) drivetrain.drive(120-adjustment,120-adjustment,120+adjustment,120+adjustment);
        //Serial.println((drivetrain.encoderCountD+drivetrain.encoderCountA+drivetrain.encoderCountB)/3);
        if((drivetrain.encoderCountD+drivetrain.encoderCountA+drivetrain.encoderCountB)/3 >= pulses/cos(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch)*(M_PI/180))){
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
    
    
    drivetrain.drive(constrain(Scale*(150+adjustment),20,150),constrain(Scale*(150+adjustment),20,150)*1.25,constrain(Scale*(150-adjustment),20,150)*1.25,constrain(Scale*(150-adjustment),20,150));
    //drivetrain.drive(150+adjustment,(150+adjustment)*1.25,(150-adjustment)*1.25,150+adjustment);
  }
  Serial.println("stop- end of fwd");
  // sometimes it barely makes it over the slope
  if(climbed == true){
    for(int i = 0; i<cnt;i++){
      Serial.println("adding ramp to map");
      markEdgeBothWays(x_pos, y_pos, currentDir);
      stepForward(currentDir, x_pos, y_pos);
      writeWallsToCurrentTile(0, 1, 0, 1);
      updateFullyExploredAt(x_pos, y_pos);
      // moving between floors
      // only elevate the first tile of a ramp.
      // transition to another map
      if(i==0){
        if(upwards==true) elevation(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor); // elevate
        else descend(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor);
      }
    }
    Serial.println("compensating");
    drivetrain.fw(200);
    delay(300);
  }
  
  fwdActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  //if(botchedfwd == false) drivetrain.reset_encoderCount(true,true,true);
  victimtoggle = false;
}
// absolute turning

void absoluteturn(double angle){
  // create PID instance.
  PID myPID(4.5,0,0.3);
  double MOTORSPEED = 0;
  double current_angle=myGyro.heading();
  bool fasterway = false;
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  // allow the camera RTOS thread to flag victims during the turn
  turnActive = true;
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
      if(victimPending){ // service camera victim mid-turn 
        drivetrain.fullstop();
        myPID.pausePID(1); myTimer.pause(1);
        while(victimPending==true){
          rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
        }
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
      
      drivetrain.turnright(constrain(MOTORSPEED,20,150));
    }
  }

  else if(myGyro.inverse(angle,fasterway)-current_angle<0) {
    while(true){
      if(Pausemaze==true) {drivetrain.fullstop(); break;}
      if(victimPending){ // service camera victim mid-turn (claude version 6/16/2026)
        drivetrain.fullstop();
        myPID.pausePID(1); myTimer.pause(1);
        while(victimPending==true){
          rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
        }
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
      
      drivetrain.turnleft(constrain(MOTORSPEED,20,150));
    }
  }
  victimtoggle = false;
  Serial.println("finished turning");
  turnActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true); // reset encoder counters.
}
// full stop function
