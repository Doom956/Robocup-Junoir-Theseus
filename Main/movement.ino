
void init_drive(){
  drivetrain.init_drive();
  // initialize gyro
  myGyro.init_Gyro();
}

// Service a camera victim that the RTOS thread flagged via victimPending.
// The caller (fwd/absoluteturn) has already paused its PID + timer. This stops
// the drivetrain, identifies the victim on the wall, dispenses the rescue kit,
// and labels the correct tile using the encoder position. (claude version 6/16/2026)
void serviceCameraVictim(){
  drivetrain.fullstop();
  if(victimSide == 1){            // left camera (Serial3)
    if(detectWall(3) == 0){       // RCJ victims are wall-mounted
      Serial.println("victim at left");
      clearSerialBuffer1();
      detectCam1();
      markVictimAtEncoderPosition(TILE_MM);
      victimtoggle = true;
    }
  }
  else if(victimSide == 2){       // right camera (Serial2)
    if(detectWall(1) == 0){
      Serial.println("victim at right");
      clearSerialBuffer2();
      detectCam2();
      markVictimAtEncoderPosition(TILE_MM);
      victimtoggle = true;
    }
  }
  isVictim = true;       // at most one victim serviced per move
  victimPending = false; // re-enable the camera thread
}
// full stop

void fwd(double dist){ // in mm
  double pulses = dist/(wheel_diameter*M_PI)*wheel_cpr*gear_ratio; // easier to make a variable.
  bool black = false; // toggle for black tile
  bool climbtoggle = false; // toggle for climbing
  int cnt = 0; // tiles traversed while climbing.
  double difference = 0; // centering distance
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
  double init_heading = myGyro.heading();
  PID myPID(0.30,0,0.2); // pid for centering
  PID Scale_PID(0.002,0,0.0008); // pid for encoder
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
  while((climbtoggle==true||(drivetrain.encoderCountA+drivetrain.encoderCountB)/2<=pulses)&&black!=true){
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
    // Skipped on ramps / upper floors where the color sensor is unreliable
    // (use_color is incremented after each climbed ramp section below).
    if(use_color == 0){
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
        while(drivetrain.encoderCountA >= 0 && drivetrain.encoderCountB >= 0){
          drivetrain.backward(200);
        }
        black = true;
      }
    }
    // PID centering
    difference = center();
    //Serial.println(center());
    double adjustment = myPID.getPID(difference);
    double Scale = Scale_PID.getPID(pulses-(drivetrain.encoderCountA+drivetrain.encoderCountB)/2);
    // emergency stop
    
    front_left_current = measure(7);
    front_right_current = measure(1);
    
    if((front_left_current<=50&&front_left_current!=-1)||(front_right_current<=50&&front_right_current!=-1)){
      Serial.println("stopping");
      drivetrain.fullstop();
      delay(50);
      break;
    }
    
    /*
    if(myTime.delta_time()>200000){
      Serial.println("ping");
      myTime.reset_delta_time();
      Serial.println(front_left_current-front_left_last);
      front_left_last = front_left_current;
      front_right_last=front_right_current;
      if(front_left_current-front_left_last>1&&front_right_current-front_right_last>1&&climbtoggle == false&&encoderCountA>pulses/30&&encoderCountA<pulses*29/30&&front_left_current!=-1&&front_right_current!=-1){
        delay(500);
        Serial.println(front_left_current-front_left_last);
        detachInterrupt(digitalPinToInterrupt(encoderPin_A_A));
        detachInterrupt(digitalPinToInterrupt(encoderPin_B_A));
        motorA->run(BACKWARD);
        motorB->run(BACKWARD);
        motorC->run(BACKWARD);
        motorD->run(BACKWARD);
        motorA->setSpeed(150);
        motorB->setSpeed(150);
        motorC->setSpeed(150);
        motorD->setSpeed(150);
        delay(250);
        fullstop();
        delay(200);
        absoluteturn(myGyro.opposite_heading(plannedTurnDeg)); // turn 180
        delay(200);
        motorA->run(BACKWARD);
        motorB->run(BACKWARD);
        motorC->run(BACKWARD);
        motorD->run(BACKWARD);
        attachInterrupt(digitalPinToInterrupt(encoderPin_A_A), encoder_update_A, RISING); // turn encoders back on
        attachInterrupt(digitalPinToInterrupt(encoderPin_B_A), encoder_update_B, RISING);
        while(encoderCountA>-pulses*1.3&&encoderCountB>-pulses*1.3){
          motorA->setSpeed(150);
          motorB->setSpeed(150);
          motorC->setSpeed(150);
          motorD->setSpeed(150);
        }
        fullstop();
        delay(200);
        absoluteturn(plannedTurnDeg);
        delay(100);
        break;
        continue;
      }
    }
    */
    // self correction
    // if acceleration is greater than a certain value and it is not just a stop then do something.
    /*
    double d = myGyro.get_filtered_acceleration();
    Serial.println(d);
    if(abs(d)>0.5&&encoderCountA>pulses/30&&encoderCountA<pulses*29/30&&victimtoggle==false){
      Serial.println("botched fwd");
      Serial.println(d);
      // step 1: go back
      motorA->run(BACKWARD);
      motorB->run(BACKWARD);
      motorC->run(BACKWARD);
      motorD->run(BACKWARD);
      motorA->setSpeed(150);
      motorB->setSpeed(150);
      motorC->setSpeed(150);
      motorD->setSpeed(150);
      delay(500);
      fullstop();
      delay(200);
      // step 2: turn( with slight offset)
      absoluteturn(init_heading+(myGyro.modulus((int)myGyro.heading())-myGyro.modulus((int)init_heading)>0 ? -10: 10)); // boundary condition kinda broken but we can fix it in the future.
      // step 3: reset encoders
      encoderCountA = 0; encoderCountB = 0;
    }
    */
    // check for steps( stop)
    /*
    
    */
    // check yaw heading
    // if it is greater than 25, the robot is going up a slope, so the encoder is turned off.
     if(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 20){
      int _encoderCountB = drivetrain.encoderCountB;
      climbtoggle = true;
      drivetrain.reset_encoderCount(true, false);
      Serial.println(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw));
      Serial.println("climbing");
      
      while(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw) > 20){
        Serial.println(drivetrain.encoderCountB);
        drivetrain.set_interrupt(false,true); // stop encoders
         // PID centering
        difference = center();
        //Serial.println(center());
        double adjustment = myPID.getPID(difference);
        // center during climbing
        drivetrain.drive(200+adjustment,200+adjustment,200-adjustment,200-adjustment);
        if(drivetrain.encoderCountB >= pulses/cos(abs(myGyro.modulus(myGyro.yaw_heading())-init_yaw)*(M_PI/180))){
          Serial.println("1 section of the ramp climbed");
          cnt++;
          drivetrain.reset_encoderCount(false,true);
        } // track tiles
      }
      use_color++;
      drivetrain.set_interrupt(true,true);
      drivetrain.set_encoderCountB(_encoderCountB);
    }
    drivetrain.drive(constrain(Scale*(150+adjustment),20,200),constrain(Scale*(150+adjustment),20,200),constrain(Scale*(150-adjustment),20,200),constrain(Scale*(150-adjustment),20,200));
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
  drivetrain.reset_encoderCount(true,true);

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
      if(victimPending){ // service camera victim mid-turn (claude version 6/16/2026)
        myPID.pausePID(1); myTimer.pause(1);
        serviceCameraVictim();
        myPID.pausePID(2); myTimer.pause(2);
      }
      motorB->run(BACKWARD);
      motorD->run(BACKWARD);
      if(myGyro.inverse(angle,fasterway)-current_angle<=0 && current_angle < 190) break;
      
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break; // turning limit
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      
      MOTORSPEED = myPID.getPID(myGyro.inverse(angle,fasterway)-current_angle);
      
      drivetrain.turnright(constrain(MOTORSPEED,20,200));
    }
  }

  else if(myGyro.inverse(angle,fasterway)-current_angle<0) {
    while(true){
      if(victimPending){ // service camera victim mid-turn (claude version 6/16/2026)
        myPID.pausePID(1); myTimer.pause(1);
        serviceCameraVictim();
        myPID.pausePID(2); myTimer.pause(2);
      }
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
      if(myGyro.inverse(angle,fasterway)-current_angle>=0 && current_angle > 170) break;
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break;
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      MOTORSPEED = myPID.getPID(current_angle-myGyro.inverse(angle,fasterway));
      
      drivetrain.turnleft(constrain(MOTORSPEED,20,200));
    }
  }
  
  if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
  Serial.println("finished turning");
  motionActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true); // reset encoder counters.
}
// full stop function
