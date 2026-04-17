void encoder_update_B(){ // why the fuck is it high when going backwards???
  int encoderState = digitalRead(encoderPin_B_B);
  
  if(encoderState == HIGH){
    encoderCountB--;
  }
  else{
    encoderCountB++;
  }
}
void encoder_update_A(){ // every pulse consists for once going up and once going down.
  int encoderState = digitalRead(encoderPin_A_B);
 
  if(encoderState == LOW){
    encoderCountA--;
  }
  else{
    encoderCountA++;
  }
}
void init_drive(){
  drivetrain.init_drive();
  // initialize gyro
  myGyro.init_Gyro();
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
  PID myPID(0.30,0,0.2); // 0.28 for 125
  Serial.println("forwarding");
  int init_yaw = myGyro.modulus((int)myGyro.yaw_heading());
  int front_left_current=measure(7); int front_right_current=measure(1);
  int front_left_last=measure(7); int front_right_last=measure(1);
  timer myTime;
  myTime.reset_delta_time();
  while((drivetrain.encoderCountA<= pulses && (drivetrain.encoderCountB <= pulses||climbtoggle == true))&&black!=true){
    //if(digitalRead(logicswitch)==true) Pausemaze = true;
    // check cameras
    
    /*
    if(readSerial1()!=-1&&victimtoggle == false&&t.getVictim()==false){ // check serial
      fullstop();
      
      if(detectWall(3)==0){
        Serial.println(measure(6));
        Serial.println("victim at left");
        //Serial.println((char)Serial2.read());
        myPID.pausePID(1);
        clearSerialBuffer1();
        detectCam1();
        myPID.pausePID(2);
        victimtoggle = true;
      }
    }
    else if(readSerial2()!=-1&&victimtoggle == false&&t.getVictim()==false){
      fullstop();
      if(detectWall(1)==0){
        Serial.println(measure(2));
        Serial.println("victim at right");
        //Serial.println((char)Serial3.read());
        myPID.pausePID(1);
        clearSerialBuffer2();
        detectCam2();
        myPID.pausePID(2);
        victimtoggle = true;
      }
    }
    
    if(victimtoggle == true){
      if(encoderCountA<2*pulses/3){
        victimAtCurrent = true;
        mapGrid[x_pos][y_pos].setVictim(true);
      }
      else victimAtCurrent = false;
    }
    */
    // color
    
    int color = read_color(); // read color
    if(color == 1&&use_color%2==0){
      bluetoggle = true;
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(1);
    }
    if(color == -1&&use_color%2==0){
      drivetrain.fullstop();
      delay(100);
      
      //Do you mean  t.setWall(plannedMoveDir, true)
      // find next tile, set it to black
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      mapGrid[nx][ny].setType(3);
      blacktoggle = true;
      
      while(encoderCountA >= 0 && encoderCountB >= 0){
        drivetrain.backward(200);
      }
      black = true;
    }
    
    // PID centering
    difference = center();
    //Serial.println(center());
    double adjustment = myPID.getPID(difference);
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
        Serial.println(encoderCountB);
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
    drivetrain.drive(150+adjustment,150+adjustment,150-adjustment,150-adjustment);
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

  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true, true);

}
// absolute turning
void absoluteturn(double angle){ 
  // create PID instance.
  PID myPID(10,0,0.3); 
  double MOTORSPEED = 0;
  double current_angle=myGyro.heading();
  bool fasterway = false;
  Tile &t = mapGrid[x_pos][y_pos]; // tile object to update
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
      motorB->run(BACKWARD);
      motorD->run(BACKWARD);
      if(myGyro.inverse(angle,fasterway)-current_angle<=0 && current_angle < 190) break;
      
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break; // turning limit
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      
      MOTORSPEED = myPID.getPID(myGyro.inverse(angle,fasterway)-current_angle);
      if(readSerial1()!=-1&&victimtoggle == false&&t.getVictim()==false){
        drivetrain.fullstop();
        if(detectWall(3)==0){
          Serial.println("victim at left");
          myPID.pausePID(1);
          myTimer.pause(1);
          
          clearSerialBuffer1();
          detectCam1();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
      }
      else if(readSerial2()!=-1&&victimtoggle == false&&t.getVictim()==false){
        drivetrain.fullstop();
        if(detectWall(1)==0){
          Serial.println("victim at right");
          myPID.pausePID(1);
          myTimer.pause(1);
          
          clearSerialBuffer2();
          detectCam2();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
      }
      drivetrain.turnright(constrain(MOTORSPEED,20,200));
    }
  }

  else if(myGyro.inverse(angle,fasterway)-current_angle<0) {
    while(true){
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
      if(myGyro.inverse(angle,fasterway)-current_angle>=0 && current_angle > 170) break;
      if(myTimer.getTime() > 2*abs(myGyro.inverse(angle,fasterway)-init_angle)/90*1000000) break;
      current_angle = myGyro.inverse(myGyro.heading(),fasterway);
      MOTORSPEED = myPID.getPID(current_angle-myGyro.inverse(angle,fasterway));
      if(readSerial1()!=-1&&victimtoggle == false&&t.getVictim()==false){
        drivetrain.fullstop();
        if(detectWall(3)==1){
          Serial.println("victim at left");
          myPID.pausePID(1);
          myTimer.pause(1);
          
          clearSerialBuffer1();
          detectCam1();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
        
      }
      else if(readSerial2()!=-1&&victimtoggle == false&&t.getVictim()==false){
        drivetrain.fullstop();
        if(detectWall(1)==1){
          Serial.println("victim at right");
          myPID.pausePID(1);
          myTimer.pause(1);
          clearSerialBuffer2();
          detectCam2();
          myTimer.pause(2);
          myPID.pausePID(2);
          victimtoggle = true;
        }
      }
      drivetrain.turnleft(constrain(MOTORSPEED,20,200));
    }
  }
  
  if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
  Serial.println("finished turning");
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true, true);
}
// full stop function
