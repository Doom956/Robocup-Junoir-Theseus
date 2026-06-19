// distance sensor code
// blue for SDA, yellow for SCL
// the motor shield takes up the I2C address at 0x70 so chatgpt made some code that prevents conflict.



void disableAllCall() {
  // Point register to MODE1
  Wire.beginTransmission((uint8_t)PCA_ADDR);
  Wire.write((uint8_t)MODE1);
  Wire.endTransmission(false);

  // Read MODE1
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  if (Wire.available() < 1) return; // couldn't read
  uint8_t mode1 = Wire.read();

  // Clear ALLCALL bit
  mode1 &= (uint8_t)~ALLCALL_BIT;

  // Write MODE1 back
  Wire.beginTransmission((uint8_t)PCA_ADDR);
  Wire.write((uint8_t)MODE1);
  Wire.write(mode1);
  Wire.endTransmission(true);
}

void init_dist() {
  // put your setup code here, to run once:
  
  if(!myMux.begin()){
    Serial.println("can't find the Mux");
  }
  else{
    Serial.println("Mux initialized");
  }
 
  for(int i = 0; i<7; i++){
    myMux.setPort(i);
    sensors[i].setAddress(0x30); // conflict with TCS34725 for some reason.
    delay(10);
    
    
    if(!sensors[i].init()){
    Serial.println("Sensor "+String(i)+" failed to initialize");
    }
    else{
      Serial.println("Sensor "+String(i)+" is able to initialize");
    }
    sensors[i].startContinuous(); // start continuous ranging.
  }
    
  
  

}

uint8_t scanI2COnCurrentBus() {
  uint8_t count = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" ");
      count++;
    }
  }
  return count;
}

void scanAllPorts() {
  for (uint8_t port = 0; port < 8; port++) {
    bool ok = myMux.setPort(port);

    Serial.print("Port ");
    Serial.print(port);
    Serial.print(ok ? ": " : ": (setPort FAILED) ");

    delay(20);

    uint8_t found = scanI2COnCurrentBus();
    if (found == 0) Serial.print("(none)");
    Serial.println();
  }
}
// measure distance
/*
int measure(int sensor){
  
  if(sensor ==1){
    myMux.setPort(2);
    int value = sensors[2].readRangeContinuousMillimeters();
    
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
      
  }
  if(sensor == 2){
    myMux.setPort(1);
    int value = sensors[1].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
      
  }
  if(sensor==3){
    myMux.setPort(0);
    int value = sensors[0].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==4){
    myMux.setPort(3);
    int value = sensors[3].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==5){
    myMux.setPort(6);
    int value = sensors[6].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==6){
    myMux.setPort(5);
    int value = sensors[5].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==7){
    myMux.setPort(4);
    int value = sensors[4].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }

  return -1;
}
old robot settings
*/
int measure(int sensor){
  // sensor→mux port mapping
  const int portMap[] = {-1, 1, 0, 6, 4, 5, 3, 2};
  if(sensor < 1 || sensor > 7) return -1;
  int port = portMap[sensor];
  int sensorIdx = port; // sensor index matches port number

  i2cMutex.lock();
  myMux.setPort(port);
  int value = sensors[sensorIdx].readRangeContinuousMillimeters();
  i2cMutex.unlock();

  return (value != -1 && value != 8191) ? value : -1;
}
// detects wall in a direction( 0 is north, 1 is east, etc..) If output = 0, there is a wall.
// realtive directions(local).
int detectWall(int dir){
  if(dir == 0){ // check if there is a wall at north
    int a = measure(1);
    int b = measure(7);
    if((a<MIN_DIST&&a!=-1&&a!=8191)||(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0; // there is a wall.
    }
    else{
      return 1; // no wall
    }
  }
  if(dir == 1){
    int a = measure(2);
    int b = measure(3);
    if((a<MIN_DIST&&a!=-1&&a!=8191)||(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0;
    }
    else{
      return 1;
    }
  }
  if(dir == 2){
    int a = measure(4);
    if(a<MIN_DIST&&a!=-1&&a!=8191){
      return 0;
    }
    else{
      return 1;
    }
  }
  if(dir == 3){
    int a = measure(5);
    int b = measure(6);
    
    if((a<MIN_DIST&&a!=-1&&a!=8191)||(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0;
    }
    else{
      return 1;
    }
    
  }

  return 1;
}

void parallel(){
  const int PARALLEL_TOL_MM = 3;
  const int PARALLEL_SPEED = 90;
  const unsigned long PARALLEL_TIMEOUT_MS = 700;
  const double MAX_PARALLEL_ROTATION_DEG = 45.0;

  int sensorA = -1;
  int sensorB = -1;
  int wallDir;
  Serial.println("paralleling");
  // Prefer aligning to the right wall; otherwise use left wall.
  if (detectWall(1) == 0) {
    sensorA = 2;
    sensorB = 3;
    wallDir=1;
  } else if (detectWall(3) == 0) {
    sensorA = 6;
    sensorB = 5;
    wallDir=3;
  } else {
    drivetrain.fullstop();
    return;
  }

  unsigned long startMs = millis();
  double startHeading = myGyro.heading();

  while (true) {
    int a = measure(sensorA);
    int b = measure(sensorB);

    // Invalid reading: stop correction to avoid runaway spinning.
    if (a < 0 || b < 0) {
      Serial.println("parallel: invalid sensor reading, aborting correction");
      break;
    }
    if(a>MIN_DIST||b>MIN_DIST){
      break;
    }

    int diff = a - b;
    if (abs(diff) <= PARALLEL_TOL_MM) {
      Serial.println("paralleled");
      break;
    }
    // break out after rotation.

    double headingDelta = myGyro.heading() - startHeading;
    while (headingDelta > 180.0) headingDelta -= 360.0;
    while (headingDelta < -180.0) headingDelta += 360.0;

    if (abs(headingDelta) >= MAX_PARALLEL_ROTATION_DEG) {
      Serial.println("parallel: rotation limit hit, aborting correction");
      break;
    }

    if ((millis() - startMs) >= PARALLEL_TIMEOUT_MS) {
      Serial.println("parallel: timeout, aborting correction");
      break;
    }

    // Reset wheel directions then apply correction turn.
    i2cMutex.lock();
    motorA->run(FORWARD);
    motorB->run(FORWARD);
    motorC->run(FORWARD);
    motorD->run(FORWARD);
    if ((diff > 0 && wallDir == 1)||(diff < 0 && wallDir==3)) {
      motorB->run(BACKWARD);
      motorD->run(BACKWARD);
    } else {
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
    }
    i2cMutex.unlock();
    drivetrain.drive(PARALLEL_SPEED,PARALLEL_SPEED,PARALLEL_SPEED,PARALLEL_SPEED);
    
  }
  drivetrain.reset_encoderCount(true,true,true);
  drivetrain.fullstop();
}
int center(){
  int a = measure(2);
  int b = measure(6);
  if(a<MIN_DIST && a != -1 && b<MIN_DIST && b != -1) return (a-b);
  else return 0;
}

int leftright = 0;
void obstacleavoidance(int leftright){ // leftright determines to manuver left or right.
  while(true){
    switch (steps){
      case TURN:{
        if(leftright == 1){ // obstacle at left
          Serial.println("turn step");
          while(measure(7) < MIN_DIST){
            motorB->run(BACKWARD);
            motorD->run(BACKWARD);
            drivetrain.drive(255,255,255,255);
          }
        }
        else if(leftright == 0){ // obstacle at right
          while(measure(1)<MIN_DIST){
            motorA->run(BACKWARD);
            motorC->run(BACKWARD);
            drivetrain.drive(255,255,255,255);
          }
          
        }
        drivetrain.fullstop();
        delay(200);
        drivetrain.fw(255);
        delay(300);
        drivetrain.fullstop();
        delay(200);
        steps = PARALLEL;
        break;
      }
      case PARALLEL:{
        PID pid(1,0,0.1);
        if(leftright == 1){
          int a = measure(2); int b = measure(3);
          while(true){
            a=measure(2); b = measure(3);
            if(a<=30) break;
            Serial.println("paralleling step");
            double increment = pid.getPID(abs(a-b)); // get close to the edge as possible.
            drivetrain.drive(constrain(100-increment,50,170),constrain(100-increment,50,170),constrain(100+increment,50,170),constrain(100+increment,50,170));
            
            if(measure(6)<=35&&a<=35){
              steps = WIGGLE;
              goto end;
            }
            
            if(abs(b-a)<=15){
              //fwd
              drivetrain.fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        else if(leftright == 0){
          while(measure(6)>=35){
            double increment = pid.getPID(abs(measure(6)-measure(5))); // get close to the edge as possible.
            drivetrain.drive(constrain(100-increment,50,170),constrain(100-increment,50,170),constrain(100+increment,50,170),constrain(100+increment,50,170));
            if(measure(6)<=35&&measure(2)<=35){
              steps = WIGGLE;
              goto end;
            }
            if(abs(measure(6)-measure(5))<=10){
              drivetrain.fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        Serial.println("too close, backing up");
        Serial.println(measure(2));
        steps = BACKTRACK; // put switch step in front of end( always meet it)
        break;
        end:
          break;
        
      }
      case BACKTRACK:{
        Serial.println("backtracking step");
        // put a timer on to prevent it from taking too long
        timer myTime;
        if(leftright == 0){
          while(measure(6)<=40&&myTime.getTime()<800000){
            drivetrain.backward(120);
          }
        }
        else if(leftright == 1){
          while(measure(2)<=40&&myTime.getTime()<800000){
            drivetrain.backward(120);
          }
        }
        drivetrain.fullstop();
        delay(200);
        Serial.println("sensor 2, now reading");
        Serial.println(measure(2));
        steps = PARALLEL;
        break;
      }
      
      case FWD:{
        
        if(measure(6)<=35&&measure(2)<=35){
          steps = WIGGLE;
          break;
        }
        
        Serial.println("fwd step");
        parallel();
        drivetrain.reset_encoderCount(true,true,true);
        delay(200);
        fwd(300);
        return;
      }
      case WIGGLE:{
        PID pid(8,0,0.1);
        Serial.println("wiggle step");
        delay(2000);
        timer myTime;
        while(abs(measure(2)-measure(6))>=15&&myTime.getTime()<1000000){
          double diff = pid.getPID(measure(2)-measure(6));
          drivetrain.drive(70+diff,70+diff,70-diff,70-diff);
        }
        drivetrain.fullstop();
        delay(200);
        steps = FWD;
        break;
      }
    }
  }
}



