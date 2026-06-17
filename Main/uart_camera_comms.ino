// camera communication code
// clear serial buffers.

void clearSerialBuffer1() {
  while (Serial2.available() > 0) {
    Serial2.read();  // Read and discard one byte from the buffer
  }
}
void clearSerialBuffer2() {
  while (Serial3.available() > 0) {
    Serial3.read();  // Read and discard one byte from the buffer
  }
}
// led
void flashLED(char victimState){
  if(victimState == 'H'){
    digitalWrite(pinHarmed,HIGH);
    delay(1000);
  }
  else if(victimState == 'S'){
    digitalWrite(pinStable,HIGH);
    delay(1000);
  }
  else{
    digitalWrite(pinUnharmed,HIGH);
    delay(1000);
  }
  digitalWrite(pinHarmed,LOW);
  digitalWrite(pinStable,LOW);
  digitalWrite(pinUnharmed,LOW);
}
int readSerial1(){ // left
  char sample;
  int output;
  
  if(Serial3.available()>0){
    sample = Serial3.read();
    
    if(sample == 'H'){
      output = 0;
    }
    else if(sample == 'S'){
      output = 1;
    }
    else if(sample == 'U'){
      output = 2;
    }
    else if(sample == 'L'||sample == 'c'||sample == 'I'){
      output = -2;
    }
    return output; // output inside or it keeps outputting
  }
  else{
    // nothing right now
    return -1;
  }
  
}
int readSerial2(){ //right
  char sample;
  int output;
  
  if(Serial2.available()>0){
    sample = Serial2.read();
    Serial.println(sample);
    if(sample == 'H'){
      output = 0;
    }
    else if(sample == 'S'){
      output = 1;
    }
    else if(sample == 'U'){
      output = 2;
    }
    else if(sample == 'L'||sample == 'c'|| sample == 'I'){
      output = -2;
    }
    
    return output; // output inside or it keeps outputting
    
  }
  else{
    // nothing right now
    return -1;
  }
}
void detectCam1(){ // doesn't return anything.
   // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int n = 0;
  int maxCount = 0;
  char res = 0;
  char output;
  timer myTimer;
  while(n<5){
    if(myTimer.getTime() > 4*1000000) return;
    
    int value = readSerial1();
    
    if(value >= 0){   // only H/S/U
        samples[n] = value;
        Serial.println(value);
        n++;
    }
  }
  for(int i =0; i<5;i++){
    Serial.println(samples[i]);
  }
  
  for(int i=0;i<6;i++){
    int count = 0;
    for(int j = 0;j<5;j++){
      if(samples[j] == i) count++;
    }
    if(count > maxCount){
      maxCount = count;
      res = classes[i];
    }
  }
  flashLED(res);
  disp.dispenseLeft(res);
  return;
}
void detectCam2(){
  // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int n = 0;
  int maxCount = 0;
  char res = 0;
  char output;
  timer myTimer;
  while(n<5){
    if(myTimer.getTime() > 4*1000000) return;
    
    int value = readSerial2();

    if(value >= 0){   // only H/S/U
        samples[n] = value;
        Serial.println(value);
        n++;
    }
  }
  for(int i =0; i<5;i++){
    Serial.println(samples[i]);
  }
  
  for(int i=0;i<6;i++){
    int count = 0;
    for(int j = 0;j<5;j++){
      if(samples[j] == i) count++;
    }
    if(count > maxCount){
      maxCount = count;
      res = classes[i];
    }
  }
  flashLED(res);
  disp.dispenseRight(res);
  return;
}
void detect(){ // the robot goes forward until it detects something( does not return)
  drivetrain.fullstop();
  bool victimAtLeft = false; // victim at left
  bool victimAtRight = false; // victim at right
  // clear buffers
  clearSerialBuffer1();
  clearSerialBuffer2();
  // don't detect if there is no wall( prevent misdetection)
  bool wallAtLeft = false;
  bool wallAtRight = false;
  int right = measure(3);
  int left = measure(6);
  if(right <MIN_DIST&&right!=-1) wallAtRight = true;
  if(left<MIN_DIST&&right!=-1) wallAtLeft = true;
  if(wallAtRight == false && wallAtLeft == false) return;
  timer myTimer;
  while(true){
    if(readSerial1() != -1&&wallAtLeft == true){
      Serial.println("left");
      victimAtLeft = true;
      victimtoggle = true;
      break;
    }
    if(readSerial2() != -1&&wallAtRight == true){
      Serial.println("right");
      victimAtRight = true;
      victimtoggle = true;
      break;
    }
    if(myTimer.getTime() >= 1000000*1.2) break; // give 1.5 seconds to detect.
    drivetrain.fw(80);
  }
  drivetrain.fullstop();
  delay(200);
  if(victimAtLeft == true){
    
    detectCam1();
  }
  else if(victimAtRight == true){
    
    detectCam2();
  }
  
  // backpedal
  while(drivetrain.encoderCountA > 0){
    drivetrain.backward(100);
  }
  drivetrain.fullstop();

}