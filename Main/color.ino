
void init_color(){
  //tcs.setInterrupt(true);
  myMux.setPort(TCS_PORT);
  //digitalWrite(POWERPIN,HIGH);
  //tcs.enable();
  if (tcs.begin()) {
    Serial.println("Found tcs34725 sensor");
    
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  
    
    
    //while (1); // halt!
  }
  myMux.setPort(TCS_PORT);          
  tcs.setInterrupt(true);  // turn on LED
  float red, green, blue;
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  clear = c;
  Serial.println("clear value");
  Serial.println(clear);
  
}
int read_color(){
  myMux.setPort(TCS_PORT);          
  tcs.setInterrupt(true);  // turn on LED
  float red, green, blue;
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
 
  if((float)c/clear<BLACK_THRESHOLD){
    
    
    return -1; // black
  }
  if((float)c/clear>SILVER_THRESHOLD){
    int nx = x_pos; int ny = y_pos;
    stepForward(currentDir,nx,ny);
    mapGrid[nx][ny].setType(CHECKPOINT);
    x_checkpoint = nx; y_checkpoint = ny;
    return 3; // SILVER — prevent fall-through into blue/red checks
  }
  if(b>g&&b>r){
    return 1; // blue
  }
  if(r>g&&r>b){
    return 2;
  }
  else{
    return 0; // good
  }
}
  