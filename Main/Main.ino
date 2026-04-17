
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Stepper.h>
#include <ArduinoQueue.h> // queue
#include <Vector.h> // vector
#include "PID.h"
#include "timer.h"
#include "gyro.h"
#include "dispenser.h"
#include "motors.h"
#define MIN_DIST 120         // mm (tune this)
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define BLACK_THRESHOLD 0.10 // color clear-channel threshold ratio for black
#define SILVER_THRESHOLD 800 // tun3
float clear; 

#include "MazeTile.h"

// set up mux and distance senosrs
VL53L0X sensors[7];
QWIICMUX myMux;
// shut down allcall
#define PCA_ADDR 0x60
#define MODE1    0x00
#define ALLCALL_BIT 0x01  // MODE1 bit0

// set up color sensor
#define TCS_PORT 7
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// set up gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
gyro myGyro;
// set up motorshield and motors.
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorA = AFMS.getMotor(1);
Adafruit_DCMotor *motorB = AFMS.getMotor(2);
Adafruit_DCMotor *motorC = AFMS.getMotor(3);
Adafruit_DCMotor *motorD = AFMS.getMotor(4);

// set up encoder pins
const int encoderPin_A_A = 2;
const int encoderPin_A_B = 4; 
const int encoderPin_B_A = 3;
const int encoderPin_B_B = 5; 
// encoder counters
volatile int encoderCountB;
volatile int encoderCountA;
//drivetrain class object
motors drivetrain(encoderPin_A_A,encoderPin_A_B,encoderPin_B_A,encoderPin_B_B);
// wheel cpr
const double wheel_cpr = 5; // 20/4
//gear ratio
const double gear_ratio = 195;
// wheel diameter
const double wheel_diameter = 68.7; // millimeters.
// detection classes

char classes[6] = {'H','S','U','R','Y','G'};

// create stepper object
const int steps_per_revolution = 2048;
Stepper myStepper = Stepper(steps_per_revolution, 8, 9,10,11); 
// map size variables
const int MAP_SIZE=20;
Tile mapGrid[MAP_SIZE][MAP_SIZE]; // array of tiles
// queue


//states that the robot will be in
enum RobotState {
  SENSE_TILE,
  UPDATE_MAP,
  PLAN_NEXT,
  VICTIM_DETECT,
  EXECUTE_MOVE,
  BOTCHED_TURN_RECOVERY,
  BACKPEDAL,
  PAUSE,
  RETURN
};
enum Steps {
  TURN,
  PARALLEL,
  BACKTRACK,
  FWD,
};
// coord struct
struct coord {
  int x;
  int y;
};
Steps steps = TURN;

// initialize 

Direction currentDir = NORTH;     // robot heading in map coords (0..3)
int plannedTurnDeg = 0;           // -90,0,+90,180
Direction plannedMoveDir = NORTH; // absolute direction robot will move next
bool turnCompletedForMove = false;
int x_pos = MAP_SIZE/2;
int y_pos = MAP_SIZE/2;
RobotState state = SENSE_TILE;
// maze return to start condition variables
int medkits = 8;
timer mazeTime;
// black blue toggles
bool blacktoggle = false;
bool bluetoggle = false;
bool stairtoggle = false;
// victim toggles
bool victimtoggle = false;
bool victimAtCurrent = false;
// LED pins
const int pinHarmed = 41;
const int pinStable = 37;
const int pinUnharmed = 29;
// camera GPIOs
const int gpio1 = 13;
const int gpio2 = 12;
// de-activate color sensor while climbing
int use_color = 0;
// stepper variables
const int angle_offset = 44;
const int angle_increment = 22;
dispenser disp(angle_increment,angle_offset,steps_per_revolution);
// logic switch pin
const int logicswitch = 31;
bool Pausemaze = false;
int x_checkpoint, y_checkpoint;
bool tilecheck = false;

double headingErrorDeg(double targetDeg, double actualDeg) {
  double err = targetDeg - actualDeg;
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return abs(err);
}

bool turnCompletedSuccessfully(Direction intendedDir) {
  const double TURN_SUCCESS_TOLERANCE_DEG = 20.0;
  double targetHeading = turnNeededDeg(intendedDir);
  double actualHeading = myGyro.heading();
  double err = headingErrorDeg(targetHeading, actualHeading);
  Serial.print("turn target=");
  Serial.print(targetHeading);
  Serial.print(", actual=");
  Serial.print(actualHeading);
  Serial.print(", err=");
  Serial.println(err);
  return err <= TURN_SUCCESS_TOLERANCE_DEG;
}
void setup(){
  // initialize LED puns
  pinMode(pinHarmed,OUTPUT);
  pinMode(pinStable,OUTPUT);
  pinMode(pinUnharmed,OUTPUT);
  // initialize camera gpio pins
  pinMode(gpio1, INPUT);
  pinMode(gpio2, INPUT);
  // initialize logic switch pin

  // begin UART communication.
  Serial.begin(115200);
  Serial2.begin(115200); // switch to 9600 for reliability
  Serial3.begin(115200);
  flashLED('S');
  uint8_t cause = MCUSR;
  MCUSR = 0;
  Wire.begin();
  disableAllCall();
  myMux.begin();
  init_dist(); // initialize mux before distance sensors.
  scanAllPorts();
  init_color();
  init_drive();
  //detect();
  //initialize map
  initializeMap();
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].setDiscovered(true); 
  currentDir = NORTH;
  state = SENSE_TILE;
  
  delay(2000); // wait for camera to start.
  
  
}
int iterator = 0;
void loop(){
  
  /*
  if(Serial2.available()>0){
    Serial.println((char)Serial2.read());
    Serial.println("letters coming");
    delay(1000);
    detectCam1();
    clearSerialBuffer1();
  }
  Serial.println("nothing");
  */
  
  
  static bool wallF, wallR, wallB, wallL;
  switch (state) {
    case SENSE_TILE: {
      // Read for walls
      readWallsRel(wallF, wallR, wallB, wallL);
      delay(500);
      state = UPDATE_MAP; // next state.
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case UPDATE_MAP: {
      writeWallsToCurrentTile(wallF, wallR, wallB, wallL);
      updateFullyExploredAt(x_pos, y_pos);
      state = PLAN_NEXT;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case VICTIM_DETECT: {
       if(mapGrid[x_pos][y_pos].getVictim() == false){
        if(measure(1)>MIN_DIST){
          detect();
        }

        if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
        victimtoggle = false;
      }
      delay(200);
      parallel();
      delay(100);
      break;
    }
    case PLAN_NEXT: {
      plannedMoveDir = pickNextDirection();

      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      Serial.println(plannedTurnDeg);
      if(Pausemaze == true) state = PAUSE;
      state = EXECUTE_MOVE;
      break;
    }
    case EXECUTE_MOVE: {
      if (turnCompletedForMove == false) {
        if(plannedMoveDir != currentDir){
          absoluteturn(plannedTurnDeg);
        }
        delay(200);
        parallel();
        delay(100);

        if (turnCompletedSuccessfully(plannedMoveDir) == false) {
          state = BOTCHED_TURN_RECOVERY;
          break;
        }
        currentDir = plannedMoveDir;
        turnCompletedForMove = true;
      }
      if(tilecheck == false){
        if(mapGrid[x_pos][y_pos].getVictim() == false){
          if(measure(1)>MIN_DIST){
            tilecheck = true;
            detect();
          }
        if(victimtoggle == true) mapGrid[x_pos][y_pos].setVictim(true);
          victimtoggle = false;
        }
      }
      // 2) drive one tile
      fwd(TILE_MM);
      // 3) update map + robot position only on successful move
      if(bluetoggle == true){ // stop for 5 seconds on the blue tile.
        drivetrain.fullstop();
        delay(5000);
        mapGrid[x_pos][y_pos].setBlue(true);
      }
      bluetoggle = false;
      if(blacktoggle == false && stairtoggle == false){
        markEdgeBothWays(x_pos, y_pos, currentDir);
        stepForward(currentDir, x_pos, y_pos);
        
      }
      else{
        state = BACKPEDAL;
        turnCompletedForMove = false;
        break;
      }
      delay(200);
      parallel();
      delay(100);
      iterator += 1;
      
      victimtoggle = false;
      turnCompletedForMove = false;
      tilecheck = false;
      state = SENSE_TILE;
      if(Pausemaze == true) state = PAUSE;
      //if(mazeTime.getTime() >= 1000000*60*6) state = RETURN;
      //if(medkits <= 0) state = RETURN;
      if(iterator >= 15) state = RETURN;
      break;
     
    }
    case BOTCHED_TURN_RECOVERY: {
      Direction snappedDir = (Direction)myGyro.headingToCardinal(myGyro.heading());
      int snappedHeading = turnNeededDeg(snappedDir);
      Serial.println("botched turn detected, snapping to cardinal");
      absoluteturn(snappedHeading);
      delay(150);
      parallel();
      delay(100);

      currentDir = snappedDir;
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      state = EXECUTE_MOVE;
      break;
    }
    case BACKPEDAL: {
      plannedMoveDir = pickNextDirection();
     
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      state = EXECUTE_MOVE;
      blacktoggle = false;
      stairtoggle = false;
      if(Pausemaze == true) state = PAUSE;
      delay(500);
      break;
    }
    case RETURN: {
      coord currentpos = {x_pos,y_pos};
      coord endpos = {MAP_SIZE/2,MAP_SIZE/2};
      coord path[MAP_SIZE*MAP_SIZE];
      flashLED('H');
      flashLED('U');
      Serial.println("starting bfs");
      int length = BFS(currentpos,mapGrid,endpos,path);
      Serial.println("path calculated");
      for(int i = length;i>0;i--){
        // coorinates to direction
        if(path[i-1].y-path[i].y == 0){
          if(path[i-1].x-path[i].x == 1){
            plannedTurnDeg = turnNeededDeg(1);
            absoluteturn(plannedTurnDeg);
            delay(200);
            parallel();
            delay(100);
            //update currentDir
            currentDir = 1;
            // 2) drive one tile
            fwd(TILE_MM);
          }
          else if(path[i-1].x-path[i].x == -1){
            plannedTurnDeg = turnNeededDeg(3);
            absoluteturn(plannedTurnDeg);
            delay(200);
            parallel();
            delay(100);
            //update currentDir
            currentDir = 3;
            // 2) drive one tile
            fwd(TILE_MM);
          }
        }
        else{
          if(path[i-1].y-path[i].y == 1){
            plannedTurnDeg = turnNeededDeg(0);
            absoluteturn(plannedTurnDeg);
            delay(200);
            parallel();
            delay(100);
            currentDir = NORTH;
            fwd(TILE_MM);
          }
          else if(path[i-1].y-path[i].y == -1){
            plannedTurnDeg = turnNeededDeg(2);
            absoluteturn(plannedTurnDeg);
            delay(200);
            parallel();
            delay(100);
            currentDir = SOUTH;
            fwd(TILE_MM);
          }
        }
        
      }
      flashLED('H');
      while(true){
        drivetrain.fullstop();
      }
    }
    case PAUSE: {
      while(digitalRead(logicswitch)==true){
        drivetrain.fullstop();
        x_pos = x_checkpoint; y_pos = y_checkpoint;
      }
      myGyro.headingToCardinal(myGyro.heading());
      state = SENSE_TILE;
      break;
    }
 }
 



}
