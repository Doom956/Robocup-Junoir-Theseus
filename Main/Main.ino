// claude version 6/16/2026 — single-core port of the multicore_version loop +
// navigation, with multi-floor elevation and an RTOS camera-detection thread.
#include <mbed.h> // access arduino mbed OS (rtos::Thread)
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Stepper.h>
#include <array> // std::array (Grid type for multi-floor maps)
#include <ArduinoQueue.h> // queue
#include <Vector.h> // vector
#include "PID.h"
#include "timer.h"
#include "gyro.h"
#include "dispenser.h"
#include "motors.h"
#define MIN_DIST 120         // mm (tune this)
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define BLACK_THRESHOLD 0.1 // color clear-channel threshold ratio for black
#define SILVER_THRESHOLD 1.2f // ratio threshold — calibrate on real silver tile (typical normal~0.8, silver~2.0+)
#define WHITE_THRESHOLD 0.9f
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
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
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
const int encoderPin_A_A = 3;
const int encoderPin_A_B = 5; 
const int encoderPin_B_A = 2;
const int encoderPin_B_B = 4; 
// encoder counters

//drivetrain class object
motors drivetrain(encoderPin_A_A,encoderPin_A_B,encoderPin_B_A,encoderPin_B_B);
// wheel cpr
const double wheel_cpr = 5; // 20/4
//gear ratio
const double gear_ratio = 195;
// wheel diameter
const double wheel_diameter = 80; // millimeters.
// detection classes

char classes[6] = {'H','S','U','R','Y','G'};

// create stepper object
const int steps_per_revolution = 2048;
Stepper myStepper = Stepper(steps_per_revolution, 8, 9,10,11); 
// map grids (MAP_SIZE and the Grid type are defined in MazeTile.h)
Grid mapGrid; // active floor's tiles
Grid m1;      // floor storage ("basement"/floor 1)
Grid m2;      // floor 2
Grid m3;      // floor 3
int currentFloor = 1; // current floor (1..3) for elevation()/descend()
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
  WIGGLE,
  FWD
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
volatile bool Pausemaze = false; // set by pauseThread, read by loop()
int x_checkpoint, y_checkpoint;
bool tilecheck = false;

double headingErrorDeg(double targetDeg, double actualDeg) {
  double err = targetDeg - actualDeg;
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return abs(err);
}

// ===== camera victim-detection RTOS thread (claude version 6/16/2026) =====
// The thread only checks the camera UARTs (Serial3 = left, Serial2 = right).
// It never touches the I2C bus (mux/distance/color) so it cannot race the main
// context's measure()/detectWall() calls. When a camera reports a letter while
// the robot is moving, the thread raises victimPending; fwd()/absoluteturn()
// then stop the drivetrain, pause their PID + timer, run detectCam(), and use
// markVictimAtEncoderPosition() to label the correct tile before resuming.
volatile bool motionActive = false; // true only while inside fwd()/absoluteturn()
volatile bool victimPending = false; // a camera reported -> movement must service it
volatile int  victimSide = 0;        // 1 = left (Serial3), 2 = right (Serial2)
volatile bool isVictim = false;      // a victim already handled during current move

rtos::Thread cameraThread;
void cameraTask(){
  while(true){
    if(motionActive && !victimPending && !isVictim){
      if(mapGrid[x_pos][y_pos].getVictim() == false){
        if(readSerial1() != -1){        // left camera (Serial3)
          victimSide = 1;
          victimPending = true;
        }
        else if(readSerial2() != -1){   // right camera (Serial2)
          victimSide = 2;
          victimPending = true;
        }
      }
    }
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
  }
}

// pause maze thread: watches the logic switch and requests a stop.
rtos::Thread pauseThread;
void pauseTask(){
  while(true){
    if(digitalRead(logicswitch)==true) Pausemaze = true;
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
  }
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
  pinMode(logicswitch, INPUT);
  // begin UART communication.
  Serial.begin(115200);
  Serial2.begin(115200); // switch to 9600 for reliability
  Serial3.begin(115200);
  //flashLED('S');
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
  // every floor starts as a copy of the freshly initialized (empty) grid.
  m1 = mapGrid;
  m2 = mapGrid;
  m3 = mapGrid;
  currentFloor = 1;
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].setDiscovered(true);
  currentDir = NORTH;
  state = SENSE_TILE;

  // start RTOS threads: camera victim detection + pause-switch watcher.
  cameraThread.start(cameraTask);
  pauseThread.start(pauseTask);

  delay(2000); // wait for camera to start.
}
int iterator = 0;
void loop(){
  static bool wallF, wallR, wallB, wallL;
  switch (state) {
    case SENSE_TILE: {
      // reset per-tile toggles
      blacktoggle = false; bluetoggle = false;
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
      state = VICTIM_DETECT; // poll cameras while stopped before planning.
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case VICTIM_DETECT: {
      // The robot is stationary here, so the camera thread is idle (motionActive
      // == false) and this is the only context touching the camera UARTs.
      // Poll both cameras; RCJ victims are wall-mounted, so a wall must be
      // present on that side before we identify/dispense.
      Tile &t = mapGrid[x_pos][y_pos];
      if(t.getVictim() == false){
        if(readSerial1() != -1 && detectWall(3) == 0){       // left camera (Serial3)
          Serial.println("victim at left");
          clearSerialBuffer1();
          detectCam1();
          victimtoggle = true;
        }
        else if(readSerial2() != -1 && detectWall(1) == 0){  // right camera (Serial2)
          Serial.println("victim at right");
          clearSerialBuffer2();
          detectCam2();
          victimtoggle = true;
        }
        // a victim discovered for this tile -> record it on the tile datatype.
        if(victimtoggle == true) t.setVictim(true);
        victimtoggle = false;
      }
      delay(100);
      state = PLAN_NEXT;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case PLAN_NEXT: {
      plannedMoveDir = pickNextDirection();
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      Serial.println(plannedTurnDeg);
      state = EXECUTE_MOVE;
      if(Pausemaze == true) state = PAUSE;
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

      // 2) drive one tile. fwd() sets blacktoggle/bluetoggle, handles ramps
      //    (advancing x_pos/y_pos for any climbed tiles) and services any
      //    camera victim reported by the RTOS thread during the move.
      fwd(TILE_MM);

      // 3) update map + robot position only on a successful (non-black) move
      if(blacktoggle == false){
        markEdgeBothWays(x_pos, y_pos, currentDir);
        stepForward(currentDir, x_pos, y_pos); // x_pos/y_pos now = new tile
        if(bluetoggle == true){ // RCJ: stop 5 seconds on the blue tile we entered
          drivetrain.fullstop();
          delay(5000);
          mapGrid[x_pos][y_pos].setBlue(true); // marks the correct (new) tile
          mapGrid[x_pos][y_pos].setType(BLUE);
        }
        bluetoggle = false;
      }
      else{
        bluetoggle = false;
        state = BACKPEDAL; // black tile ahead (marked BLACK by fwd) -> back off
        turnCompletedForMove = false;
        break;
      }

      delay(200);
      parallel();
      delay(100);
      iterator += 1;

      isVictim = false;
      turnCompletedForMove = false;
      tilecheck = false;
      state = SENSE_TILE;
      if(Pausemaze == true) state = PAUSE;
      //if(mazeTime.getTime() >= 1000000*60*6) state = RETURN;
      //if(medkits <= 0) state = RETURN;
      if(iterator >= 15) state = RETURN;
      break;
    }
    case BACKPEDAL: {
      plannedMoveDir = pickNextDirection();
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      state = EXECUTE_MOVE;
      blacktoggle = false;
      if(Pausemaze == true) state = PAUSE;
      delay(500);
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
    case RETURN: {
      coord currentpos = {x_pos,y_pos};
      coord endpos = {MAP_SIZE/2,MAP_SIZE/2};
      static coord path[MAP_SIZE*MAP_SIZE]; // static: avoid a large stack frame
      flashLED('H');
      flashLED('U');
      Serial.println("starting bfs");
      int length = BFS(currentpos,mapGrid,endpos,path);
      Serial.println("path calculated");
      for(int i = length - 1;i>0;i--){
        // convert the coordinate step into an absolute direction
        Direction moveDir;
        if(path[i-1].y - path[i].y == 0){
          moveDir = (path[i-1].x - path[i].x == 1) ? EAST : WEST;
        }
        else{
          moveDir = (path[i-1].y - path[i].y == 1) ? NORTH : SOUTH;
        }
        plannedTurnDeg = turnNeededDeg(moveDir);
        absoluteturn(plannedTurnDeg);
        delay(200);
        parallel();
        delay(100);
        currentDir = moveDir;
        fwd(TILE_MM);
      }
      flashLED('H');
      while(true){
        drivetrain.fullstop();
      }
    }
    case PAUSE: {
      drivetrain.fullstop();
      delay(200);
      x_pos = x_checkpoint; y_pos = y_checkpoint; // resume from last checkpoint
      if(digitalRead(logicswitch)==LOW){
        Pausemaze = false;
        Direction snapped = (Direction)myGyro.headingToCardinal(myGyro.heading()); // snap to cardinal
        absoluteturn(turnNeededDeg(snapped));
        currentDir = snapped;
        state = PLAN_NEXT;
      }
      break;
    }
 }
}
