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
#include <LiquidCrystal.h> // lcd screen
#include <array> // std::array (Grid type for multi-floor maps)
#include <deque>
#include <vector>
#include <utility>

#include <ArduinoQueue.h> // queue
#include <Vector.h> // vector
#include "PID.h"
#include "timer.h"
#include "gyro.h"
#include "dispenser.h"
#include "motors.h"
#define MIN_DIST 150         // mm (tune this)
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define ROBOT_LENGTH_MM 195                                      // mm, robot front-to-back length
#define TARGET_GAP_MM (((double)TILE_MM - ROBOT_LENGTH_MM) / 2.0) // mm, ideal front/back clearance when centered (52.5)
#define CENTER_TOL_MM 10                                          // mm, front-back centering tolerance
#define MAX_CENTER_CORRECTION_MM 300.0                            // mm, one tile — offset this large means an unreliable reading or the robot isn't really in-tile; skip/abort centering
#define BLACK_THRESHOLD 0.1 // color clear-channel threshold ratio for black
#define SILVER_THRESHOLD 0.90f // ratio threshold — calibrate on real silver tile (typical normal~0.8, silver~2.0+)
#define WHITE_THRESHOLD 0.99f
#define MULTIPLER 1.1 
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
const int encoderPin_D_A = 18;
const int encoderPin_D_B = 19;


//drivetrain class object
motors drivetrain(encoderPin_A_A,encoderPin_A_B,encoderPin_B_A,encoderPin_B_B,encoderPin_D_A,encoderPin_D_B);
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
// create lcd object
int en = 25; int rs = 27; int d4 = 23; int d5 = 53; int d6 = 29; int d7 = 31;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// map grids 
// MAP_SIZE and grid are defined here
const int MAP_SIZE = 40;
using Grid = std::array<std::array<Tile, MAP_SIZE>, MAP_SIZE>;
Grid mapGrid; // active floor's tiles
Grid m1;      // floor storage ("basement"/floor 0)
Grid m2;      // floor 1
Grid m3;      // floor 2

int currentFloor = 0; // current floor (0..2) for elevation()/descend()



//states that the robot will be in
enum RobotState {
  SENSE_TILE,
  CENTERING,
  UPDATE_MAP,
  PLAN_NEXT,
  VICTIM_DETECT,
  EXECUTE_MOVE,
  BOTCHED_TURN_RECOVERY,
  BOTCHED_FWD_RECOVERY,
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
// obstacle toggle
bool obstacleright = false;
bool obstacleleft = false;
// victim toggles
bool victimtoggle = false;
bool victimAtCurrent = false;
// camera GPIOs
const int gpio1 = 13;
const int gpio2 = 12;
// stepper variables
const int angle_offset = 44;
const int angle_increment = 22;
dispenser disp(angle_increment,angle_offset,steps_per_revolution);
// logic switch pin
const int logicswitch = 22;
volatile bool Pausemaze = false; // set by pauseThread, read by loop()
int x_checkpoint = MAP_SIZE/2, y_checkpoint = MAP_SIZE/2;
int floor_checkpoint = 0; // floor the last checkpoint was recorded on (0..2)
bool tilecheck = false;

// Forward declaration: Arduino can't auto-prototype template return types.
std::deque<std::pair<int, std::pair<int,int>>> BFS(std::pair<int, std::pair<int,int>> currentpos, Grid& m1, Grid& m2, Grid& m3, std::pair<int, std::pair<int,int>> endpos, bool allowBlue);

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
volatile bool fwdActive = false; // true only while inside fwd()
volatile bool turnActive = false;
volatile bool victimPending = false; // a camera reported -> movement must service it
volatile int  victimSide = 0;        // 1 = left (Serial3), 2 = right (Serial2)
volatile bool isVictim = false;      // a victim already handled during current move

rtos::Thread cameraThread;
rtos::Mutex i2cMutex;
rtos::Mutex lcdMutex; // lcd mutex to prevent conflict
void cameraTask(){
  while(true){
    int encoderCount = (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3;
    int nx = x_pos; int ny=y_pos;
    if((fwdActive||turnActive) && !victimPending && !isVictim){
      
      //if(encoderCount>=0.3*pulsesForDistanceMm(TILE_MM)||encoderCount<=0.7*pulsesForDistanceMm(TILE_MM)){
        if(readSerial1() != -1){        // left camera (Serial4)
          if(fwdActive) victimTileFromEncoder(TILE_MM,encoderCount,nx,ny);
          Serial.println("nx, ny");
          Serial.println(nx);
          Serial.println(ny);
          Serial.println(mapGrid[nx][ny].getVictim());
          if(mapGrid[nx][ny].getVictim() == false){
            i2cMutex.lock();
            victimSide = 1;
            drivetrain.fullstop();
            victimPending = true;
            i2cMutex.unlock();
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
            i2cMutex.lock();
            serviceCameraVictim();
            i2cMutex.unlock();
          }
        }
        else if(readSerial2() != -1){   // right camera (Serial3)
          if(fwdActive) victimTileFromEncoder(TILE_MM,encoderCount,nx,ny);
          Serial.println("nx, ny");
          Serial.println(nx);
          Serial.println(ny);
          Serial.println(mapGrid[nx][ny].getVictim());
          if(mapGrid[nx][ny].getVictim() == false){
            i2cMutex.lock();
            victimSide = 2;
            drivetrain.fullstop();
            victimPending = true;
            i2cMutex.unlock();
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
            i2cMutex.lock();
            serviceCameraVictim();
            i2cMutex.unlock();
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
    
    if(digitalRead(logicswitch)==HIGH){
      
      Pausemaze = true;
    }
    else{
      
      Pausemaze = false;
    }
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
  // initialize camera gpio pins
  pinMode(gpio1, INPUT);
  pinMode(gpio2, INPUT);
  // initialize logic switch pin
  pinMode(logicswitch, INPUT);
  // begin UART communication.
  Serial.begin(115200);
  Serial3.begin(115200); // switch to 9600 for reliability
  Serial4.begin(115200);
  
  
  Wire.begin();
  disableAllCall();
  myMux.begin();
  init_dist(); // initialize mux before distance sensors.
  scanAllPorts();
  init_color();
  init_drive();
  //detect();
  //initialize map
  initializeMap(); // initialize mapgrid
  // every floor starts as a copy of the freshly initialized (empty) grid.
  m1 = mapGrid;
  m2 = mapGrid;
  m3 = mapGrid;
  currentFloor = 0;
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].setDiscovered(true);
  currentDir = NORTH;
  state = SENSE_TILE;
  // start lcd
  lcd.begin(16, 2);
  // start RTOS threads: camera victim detection + pause-switch watcher.
  cameraThread.start(cameraTask);
  cameraThread.set_priority(osPriorityAboveNormal);
  pauseThread.start(pauseTask);
  //Serial.println("starting");
  delay(2000); // wait for camera to start.
  
}
int iterator = 0;

void loop(){
  /*
  for(int i = 1;i<=7;i++){
    Serial.print("sensor ");
    Serial.println(i);
    Serial.println(measure(i));
    delay(500);
  }
  */
  //Serial.println(read_color());

  //lcdPrint("working");
  //delay(500);
  
  static bool wallF, wallR, wallB, wallL;
  switch (state) {
    case SENSE_TILE: {
      // reset per-tile toggles
      blacktoggle = false; bluetoggle = false; victimtoggle = false;
      // Read for walls
      readWallsRel(wallF, wallR, wallB, wallL);
      delay(200);
      state = UPDATE_MAP; // next state.
      // Auto-trigger front-back centering: only when a front wall is
      // present, off-center beyond CENTER_TOL_MM, and the offset isn't so
      // large (>= one tile) that the reading is unreliable. Back-wall
      // centering isn't implemented yet, so wallB is not checked here.
      if(wallF == true){
        int front1 = measure(1);
        int front7 = measure(7);
        if(front1 != -1 && front7 != -1){
          double frontGap = (front1 + front7) / 2.0;
          double offset = frontGap - TARGET_GAP_MM;
          if(abs(offset) > CENTER_TOL_MM && abs(offset) < MAX_CENTER_CORRECTION_MM) state = CENTERING;
        }
      }
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case CENTERING: {
      centerFrontBack();
      state = UPDATE_MAP;
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
      
      
      state = PLAN_NEXT;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case PLAN_NEXT: {
      Serial.println("plan next");
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
        if(bluetoggle == true){
          delay(5000);
          mapGrid[x_pos][y_pos].setType(BLUE);
        }
      }
      else{
        
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
      if(iterator >= 25) state = RETURN;
      break;
    }
    case BACKPEDAL: {
      plannedMoveDir = pickNextDirection();
      Serial.println("next direction picked");
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      state = EXECUTE_MOVE;
      blacktoggle = false;
      if(Pausemaze == true) state = PAUSE;
      delay(200);
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
      // in case of no elevation used, m1,m2,m3 are all blank grids.
      // let the current floor grid be mapgrid.
      if(currentFloor == 0)      m1 = mapGrid;
      else if(currentFloor == 1) m2 = mapGrid;
      else if(currentFloor == 2) m3 = mapGrid;
      // currentFloor is already 0-indexed (0..2), matching BFS's floor arrays.
      std::pair<int, std::pair<int, int>> currentpos = {currentFloor, {x_pos, y_pos}};
      std::pair<int, std::pair<int, int>> endpos     = {0, {MAP_SIZE/2, MAP_SIZE/2}};

      lcdPrint("starting bfs");
      
      Serial.println("starting bfs");
      std::deque<std::pair<int, std::pair<int,int>>> path = BFS(currentpos, m1, m2, m3, endpos, false);
      if(path.empty()){
        lcdPrint("blue allowed");
        path = BFS(currentpos, m1, m2, m3, endpos, true);
      }
      if(path.empty()){
        lcdPrint("no path found");
        while(true) drivetrain.fullstop();
      }
      Serial.println("path calculated");
      // path[0]=currentpos, path[last]=endpos — iterate forward toward home
      for(int i = 0; i < (int)path.size() - 1; i++){
        Direction moveDir;
        int dx = path[i+1].second.first  - path[i].second.first;
        int dy = path[i+1].second.second - path[i].second.second;
        if(dy == 0) moveDir = (dx == 1) ? EAST : WEST;
        else        moveDir = (dy == 1) ? NORTH : SOUTH;

        plannedTurnDeg = turnNeededDeg(moveDir);
        absoluteturn(plannedTurnDeg);
        delay(200);
        parallel();
        delay(100);
        currentDir = moveDir;
        fwd(TILE_MM);

        // track floor changes: update currentFloor and swap the active grid
        int dz = path[i+1].first - path[i].first;
        if(dz > 0){
          currentFloor++;
          mapGrid = (currentFloor == 1) ? m2 : m3;
        }
        else if(dz < 0){
          currentFloor--;
          mapGrid = (currentFloor == 0) ? m1 : m2;
        }
      }
      
      while(true){
        drivetrain.fullstop();
        lcdPrint("back to start");
      }
    }
    case PAUSE: {
      drivetrain.fullstop();
      delay(200);
      if(digitalRead(logicswitch)==LOW){
        Pausemaze = false;
        // Restore the checkpoint's FLOOR as well as its tile. Save the grid we
        // were working on back into its floor slot (m1=floor0, m2=floor1,
        // m3=floor2), then load the checkpoint floor's grid as the active grid
        // so victim flags / walls are looked up on the correct floor. (Without
        // this, resuming after a ramp left mapGrid pointing at the wrong floor.)
        if(currentFloor == 0)      m1 = mapGrid;
        else if(currentFloor == 1) m2 = mapGrid;
        else if(currentFloor == 2) m3 = mapGrid;
        currentFloor = floor_checkpoint;
        if(currentFloor == 0)      mapGrid = m1;
        else if(currentFloor == 1) mapGrid = m2;
        else if(currentFloor == 2) mapGrid = m3;
        x_pos = x_checkpoint; y_pos = y_checkpoint; // resume from last checkpoint
        Direction snapped = (Direction)myGyro.headingToCardinal(myGyro.heading()); // snap to cardinal
        absoluteturn(turnNeededDeg(snapped));
        currentDir = snapped;
        
        Serial.println("checkpoint coordinates");
        Serial.println(x_checkpoint);
        Serial.println(y_checkpoint);
        Serial.println(currentDir);
        state = PLAN_NEXT;
      }
      break;
    }
 }
 
 
}
