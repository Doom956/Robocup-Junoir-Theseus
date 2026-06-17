// navigation.ino — claude version 6/16/2026
// Ported from multicore_version (M7_main/navigation.ino) into the single-core
// main branch. All RPC.call(...) cross-core calls have been replaced with the
// original single-core functions (drivetrain.encoderCountA, detectWall, ...).
// Adds the multi-floor elevation features (m1/m2/m3, elevation/descend) and the
// encoder-based victim tile marking, while keeping main's working BFS.

// I assume is global?
Direction rotateDir(Direction base, int offset) {
  return (Direction)((base + offset + 4) % 4);
}
// at orientation w, conver heading of x to local heading of y.

void stepForward(Direction d, int &x, int &y) {
  // global direction
  if (d == NORTH) y++;
  else if (d == EAST) x++;
  else if (d == SOUTH) y--;
  else if (d == WEST) x--;
}
// pulses for distance(mm)
double pulsesForDistanceMm(double distanceMm) {
  return distanceMm / (wheel_diameter * M_PI) * wheel_cpr * gear_ratio;
}
// which tile victim is depending on encoder.
void victimTileFromEncoder(int distanceMm, int encoderCount, int &victimX, int &victimY) {
  victimX = x_pos;
  victimY = y_pos;

  double tilePulses = pulsesForDistanceMm(distanceMm); // total pulses to traverse a tile.
  if(abs(encoderCount) >= tilePulses / 2.0){
    stepForward(currentDir, victimX, victimY); // past tile midpoint -> victim belongs to the next tile.
  }
}
// mark victim of tile based on encoder position (single-core: read encoder directly)
void markVictimAtEncoderPosition(int distanceMm) {
  int encoderCount = drivetrain.encoderCountA;
  int victimX, victimY;
  victimTileFromEncoder(distanceMm, encoderCount, victimX, victimY);
  if(!inBounds(victimX, victimY)) return;

  mapGrid[victimX][victimY].setVictim(true);
  mapGrid[victimX][victimY].setDiscovered(true);

  Serial.print("marked victim at tile x=");
  Serial.print(victimX);
  Serial.print(", y=");
  Serial.print(victimY);
  Serial.print(", encoderA=");
  Serial.println(encoderCount);
}

bool inBounds(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      mapGrid[x][y].setDiscovered(false);
      mapGrid[x][y].setFully(false);
      mapGrid[x][y].setVisited(false);
      mapGrid[x][y].setElevate(false);
      mapGrid[x][y].setDescend(false);
      for (int d = 0; d < 4; d++) {
        mapGrid[x][y].setWall(d, false);
        mapGrid[x][y].setEdge(d, false);
      }
      mapGrid[x][y].setType(BLANK);
    }
  }
}

// mark traveled edge in BOTH tiles (current and next)
void markEdgeBothWays(int x, int y, Direction d) {
  int nx = x, ny = y;
  stepForward(d, nx, ny);
  if (!inBounds(nx, ny)) return;

  mapGrid[x][y].setEdge(d, true); // connected
  mapGrid[nx][ny].setEdge(opposite(d), true); // update both sides.
}

// update fullyExplored = all OPEN dirs have been traveled at least once
void updateFullyExploredAt(int x, int y) {
  Tile &t = mapGrid[x][y];
  bool allDone = true;
  t.setVisited(true);
  for (int d = 0; d < 4; d++) {
    if (t.getWall(d) == false) {     // open
      if (t.getEdge(d) == false) {   // not traveled yet
        allDone = false;
        break;
      }
    }
  }
  t.setFully(allDone);
}
// 0=front, 1=right, 2=back, 3=left
void readWallsRel(bool &wallF, bool &wallR, bool &wallB, bool &wallL) { // references needed here to update the variable values
  wallF = (detectWall(0)==0);
  wallR = (detectWall(1)==0);
  wallB = (detectWall(2)==0);
  wallL = (detectWall(3)==0);
  Serial.println(wallF);
  Serial.println(wallR);
  Serial.println(wallB);
  Serial.println(wallL);
}
//get the wall from L,R(local) into N W(global)
// absF is the absolute heading the the robot front is heading.
void writeWallsToCurrentTile(bool wallF, bool wallR, bool wallB, bool wallL) {
  Tile &t = mapGrid[x_pos][y_pos];
  t.setDiscovered(true);
  // shouldn't absolute directions be north south east west?
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  Direction absL = rotateDir(currentDir, -1);
  t.setWall(absF, wallF);
  t.setWall(absR, wallR);
  t.setWall(absB, wallB);
  t.setWall(absL, wallL);
  // need to mark both ways.
}
Direction pickNextDirection() {
  Tile &t = mapGrid[x_pos][y_pos];

  Direction absL = rotateDir(currentDir, -1);
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  // Plan directly in absolute map directions.
  const Direction priority[3] = {absF,absR, absL};

  auto open  = [&](Direction d){ return t.getWall(d) == false; };
  auto untr  = [&](Direction d){ return t.getEdge(d) == false; };
  auto isBlueTile = [&](int nx, int ny){
    return mapGrid[nx][ny].getType() == BLUE || mapGrid[nx][ny].getBlue();
  };
  auto blockedForTravel = [&](int nx, int ny){
    return mapGrid[nx][ny].getType() == BLACK || mapGrid[nx][ny].getType() == STAIR || isBlueTile(nx, ny);
  };
  auto isBlackTile = [&](int nx, int ny){
    return mapGrid[nx][ny].getType() == BLACK;
  };

  // 1) try open + untraveled first
  for (int i = 0; i < 3; i++) {
    int nx = x_pos, ny = y_pos;

    Direction d = priority[i];
    stepForward(d,nx,ny);

    if (inBounds(nx, ny) && open(d) && untr(d) && !mapGrid[nx][ny].getVisited() && !blockedForTravel(nx, ny)) return d;
  }

  // 2) else any open (still avoid black/blue) using the same absolute priority.
  for (int i = 0; i < 3; i++) {
    int nx = x_pos, ny = y_pos;
    Direction d = priority[i];
    stepForward(d,nx,ny);
    if (inBounds(nx, ny) && open(d) && !blockedForTravel(nx, ny)) return d;
  }
  for (int i=0;i<3;i++){
    int nx = x_pos, ny = y_pos;
    Direction d = priority[i];
    stepForward(d,nx,ny);
    if (inBounds(nx, ny) && open(d) && !isBlackTile(nx,ny) && mapGrid[nx][ny].getType() != STAIR) return d;
  }

  // figure out BFS later
  // 3) trapped
  return absB;
}

int turnNeededDeg(Direction direction) {
  // Convert an absolute direction enum to an absolute heading angle.
  if (direction == 0) return 0;
  if (direction == 1) return 90;
  if (direction == 2) return 180;
  return 270; // diff==3

}
int dir[4][2] = {
    {0, 1},
    {1, 0},
    {0, -1},
    {-1, 0}
};
void initTile(int x, int y, Grid& map) {
    map[x][y].setDiscovered(false);
    map[x][y].setFully(false);
    map[x][y].setVisited(false);
    map[x][y].setElevate(false);
    map[x][y].setDescend(false);
    for (int d = 0; d < 4; d++) {
        map[x][y].setWall(d, false);
        map[x][y].setEdge(d, false);
    }
    map[x][y].setType(BLANK);
}

void reallocate(Grid& mapgrid, int pos_x = 0, int pos_y = 0) { //input mapgrid, and next tile location
    //cout << "start reallocate" << endl;

    //expand to bottom (remove top)
    if (pos_y >= MAP_SIZE) {
        for (int i = 0; i < MAP_SIZE - 1; i++) {
            for (int j = 0; j < MAP_SIZE; j++) {
                mapgrid[i][j] = mapgrid[i + 1][j];
                m1[i][j] = m1[i + 1][j];
                m2[i][j] = m2[i + 1][j];
                m3[i][j] = m3[i + 1][j];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(MAP_SIZE - 1, i, mapgrid);
            initTile(MAP_SIZE - 1, i, m1);
            initTile(MAP_SIZE - 1, i, m2);
            initTile(MAP_SIZE - 1, i, m3);
        }
    }
    //expand to top (remove bottom)
    else if (pos_y < 0) {
        for (int i = MAP_SIZE-1; i > 0; i--) {
            for (int j = 0; j < MAP_SIZE; j++) {
                mapgrid[i][j] = mapgrid[i - 1][j];
                m1[i][j] = m1[i - 1][j];
                m2[i][j] = m2[i - 1][j];
                m3[i][j] = m3[i - 1][j];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(0, i, mapgrid);
            initTile(0, i, m1);
            initTile(0, i, m2);
            initTile(0, i, m3);
        }
    }

    //expand to right (remove left)
    else if (pos_x >= MAP_SIZE) {
        for (int j = 0; j < MAP_SIZE - 1; j++) {
            for (int i = 0; i < MAP_SIZE; i++) {
                mapgrid[i][j] = mapgrid[i][j+1];
                m1[i][j] = m1[i][j+1];
                m2[i][j] = m2[i][j+1];
                m3[i][j] = m3[i][j+1];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(i, MAP_SIZE-1, mapgrid);
            initTile(i, MAP_SIZE-1, m1);
            initTile(i, MAP_SIZE-1, m2);
            initTile(i, MAP_SIZE-1, m3);
        }
    }

    //expand to left (remove right)
    else if (pos_x < 0) {
        for (int j = MAP_SIZE - 1; j > 0; j--) {
            for (int i = 0; i < MAP_SIZE; i++) {
                mapgrid[i][j] =  mapgrid[i][j - 1];
                m1[i][j] = m1[i][j-1];
                m2[i][j] = m2[i][j-1];
                m3[i][j] = m3[i][j-1];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(i, 0, mapgrid);
            initTile(i, 0, m1);
            initTile(i, 0, m2);
            initTile(i, 0, m3);
        }
    }
}

// Move up one floor. The current floor grid is saved into the appropriate
// storage grid (m1/m2/m3) and the active mapgrid is swapped to the floor above.
void elevation(Grid& mapgrid, int xpos, int ypos, Grid& m1, Grid& m2, Grid& m3, int& floor){
  mapgrid[xpos][ypos].setElevate(true);
  if(floor == 1){
    m1 = mapgrid;
    mapgrid = m2;
  } else if(floor == 2){
    m2 = mapgrid;
    mapgrid = m3;
  }
  mapgrid[xpos][ypos].setDescend(true);
  floor++;
}

// Move down one floor.
void descend(Grid& mapgrid, int xpos, int ypos, Grid& m1, Grid& m2, Grid& m3, int& floor){
  mapgrid[xpos][ypos].setDescend(true);
  if(floor == 1){
    m1 = m3; //m3 should always be empty if descended twice
    m3 = m2;
    m2 = m1;
    mapgrid = m1;
    floor++;
  }
  if(floor == 2){
    m2 = mapgrid;
    mapgrid = m1;
  }
  if(floor == 3){
    m3 = mapgrid;
    mapgrid = m2;
  }
  mapgrid[xpos][ypos].setElevate(true);
  floor--;
}

// pair structure
int BFS(coord currentpos, Grid& mapGrid, coord endpos, coord path[MAP_SIZE * MAP_SIZE]) { // auto updates path
    ArduinoQueue<coord> queue = {};
    size_t rows = MAP_SIZE;
    size_t columns = MAP_SIZE;
    bool visited[MAP_SIZE][MAP_SIZE] = {false};
    coord prev[MAP_SIZE][MAP_SIZE];
    queue.enqueue(currentpos); // current tile
    visited[currentpos.x][currentpos.y] = true;
    //search
    while (queue.itemCount() > 0) {
        int x = queue.getHead().x; int y = queue.getHead().y;
        //cout << "visting: " << x << "," << y << endl;

        for (int i = 0; i < 4; i++) {
            int nx = x + dir[i][0];
            int ny = y + dir[i][1];
            if (nx < rows && ny < columns && nx >= 0 && ny >= 0) {
                if (!visited[nx][ny] &&
                    !mapGrid[x][y].getWall((Direction)i) &&
                    !mapGrid[nx][ny].getWall(opposite((Direction)i)) &&
                    mapGrid[nx][ny].getDiscovered() &&
                    mapGrid[nx][ny].getType() != BLACK &&
                    mapGrid[nx][ny].getType() != STAIR) { //IMPORTANT: ADD MORE CONDITIONALS HERE
                    queue.enqueue(coord{nx, ny}); // add tile
                    visited[nx][ny] = true;

                    prev[nx][ny] = coord{x, y};

                }
            }
        }
        queue.dequeue();
    }
    // Check endpos was actually reached before reconstructing
    if (!visited[endpos.x][endpos.y]) {
      return 0; // endpos unreachable — caller must handle empty path
    }
    //reconstruct path
    // path[0] is endpos, path[n] is current tile.
    int i = 0;
    coord curr = endpos;
    while (true) {
      path[i] = curr;
      i++;

      if (curr.x == currentpos.x&&curr.y==currentpos.y) {
        break;
      }
      curr = prev[curr.x][curr.y];
    }
    return i;

}
