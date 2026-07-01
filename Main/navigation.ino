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
  int encoderCount = (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3;
  int victimX, victimY;
  victimTileFromEncoder(distanceMm, encoderCount, victimX, victimY);
  if(!inBounds(victimX, victimY)) return;

  mapGrid[victimX][victimY].setVictim(true);
  //mapGrid[victimX][victimY].setDiscovered(true);

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
// necessary for picknextdirection.
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
  t.setDiscovered(true); // tile is discovered(walls are found)
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
    return mapGrid[nx][ny].getType() == BLACK || isBlueTile(nx, ny);
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
    if (inBounds(nx, ny) && open(d) && !isBlackTile(nx,ny)) return d;
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
  if(floor == 0){
    m1 = mapgrid;
    mapgrid = m2;
  } else if(floor == 1){
    m2 = mapgrid;
    mapgrid = m3;
  }
  mapgrid[xpos][ypos].setDescend(true); // bidirection elevate/descend
  markEdgeBothWays(x_pos, y_pos, currentDir);
  writeWallsToCurrentTile(0, 1, 0, 1);
  updateFullyExploredAt(x_pos, y_pos);
  floor++;
}

// Move down one floor.
void descend(Grid& mapgrid, int xpos, int ypos, Grid& m1, Grid& m2, Grid& m3, int& floor){
  mapgrid[xpos][ypos].setDescend(true);
  if(floor == 0){
    m1 = m3; //m3 should always be empty if descended twice
    m3 = m2;
    m2 = m1;
    mapgrid = m1;
    floor++;
  }
  else if(floor == 1){
    m2 = mapgrid;
    mapgrid = m1;
  }
  else if(floor == 2){
    m3 = mapgrid;
    mapgrid = m2;
  }
  mapgrid[xpos][ypos].setElevate(true);
  markEdgeBothWays(x_pos, y_pos, currentDir);
  writeWallsToCurrentTile(0, 1, 0, 1);
  updateFullyExploredAt(x_pos, y_pos);
  floor--;
}

// old 2d bfs
// pair structure
/*
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
                    mapGrid[nx][ny].getType() != BLACK) { //IMPORTANT: ADD MORE CONDITIONALS HERE
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
*/
// Compact BFS node. uint8_t is safe because MAP_SIZE (40) and floors (3) both
// fit easily; keeps the static scratch arrays small.
struct BfsNode { uint8_t z, x, y; }; // 3d coords by claude?

// allowBlue: if true, BLUE tiles are traversable (fallback mode).
// Returns empty deque if endpos is unreachable under the given constraints.
//
// Memory note: this used to copy all three floor grids into a ~37.5 KB local
// stack array and allocate ~60 KB of nested std::vector on the heap *per call*,
// which hard-faulted / fragmented the Giga's D1 SRAM. Now the grids are indexed
// through pointers (no copy) and all scratch lives in fixed static .bss arrays
// reserved once at link time (visited ~4.7 KB, prev ~14 KB, queue ~14 KB). The
// only per-call heap use is the returned path, which is just the route length.
std::deque<std::pair<int, std::pair<int,int>>> BFS(std::pair<int, std::pair<int, int>> currentpos, Grid& m1, Grid& m2, Grid& m3, std::pair<int, std::pair<int, int>> endpos, bool allowBlue = false) {
    Grid* map[3] = { &m1, &m2, &m3 };  // index, don't copy

    static bool    visited[3][MAP_SIZE][MAP_SIZE];
    static BfsNode prev[3][MAP_SIZE][MAP_SIZE];
    static BfsNode queue[3 * MAP_SIZE * MAP_SIZE]; // each node enqueued once -> never overflows
    memset(visited, 0, sizeof(visited));
    int head = 0, tail = 0;

    BfsNode start = { (uint8_t)currentpos.first, (uint8_t)currentpos.second.first, (uint8_t)currentpos.second.second };
    int ez = endpos.first, ex = endpos.second.first, ey = endpos.second.second;

    // already at the goal: return a trivial path. Guards the reconstruction loop
    if (start.z == ez && start.x == ex && start.y == ey) {
        return { currentpos };
    }

    queue[tail++] = start;
    visited[start.z][start.x][start.y] = true;
    while (head < tail) {
        BfsNode cur = queue[head++];
        int x = cur.x, y = cur.y, z = cur.z;

        for (int i = 0; i < 4; i++) {
            int nx = x + dir[i][0];
            int ny = y + dir[i][1];
            int nz = z;

            if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) {
                // floor change: a neighbor tile flagged elevate/descend is a ramp
                // entry. Index (*map[z])[nx][ny] only after the bounds check above,
                // and clamp nz to the valid floor range [0,2] so map[nz]/visited[nz]
                // can never go out of bounds.
                if ((*map[z])[nx][ny].getElevate() && z + 1 < 3) nz = z + 1;
                else if ((*map[z])[nx][ny].getDescend() && z - 1 >= 0) nz = z - 1;

                bool passable = !(*map[z])[x][y].getWall((Direction)i) &&
                                !(*map[nz])[nx][ny].getWall(opposite((Direction)i)) &&
                                (*map[nz])[nx][ny].getDiscovered() &&
                                (*map[nz])[nx][ny].getType() != BLACK;
                if (!allowBlue) {
                    passable = passable && (*map[nz])[nx][ny].getType() != BLUE;
                }

                if (!visited[nz][nx][ny] && passable) {
                    visited[nz][nx][ny] = true;
                    prev[nz][nx][ny] = cur;
                    queue[tail++] = { (uint8_t)nz, (uint8_t)nx, (uint8_t)ny };
                }
            }
        }
    }

    // endpos unreachable under current constraints — return empty path
    if (!visited[ez][ex][ey]) {
        return {};
    }

    // reconstruct: walk backward from endpos to start via prev[], push_front
    // so path[0]=currentpos, path[last]=endpos
    std::deque<std::pair<int, std::pair<int,int>>> path;
    BfsNode curr = { (uint8_t)ez, (uint8_t)ex, (uint8_t)ey };
    while (!(curr.z == start.z && curr.x == start.x && curr.y == start.y)) {
        path.push_front({ curr.z, { curr.x, curr.y } });
        curr = prev[curr.z][curr.x][curr.y];
    }
    path.push_front(currentpos);
    return path;
}
