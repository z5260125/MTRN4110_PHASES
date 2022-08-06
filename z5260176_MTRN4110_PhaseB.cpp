// File:          z5260176_MTRN4110_PhaseB.cpp
// Date:
// Description:   MTRN4110 Phase B - Path Planning
// Author:        z5260176
// Modifications:
// Platform:      Windows

#include <webots/Robot.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <list>
#include <string>
#include <algorithm>

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

const std::string MAP_FILE_NAME = "../../Map.txt";
const std::string OUTPUT_FILE_NAME = "../../Output.txt";
const std::string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string prefix = "[z5260176_MTRN4110_PhaseB] ";
const std::string hBorder = " --- --- --- --- --- --- --- --- --- ";

// Continue writing to file
void writeToOutputFile(std::string output, bool startOfFile) {
  std::ofstream inFile;
  if (startOfFile) {
    inFile.open(OUTPUT_FILE_NAME);
  } else {
    inFile.open(OUTPUT_FILE_NAME, std::ios::in | std::ios::app);
  }
  inFile << output;
  inFile.close();
}

void writeToPathPlanFile(std::string output) {
  std::ofstream inFile;
  inFile.open(PATH_PLAN_FILE_NAME);
  inFile << output;
  inFile.close();
}

std::string readMap() {
  std::ifstream inFile(MAP_FILE_NAME);
  std::string map;
  std::string output;
  if (inFile.is_open()) {
    output.append(prefix);
    output.append("Reading in map from ../../Map.txt...\n");
    std::string str;
    while (std::getline(inFile, str)) {
      map.append(str);
      output.append(prefix);
      output.append(str);
      output.append("\n");
    }
    output.append(prefix);
    output.append("Map read in!\n");
    std::cout << output;
  } else {
    throw std::runtime_error(prefix + " Reading data failed!");
  }
  inFile.close();
  writeToOutputFile(output, true);
  map.erase(0, 37); // delete first line (border)
  return map;
}

std::vector<std::vector<int>> findHorizontalWalls(std::string map) {
  map.erase(0, 38); // skip to next line
  std::vector<std::vector<int>> hWallArray(4, std::vector<int>(9));
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 9; j++) {
      std::string wall = map.substr(0,3);
      if (wall.compare("---") == 0) {
        hWallArray[i][j] = 1;  
      }
      map.erase(0,4);
      if (j == 8) {
        map.erase(0,38);
      }
    }
  }
  return hWallArray;
}

std::vector<std::vector<int>> findVerticalWalls(std::string map) {
  map.erase(0,4);
  std::vector<std::vector<int>> vWallArray(5, std::vector<int>(8));
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 8; j++) {
      std::string wall = map.substr(0,1);
      if (wall.compare("|") == 0) {
        vWallArray[i][j] = 1;
      }
      map.erase(0,4);
      if (j == 7) {
        map.erase(0, 42);
      }
    }
  }
  return vWallArray;
}

// still need to store heading of starting position
std::vector<int> getStartingAndTargetPos(std::string map) {
  map.erase(0,1);
  std::vector<int> pos = {-1,-1,-1};
  for (int i = 0; i < 45; i++) {
    std::string cell = map.substr(0,3);
    if (cell.compare(" ^ ") == 0) {
      pos[0] = i;
      pos[2] = NORTH;
    } else if (cell.compare(" > ") == 0) {
      pos[0] = i;
      pos[2] = EAST;
    } else if (cell.compare(" v ") == 0) {
      pos[0] = i;
      pos[2] = SOUTH;
    } else if (cell.compare(" < ") == 0) {
      pos[0] = i;
      pos[2] = WEST;
    } else if (cell.compare(" x ") == 0) {
      pos[1] = i;
    }
    map.erase(0,4);
    if ((i+1) % 9 == 0) {
      map.erase(0,38);
    }
  }
  return pos;
}

void addEdge(std::vector<int> graph[], int a, int b) {
  graph[a].push_back(b);
  graph[b].push_back(a);
}

void createGraph(std::vector<int> adj[], std::vector<std::vector<int>> hWallArray, 
std::vector<std::vector<int>> vWallArray) {
  int node = 0;
  // connect graph horizontally
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 8; j++) {
      if ((node+1) % 9 == 0) {
        node++;
      }
      if (!vWallArray[i][j]) {
        addEdge(adj, node, node+1);
      }
      node++;
    }
  }
  // connect graph vertically
  node = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 9; j++) {
      if (!hWallArray[i][j]) {
        addEdge(adj, node, node+9);
      }
      node++;
    }
  }
}

void bfs(std::vector<int> graph[], std::vector<int> prev[], int v, int src) {
  std::queue<int> queue;
  //stores the shortest distance to src to every other node
  std::vector<int> dist(v, INT_MAX);
  
  dist[src] = 0;
  prev[src] = {-1};
  //push source node onto queue
  queue.push(src);
  
  while (!queue.empty()) {
    int node = queue.front();
    queue.pop();
    //loop through all adjacent vertices to node
    for (int adj : graph[node]) {
      // If dist from src to adj through node is shorter than adj's current prev
      if (dist[adj] > dist[node] + 1) {
        // Replace previous path of adjacent with previous path of node
        prev[adj].clear();
        prev[adj].push_back(node);
        dist[adj] = dist[node] + 1; // update new shorter distance
        queue.push(adj);
      } else if (dist[adj] == dist[node] + 1) {
        // Alternate path has been found
        prev[adj].push_back(node);
      }
    }
  }
}

void recurse(std::vector<std::vector<int>> &paths, std::vector<int> &path,
std::vector<int> prev[], int src, int dest) {
  // when destination reaches source
  if (dest == src) {
    paths.push_back(path);
    return;
  }
  // recursively find the previous nodes
  for (int p : prev[dest]) {
    path.push_back(dest);
    recurse(paths, path, prev, src, p);
    path.pop_back();
  }
}

void printMapInConsole(std::string cellArr[5][9], std::vector<std::vector<int>> vWallArray, 
std::vector<std::vector<int>> hWallArray, std::vector<int> pos) {
  std::string output;
  output.append(prefix);
  output.append(hBorder);
  output.append("\n");
  for (int i = 0; i < 5; i++) {
    output.append(prefix);
    output.append("|");
    for (int j = 0; j < 8; j++) {
      output.append(cellArr[i][j]);
      if (vWallArray[i][j]) {
        output.append("|");
      } else {
        output.append(" ");
      }
    }
    output.append(cellArr[i][8]);
    output.append("|\n");
    // print horizontal walls
    if (i == 4) {
      break;
    }
    output.append(prefix);
    for (int k = 0; k < 9; k++) {
      if (hWallArray[i][k]) {
        output.append(" ---");
      } else {
        output.append("    ");
      }
    }
    output.append(" \n");
  }
  output.append(prefix);
  output.append(hBorder);
  output.append("\n");
  std::cout << output;
  writeToOutputFile(output, false);
}

void printPath(std::vector<int> path, std::vector<std::vector<int>> hWallArray, 
std::vector<std::vector<int>> vWallArray, std::vector<int> pos) {
  std::string cellArr[5][9];
  // check if cell is in path, then mark with path location
  int x = 0;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 9; j++) {
      cellArr[i][j] = "   ";
      int index = 0;
      for (int n : path) {
        if (n == x) {
          cellArr[i][j] = " " + std::to_string(index);
          if (index < 10) {
            cellArr[i][j].append(" ");
          }
        } else if (x == pos[0]) {
          if (pos[2] == NORTH) {
            cellArr[i][j] = " ^ ";
          } else if (pos[2] == EAST) {
            cellArr[i][j] = " > ";
          } else if (pos[2] == SOUTH) {
            cellArr[i][j] = " v ";
          } else if (pos[2] == WEST) {
            cellArr[i][j] = " < ";
          }
        }
        index++;
      }
      x++;
    }
  }
  printMapInConsole(cellArr, vWallArray, hWallArray, pos);
}

void printShortestPaths(std::vector<std::vector<int>> paths, std::vector<std::vector<int>> hWallArray,
std::vector<std::vector<int>> vWallArray, std::vector<int> pos) {
  std::string output;
  int pathNumber = 0;
  for (auto p : paths) {
    pathNumber++;
    output = prefix;
    output.append("Path - ");
    output.append(std::to_string(pathNumber));
    output.append(":\n");
    std::cout << output;
    writeToOutputFile(output, false);
    printPath(p, hWallArray, vWallArray, pos);
  }
  output = prefix;
  output.append(std::to_string(pathNumber));
  output.append(" shortest paths found!\n");
  output.append(prefix);
  output.append("Finding shortest path with least turns...\n");
  std::cout << output;
  writeToOutputFile(output, false);
}

// Returns the direction of the next node with respect to the current node
int updateDirection(int direction, int curr, int next) {
  if (curr - next == 9) {
    return NORTH;
  } else if (curr - next == -1) {
    return EAST;
  } else if (curr - next == -9) {
    return SOUTH;
  }
  return WEST;
}

// Get the number of turns to get from current node and direction to the next position
int getNumberOfTurns(int direction, int curr, int next) {
  int newDir = updateDirection(direction, curr, next);
  // Go forward - no turns
  if (direction == newDir) {
    return 0;
  // Opposite direction - turn twice
  } else if (direction - newDir == 2 || direction - newDir == -2) {
    return 2;
  }
  // Turn right or left
  return 1;
}

// Loops through all the paths and finds the motion plan for each path
std::vector<int> findShortestPathLeastTurns(std::vector<std::vector<int>> paths, std::vector<int> pos) {
  int x = 0, leastTurns = INT_MAX;
  std::vector<int> shortestPath;
  for (auto path : paths) {
    int direction = pos[2], turns = 0;
    turns += getNumberOfTurns(direction, pos[0], path[0]);
    direction = updateDirection(direction, pos[0], path[0]);
    for (int i = 0; i < int(path.size() - 1); i++) {
      turns += getNumberOfTurns(direction, path[i], path[i+1]);
      direction = updateDirection(direction, path[i], path[i+1]);
    }
    if (turns < leastTurns) {
      leastTurns = turns;
      shortestPath = path;
    }
    x++;
  }
  return shortestPath;
}

// Returns the motion sequence based on the current and next nodes
std::string getNewMotion(int direction, int curr, int next) {
  int newDirection = updateDirection(direction, curr, next);
  // Directions are the same - go forward
  if (direction == newDirection) {
    return "F";
  // Opposite direction - turn left twice then go forward
  } else if (direction - newDirection == 2 || direction - newDirection == -2) {
    return "LLF";
  // Right turn and go forward
  } else if (newDirection - direction == 1 || newDirection - direction == -3) {
    return "RF";
  }
  // Left turn
  return "LF";
}

// Given the shortest path with the least turns, returns the motion plan as a string
std::string findMotionPlan(std::vector<int> path, std::vector<int> pos) {
  std::string motionPlan;
  int direction = pos[2];
  std::reverse(path.begin(), path.end());
  motionPlan.append(getNewMotion(direction, pos[0], path[0]));
  direction = updateDirection(direction, pos[0], path[0]);
  for (int i = 0; i < int(path.size() - 1); i++) {
    motionPlan.append(getNewMotion(direction, path[i], path[i+1]));
    direction = updateDirection(direction, path[i], path[i+1]);
  }
  return motionPlan;
}

// Prints the path plan to console and file
void printPathPlan(std::string pathPlan, std::vector<int> pos) {
  std::string output = prefix;
  output.append("Shortest path with least turns found!\n");
  output.append(prefix);
  output.append("Path Plan (");
  output.append(std::to_string(pathPlan.size()));
  output.append(" steps): ");
  std::string foundPath;
  // Get starting coordinate from node number
  int row = pos[0] / 9, col = pos[0] % 9;
  foundPath.append(std::to_string(row));
  foundPath.append(std::to_string(col));
  if (pos[2] == NORTH) {
    foundPath.append("N");
  } else if (pos[2] == EAST) {
    foundPath.append("E");
  } else if (pos[2] == SOUTH) {
    foundPath.append("S");
  } else if (pos[2] == WEST) {
    foundPath.append("W");
  }
  foundPath.append(pathPlan);
  output.append(foundPath);
  output.append("\n");
  output.append(prefix);
  output.append("Writing path plan to ../../PathPlan.txt...\n");
  writeToPathPlanFile(foundPath);
  output.append(prefix);
  output.append("Path plan written to ../../PathPlan.txt!\n");
  std::cout << output << std::endl;
  writeToOutputFile(output, false);
}

// Finds all shortest paths, then the shortest path with the least turns and prints
void findShortestPaths(std::vector<int> graph[], int v, std::vector<std::vector<int>> hWallArray, 
std::vector<std::vector<int>> vWallArray, std::vector<int> pos) {
  std::vector<std::vector<int>> paths;
  std::vector<int> path;
  std::vector<int> prev[v]; // stores the previous nodes from the src to each node
  int src = pos[0], dest = pos[1];
  bfs(graph, prev, v, src);
 
  recurse(paths, path, prev, src, dest);
  
  printShortestPaths(paths, hWallArray, vWallArray, pos);
  
  std::vector<int> shortestPath = findShortestPathLeastTurns(paths, pos);
  printPath(shortestPath, hWallArray, vWallArray, pos);
  std::string plan = findMotionPlan(shortestPath, pos);
  printPathPlan(plan, pos);
}

int main(int argc, char **argv) {
  std::string map = readMap();
  std::vector<std::vector<int>> hWallArray = findHorizontalWalls(map);
  std::vector<std::vector<int>> vWallArray = findVerticalWalls(map);
  std::vector<int> pos = getStartingAndTargetPos(map);
  // Create a graph
  int v = 45;
  std::vector<int> graph[v];
  createGraph(graph, hWallArray, vWallArray);
  
  // Generate all possible minimum paths
  std::string output = prefix;
  output.append("Finding shortest paths...\n");
  writeToOutputFile(output, false);
  std::cout << output;
  findShortestPaths(graph, v, hWallArray, vWallArray, pos);

  return 0;
}
