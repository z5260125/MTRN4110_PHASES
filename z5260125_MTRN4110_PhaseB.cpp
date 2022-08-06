// File:          z5260125_MTRN4110_PhaseB.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>

#include <memory>
#include <fstream>
#include <iomanip>
#include <vector>
#include <list>
#include <algorithm>
#include <sstream>

#define ROWS 5
#define COLUMNS 9
#define MAP_LENGTH 11
#define MAP_WIDTH 33

const std::string MAP_FILE_NAME = "../../Map5.txt";
const std::string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string OUTPUT_FILE_NAME = "../../Output.txt";
const std::string prefix = "[z5260125_MTRN4110_PhaseB] ";

void writeToOutput(std::string s) {
  std::ofstream pathPlan;
  pathPlan.open(OUTPUT_FILE_NAME, std::ios_base::app);
  pathPlan << s;
}

void writeToOutput(char s) {
  std::ofstream pathPlan;
  pathPlan.open(OUTPUT_FILE_NAME, std::ios_base::app);
  pathPlan << s;
}

void resetMap(std::string map[]) {
  for (int i = 0; i < MAP_LENGTH; i++) {
    for (int j = 0; j < COLUMNS; j++) {
      if (i%2 != 0) {
        if (map[i][4*j+2] != 'v' && map[i][4*j+2] != '>' && map[i][4*j+2] != '<' && map[i][4*j+2] != '^') {
          map[i][4*j+2] = ' ';
          map[i][4*j+3] = ' ';
        }
      }
    }
  }    
}

void printMap(std::string map[]) {
  for (int i = 0; i < MAP_LENGTH; i++) {
    for (int j = 0; j <= 36; j++) {
      if (j == 0) {
        std::cout << prefix;
        writeToOutput(prefix);
      }
      std::cout << map[i][j];
      writeToOutput(map[i][j]);
    }
    std::cout << std::endl;
    writeToOutput("\n");
  }
}

void writePathPlan(std::vector<std::string> pathStrings, int pathWithMinTurns) {
  std::cout << prefix << "Writing path plan to " << PATH_PLAN_FILE_NAME << "..." << std::endl;
  writeToOutput(prefix);
  writeToOutput("Writing path plan to ");
  writeToOutput(PATH_PLAN_FILE_NAME);
  writeToOutput("...\n");
  std::ofstream pathPlan(PATH_PLAN_FILE_NAME);
  for (auto c : pathStrings[pathWithMinTurns]) {
    pathPlan << c;
  }
  std::cout << prefix << "Path plan written to " << PATH_PLAN_FILE_NAME << "!" << std::endl;
  writeToOutput(prefix);
  writeToOutput("Path plan written to ");
  writeToOutput(PATH_PLAN_FILE_NAME);
  writeToOutput("!\n");
}

void find_paths(std::vector<std::vector<int>>& paths, std::vector<int> path, std::vector<int> parent[], int v, int goal) {    
  if (goal == -1) {
    paths.push_back(path);
    return;
  }

  for (int par : parent[goal]) {
    path.push_back(goal);

    find_paths(paths, path, parent, v, par);

    path.pop_back();
  }
}

void bfs(std::vector<std::vector<int>> adjMatrix, std::vector<int> parent[], int v, int start) {
  std::vector<int> dist(v, INT_MAX);
  
  std::list<int> q;

  q.push_back(start);
  parent[start] = { -1 };
  dist[start] = 0;

  while (!q.empty()) {
    int u = q.front();
    q.pop_front();
    for (int i = 0; i < v; i++) {
      if (adjMatrix[u][i] == 1) {
        if (dist[i] > dist[u] + 1) {
          dist[i] = dist[u] + 1;
          q.push_back(i);
          parent[i].clear();
          parent[i].push_back(u);
        }
        else if (dist[i] == dist[u] + 1) {
          parent[i].push_back(u);
        }
      }
    }
   }
}

void printShortestDistance(std::vector<std::vector<int>> adjMatrix, int start, int goal, int v, std::string map[], char h) {
  std::vector<std::vector<int>> paths;
  std::vector<int> path;
  std::vector<int> parent[v];
  int pathNum, x, y;
  int pCounter = 1;
  std::stringstream pathNumString;
  std::string pathString;
  std::vector<std::string> pathStrings;
  int turns = 0;
  int minTurns = 0;
  int pathWithMinTurns = 0;
  
  bfs(adjMatrix, parent, v, start);
  find_paths(paths, path, parent, v, goal);

  for (auto p : paths) {
    int prev = INT_MAX;
    pathString.clear();
    char heading = h;
    
    if (start >= 9 && start <= 17) {
      pathString.push_back('1');
    } else if (start >= 18 && start <= 26) {
      pathString.push_back('2');
    } else if (start >= 27 && start <= 35) {
      pathString.push_back('3');
    } else if (start >= 36 && start <= 44) {
      pathString.push_back('4');
    } else {
      pathString.push_back('0');
    }
    pathString.push_back('0' + start%9);
    pathString.push_back(heading);
    
    std::cout << prefix << "Path - " << pCounter << ":" << std::endl;
    writeToOutput(prefix);
    writeToOutput("Path - ");
    writeToOutput('0' + pCounter);
    writeToOutput(":\n");
    pathNum = 0;
    
    resetMap(map);

    for (int u : p) {
      if (pathNum == int(p.size()-1)) {
        continue;
      }
      
      if (u >= 9 && u <= 17) {
        x = 1;
      } else if (u >= 18 && u <= 26) {
        x = 2;
      } else if (u >= 27 && u <= 35) {
        x = 3;
      } else if (u >= 36 && u <= 44) {
        x = 4;
      } else {
        x = 0;
      }
      y = u%9;
      if (pathNum < 10) {
        pathNumString << pathNum;
        pathNumString >> map[2*x+1][4*y+2];
      } else {
        pathNumString << pathNum/10;
        pathNumString >> map[2*x+1][4*y+2];
        pathNumString << pathNum%10;
        pathNumString >> map[2*x+1][4*y+3];
      }
      pathNum++;
    }
    
    std::reverse(p.begin(), p.end());
    for (int u: p) {
      if (prev != INT_MAX) {
        int diff = u - prev;
        if (diff == 1) {
          if (heading == 'E') {
            pathString.push_back('F');
          } else if (heading == 'N') {
            pathString.push_back('R');
            pathString.push_back('F');
          } else if (heading == 'S') {
            pathString.push_back('L');
            pathString.push_back('F');
          } else if (heading == 'W') {
            pathString.push_back('L');
            pathString.push_back('L');
            pathString.push_back('F');
          }
          heading = 'E';
        } else if (diff == -1) {
          if (heading == 'W') {
            pathString.push_back('F');
          } else if (heading == 'S') {
            pathString.push_back('R');
            pathString.push_back('F');
          } else if (heading == 'N') {
            pathString.push_back('L');
            pathString.push_back('F');
          } else if (heading == 'E') {
            pathString.push_back('L');
            pathString.push_back('L');
            pathString.push_back('F');
          }
          heading = 'W';
        } else if (diff == 9) {
          if (heading == 'S') {
            pathString.push_back('F');
          } else if (heading == 'E') {
            pathString.push_back('R');
            pathString.push_back('F');
          } else if (heading == 'W') {
            pathString.push_back('L');
            pathString.push_back('F');
          } else if (heading == 'N') {
            pathString.push_back('L');
            pathString.push_back('L');
            pathString.push_back('F');
          }
          heading = 'S';
        } else {
          if (heading == 'N') {
            pathString.push_back('F');
          } else if (heading == 'W') {
            pathString.push_back('R');
            pathString.push_back('F');
          } else if (heading == 'E') {
            pathString.push_back('L');
            pathString.push_back('F');
          } else if (heading == 'S') {
            pathString.push_back('L');
            pathString.push_back('L');
            pathString.push_back('F');
          }
          heading = 'N';
        }
      }
      prev = u;
    }
    
    printMap(map);
    
    pCounter++;
    pathStrings.push_back(pathString);
  }
  std::cout << prefix << pCounter - 1 << " shortest paths found!" << std::endl;
  writeToOutput(prefix);
  writeToOutput('0' + pCounter - 1);
  writeToOutput(" shortest paths found!\n");
  std::cout << prefix << "Finding shortest path with least turns..." << std::endl;
  writeToOutput(prefix);
  writeToOutput("Finding shortest path with least turns...\n");
  for (int i = 0; i < int(pathStrings.size()); i++) {
    turns = 0;
    for (auto c : pathStrings[i]) {
      if (c == 'R' || 'L') {
        turns++;
      }
    }
    if (i == 0) {
      minTurns = turns;
      pathWithMinTurns = i;
    } else {
      if (minTurns > turns) {
        minTurns = turns;
        pathWithMinTurns = i;
      }
    }
  }
  resetMap(map);
  pathNum = 0;
  for (int u : paths[pathWithMinTurns]) {
    if (pathNum == int(paths[pathWithMinTurns].size())-1) {
      continue;
    }
    if (u >= 9 && u <= 17) {
      x = 1;
    } else if (u >= 18 && u <= 26) {
      x = 2;
    } else if (u >= 27 && u <= 35) {
      x = 3;
    } else if (u >= 36 && u <= 44) {
      x = 4;
    } else {
      x = 0;
    }
    y = u%9;
    if (pathNum < 10) {
      pathNumString << pathNum;
      pathNumString >> map[2*x+1][4*y+2];
    } else {
      pathNumString << pathNum/10;
      pathNumString >> map[2*x+1][4*y+2];
      pathNumString << pathNum%10;
      pathNumString >> map[2*x+1][4*y+3];
    }
    pathNum++;
  }
  printMap(map);
  std::cout << prefix << "Shortest path with least turns found!" << std::endl;
  writeToOutput(prefix);
  writeToOutput("Shortest path with least turns found!\n");
  std::cout << prefix << "Path Plan (" << pathStrings[pathWithMinTurns].size() - 3 << " steps): ";
  writeToOutput(prefix);
  writeToOutput("Path Plan (");
  writeToOutput('0' + (pathStrings[pathWithMinTurns].size() - 3)/10);
  writeToOutput('0' + (pathStrings[pathWithMinTurns].size() - 3)%10);
  writeToOutput(" steps): ");
  for (auto c : pathStrings[pathWithMinTurns]) {
    std::cout << c;
    writeToOutput(c);
  }
  std::cout << std::endl;
  writeToOutput("\n");
  writePathPlan(pathStrings, pathWithMinTurns);
}
 

int main(int argc, char **argv) {
  // Read path from file
  std::ifstream mapFile;
  mapFile.open(MAP_FILE_NAME);
  std::string map[ROWS * 2 + 1];
  std::string line;
  int i = 0;
  std::cout << prefix << "Reading in map from " << MAP_FILE_NAME << "..." << std::endl;
  std::ofstream pathPlan(OUTPUT_FILE_NAME);
  pathPlan << prefix << "Reading in map from " << MAP_FILE_NAME << "..." << std::endl;
  
  while (std::getline(mapFile, line)) {
    map[i] = line;
    std::cout << prefix << line << std::endl;
    writeToOutput(prefix);
    writeToOutput(line);
    writeToOutput("\n");
    i++;
  }
  std::cout << prefix << "Map read in!" << std::endl;
  writeToOutput(prefix);
  writeToOutput("Map read in!\n");
  
  // Make map into an array
  int rowWalls[ROWS+1][COLUMNS]; // 1 if there is a wall
  int colWalls[ROWS][COLUMNS+1]; // 1 if there is a wall
  int rowCell = 0;
  int columnCell = 0;
  char heading = ' ';
  int startRow = 0;
  int startColumn = 0;
  int goalRow = 0;
  int goalColumn = 0;
  
  for (int i = 0; i < MAP_LENGTH; i++) {
    columnCell = 0;
    if (i%2 == 0) {
      //Row
      for (int j = 1; j <= MAP_WIDTH+1; j+=4) {
        if (map[i][j] == '-') {
          rowWalls[rowCell][columnCell] = 1;
        } else {
          rowWalls[rowCell][columnCell] = 0;
        }
        columnCell++;
      }
      rowCell++;
    } else {
      //Column
      for (int j = 0; j <= MAP_WIDTH+1; j+=2) {
        if (j%4 == 0) {
          if (map[i][j] == '|') {
            colWalls[rowCell-1][columnCell] = 1;
          } else {
            colWalls[rowCell-1][columnCell] = 0;
          }
        } else {
          columnCell++;
          if (map[i][j] == '^') {
            heading = 'N';
            startRow = rowCell-1;
            startColumn = columnCell;
          } else if (map[i][j] == 'v') {
            heading = 'S';
            startRow = rowCell-1;
            startColumn = columnCell;
          } else if (map[i][j] == '<') {
            heading = 'W';
            startRow = rowCell-1;
            startColumn = columnCell;
          } else if (map[i][j] == '>') {
            heading = 'E';
            startRow = rowCell-1;
            startColumn = columnCell;
          } else if (map[i][j] == 'x') {
            goalRow = rowCell-1;
            goalColumn = columnCell;
          }
        }
      }
    }
  }
  
  //Default sides of rectangle
  for (int i = 0; i < COLUMNS; i++) {
    rowWalls[0][i] = 1;
    if (i < ROWS) {
      colWalls[i][COLUMNS] = 1;
    }
  }
  
  // Represent map as adjacency matrix
  std::vector<std::vector<int>> adjMatrix(ROWS*COLUMNS,std::vector<int>(ROWS*COLUMNS));

  for (int i = 0; i < (ROWS*COLUMNS); i++) {
    for (int j = 0; j < (ROWS*COLUMNS); j++) {
      adjMatrix[i][j] = 0;
    }
  }

  int pos = 0;
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLUMNS; j++) {   
      if (i == ROWS - 1) {
        if (colWalls[i][j+1] == 0) {
          adjMatrix[pos][pos+1] = 1;
          adjMatrix[pos+1][pos] = 1;
        }
        pos++;
        continue;
      }
      
      if (j == COLUMNS - 1) {
        if (rowWalls[i+1][j] == 0) {
          adjMatrix[pos+COLUMNS][pos] = 1;
          adjMatrix[pos][pos+COLUMNS] = 1;        
        }
      } else {
        if (colWalls[i][j+1] == 0) {
          adjMatrix[pos][pos+1] = 1;
          adjMatrix[pos+1][pos] = 1;
        }
        if (rowWalls[i+1][j] == 0) {
          adjMatrix[pos+COLUMNS][pos] = 1;
          adjMatrix[pos][pos+COLUMNS] = 1;        
        }
      }
      pos++;
    }
  }
  
  int start = COLUMNS*startRow + startColumn;
  int goal = (COLUMNS*goalRow + goalColumn)-1;
  
  std::cout << prefix << "Finding shortest paths..." << std::endl;
  writeToOutput(prefix);
  writeToOutput("Finding shortest paths...\n");
  printShortestDistance(adjMatrix, start-1, goal, ROWS*COLUMNS, map, heading);

  return 0;
}

