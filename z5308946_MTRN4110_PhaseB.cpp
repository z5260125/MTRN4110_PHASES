// File:          z5308946_MTRN4110_PhaseB.cpp
// Date:          10/07/2022
// Description:   Used Windows 10
// Author:        Iniyan Vigneswaran
// Modifications: 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
//#include <webots/Robot.hpp>

#include <iostream>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <fstream>
#include <queue>
 

const std::string  MAP_FILE_NAME = "../../Map.txt";
const std::string  PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string  OUTPUT_FILE_NAME = "../../Output.txt";

constexpr int numMapRows = 11;

// Once again this class is just repurposed from my MTRN2500 code
class CsvProcessor {
public:
    CsvProcessor(std::string robotName, std::string fileName, char delim = ',')
        : mRobotName{ robotName }, mFileName{ fileName }, mDelim{ delim } {};
    void writeLineToCsv(const std::vector<std::string>& dataLine) const;
    std::vector<std::vector<std::string>> readDataFromCsv() const;
private:
    std::string mRobotName;
    std::string mFileName;
    char mDelim;
};

void CsvProcessor::writeLineToCsv(const std::vector<std::string>& dataLine) const {
    std::ofstream fout{ mFileName, std::ios::out | std::ios::app };
    if (fout.is_open()) {
        for (auto iter = dataLine.begin(); iter != dataLine.end() - 1; ++iter) {
            fout << std::fixed << std::setprecision(3) << *iter << mDelim;
        }
        fout << std::fixed << std::setprecision(3) << dataLine.back() << std::endl;
    }
    else {
        throw std::runtime_error(mRobotName + ": Writing new line to " + mFileName + " failed!");
    }
}

std::vector<std::vector<std::string>> CsvProcessor::readDataFromCsv() const {

    std::vector<std::vector<std::string>> data;

    std::ifstream fin{ mFileName, std::ios::in };
    if (fin.is_open()) {
        std::string readLine;
        while (std::getline(fin, readLine)) {
            std::stringstream stringStream{ readLine };
            std::vector<std::string> result;

            while (stringStream.good()) {
                std::string subReadLine;
                std::getline(stringStream, subReadLine, ',');
                //double doubReadLine = std::stod(subReadLine);
                result.push_back(subReadLine);
                //result.push_back(doubReadLine);
            }

            data.push_back(result);

        }
        
    }
    else {

        throw std::runtime_error(mRobotName + ": Reading data from " + mFileName + " failed!");

    }

    return data;
}

void printAndPush(std::string pushMe, std::string robotName) {
  
  try {
      std::vector<std::string> pushThis;
      pushThis.push_back(pushMe);
      CsvProcessor csvWriter{ robotName, OUTPUT_FILE_NAME };
      csvWriter.writeLineToCsv(pushThis);  
  }
  catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
  }
  
  std::cout << pushMe << std::endl;
}

bool checkQueue(int value, std::queue<int> q) {
  int i = 0;
  bool ret = false;
  while (!q.empty()) {
    if (q.front() == value) {
      ret = true;
    }
    i++;
    q.pop();
  }

  return ret;
  
}

int arrayLength(int arr[4]) {
  int i = 0;
  while (i < 4) {
    if (arr[i] == -1) {
      break;
    }
    i++;
  }
  return i;
}

bool alreadyInArray(int arr[4], int val) {
  int i = 0;
  bool ret = false;
  while (i < 4) {
    if (arr[i] == val) {
      ret = true;
    }
    i++;
  }
  return ret;
}

std::vector<int> generatePath(std::vector<std::vector<int>>& paths, std::vector<int> thisPath, int currentPos, int finalPos, int floodFillParents[5][9][4]) {

  while (true) {
    if (arrayLength(floodFillParents[currentPos/10][currentPos%10]) > 1) {
      int i = 1;
      while (i < arrayLength(floodFillParents[currentPos/10][currentPos%10])) {
        std::vector<int> startPath = thisPath;
        int newCurrentPos = floodFillParents[currentPos/10][currentPos%10][i];
        startPath.push_back(newCurrentPos);
        paths.push_back(generatePath(paths, startPath, newCurrentPos, finalPos, floodFillParents));
        i++;
      }
    }
    int newTing = floodFillParents[currentPos/10][currentPos%10][0];
    thisPath.push_back(newTing);
    currentPos = newTing;
    if (currentPos == finalPos) {    
      return thisPath;
    }
  }
}

void printMap(int mapToPrint[5][9], int horizontalWalls[6][9], int verticalWalls[5][10], int initialRow, int initialCol, std::string heading, std::string robotName) {
  int alternator = 0;
  int numRows = 0;
  while (numRows < 11) {
    std::string printMe = "[z5308946_MTRN4110_PhaseB] ";
    if (alternator == 0) {
      int g = 0;
      while (g < 9) {
        if (horizontalWalls[numRows/2][g] == 1) {
          printMe = printMe + " ---";
        } else {
          printMe = printMe + "    ";
        }
        g++;
      }
      printMe = printMe + " ";
    } else {
      int h = 0;
      int extraAlternator = 0;
      while (h < 19) {

        if (extraAlternator == 0) {
          if (verticalWalls[(numRows-1)/2][h/2] == 1) {
            printMe = printMe + "|";
          } else {
            printMe = printMe + " ";
          }
        } else {
          if ((numRows-1)/2 == initialRow && h/2 == initialCol) {
            
            if (heading == "N") {
              printMe = printMe + " ^ ";
            } else if (heading == "E") {
              printMe = printMe + " > ";
            } else if (heading == "S") {
              printMe = printMe + " v ";
            } else if (heading == "W") {
              printMe = printMe + " < ";
            }
            
          } else if (mapToPrint[(numRows-1)/2][h/2] < 10) {
            printMe = printMe + " " + std::to_string(mapToPrint[(numRows-1)/2][h/2]) + " ";
          } else if (mapToPrint[(numRows-1)/2][h/2] < 100) {
            printMe = printMe + " " + std::to_string(mapToPrint[(numRows-1)/2][h/2]);
          } else {
            printMe = printMe + "   ";
          }
          
        }
        
        h++;
        extraAlternator = 1 - extraAlternator;
      }
      std::cout << std::endl;
    }

  
    numRows++;
    alternator = 1 - alternator;
    printAndPush(printMe, robotName);
  }
}


void printMaps(int mapPossibilities[4096][5][9], int horizontalWalls[6][9], int verticalWalls[5][10], int numMaps, int initialRow, int initialCol, std::string heading, std::string robotName) {
  int iterator = 0;
  while (iterator < numMaps) {
    printAndPush("[z5308946_MTRN4110_PhaseB] Path - " + std::to_string(iterator + 1) + ":", robotName);
    
    printMap(mapPossibilities[iterator], horizontalWalls, verticalWalls, initialRow, initialCol, heading, robotName);

    iterator++;
  }
  
  printAndPush("[z5308946_MTRN4110_PhaseB] " + std::to_string(iterator) + " shortest paths found!", robotName);
}

int headingToBearing(std::string heading) {
  int bearing = 0;

  if (heading == "N") {
    bearing = 0;
  } else if (heading == "E") {
    bearing = 90;
  } else if (heading == "S") {
    bearing = 180;
  } else if (heading == "W") {
    bearing = 270;
  }
  
  return bearing;
}

int main(int argc, char **argv) {

  //webots::Robot robot;
  // int timeStep = robot.getBasicTimeStep();
  int timeStep = 64;

  //std::string robotName{ robot.getName() };
  std::string robotName{ "e-puck" };
  std::vector<std::vector<std::string>> data;

  try {
    //CsvProcessor csvProcessor{ robotName, fileName };
    CsvProcessor csvReader{ robotName, MAP_FILE_NAME };
    data = csvReader.readDataFromCsv();
  }
  catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
  }
  
  printAndPush("[z5308946_MTRN4110_PhaseB] Reading in map from " + MAP_FILE_NAME + "...", robotName);
  
  int horizontalWalls[6][9] = {0};
  int verticalWalls[5][10] = {0};
  
  
  int i = 0;
  
  int initialRow = -1;
  int initialCol = -1;
  std::string initialHeading;
  
  int finalRow = -1;
  int finalCol = -1;
  
  
  while (i < numMapRows) {
    std::string mapRow = data[i][0];
    printAndPush("[z5308946_MTRN4110_PhaseB] " + mapRow, robotName);
    
    if (i % 2 == 0) {
      int j = 3;
      while (j < 37) {
        if (mapRow[j] == '-') {
          horizontalWalls[i/2][(j+1)/4-1] = 1;
        }
        j = j + 4;
      }
    } else {
      int j = 0;
      while (j < 37) {
        if (mapRow[j] == '|') {
          verticalWalls[i/2][j/4] = 1;
          j = j + 2;
          if (mapRow[j] == 'v') {
            initialHeading = "S";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == '>') {
            initialHeading = "E";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == '^') {
            initialHeading = "N";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == '<') {
            initialHeading = "W";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == 'x') {
            finalRow = i/2;
            finalCol = (j-2)/4;
          }
        } else {
          j = j + 2;
          if (mapRow[j] == 'v') {
            initialHeading = "S";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == '>') {
            initialHeading = "E";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == '^') {
            initialHeading = "N";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == '<') {
            initialHeading = "W";
            initialRow = i/2;
            initialCol = (j-2)/4;
          } else if (mapRow[j] == 'x') {
            finalRow = i/2;
            finalCol = (j-2)/4;
          }
        }
        j = j + 2;
      }
    
    }
    
    
    i++;
  }
  

  printAndPush("[z5308946_MTRN4110_PhaseB] Map read in!", robotName);
    
  /*
  while (robot.step(timeStep) != -1) {

  };
  */

  int floodFillMap[5][9];

  int e = 0;
  while (e < 5) {
    int f = 0;
    while (f < 9) {
      floodFillMap[e][f] = 45;
      f++;
    }
    e++;
  }

    
  int floodFillParents[5][9][4];
  int o = 0;
  while (o < 5) {
    int p = 0;
    while (p < 9) {
      int q = 0;
      while (q < 4) {
        floodFillParents[o][p][q] = -1;
        q++;
      }
      p++;
    }
    o++;
  }

  printAndPush("[z5308946_MTRN4110_PhaseB] Finding shortest paths...", robotName);
  
  std::queue<int> q;
  std::queue<int> visited;
  
  floodFillMap[finalRow][finalCol] = 0;
  visited.push(finalRow*10+finalCol);
  q.push(finalRow*10+finalCol);
  
  while (!(q.empty())) {
    std::queue<int> q1 = visited;
    std::queue<int> q2 = visited;
    std::queue<int> q3 = visited;
    std::queue<int> q4 = visited;
    
    int currentPos = q.front();
    q.pop();
    int currentRow = currentPos / 10;
    int currentCol = currentPos % 10;

    if (horizontalWalls[currentRow][currentCol] == 0 && !checkQueue((currentRow-1)*10+(currentCol),q1)) {
      floodFillMap[currentRow-1][currentCol] = floodFillMap[currentRow][currentCol] + 1;
      visited.push((currentRow-1)*10+(currentCol));
      q.push((currentRow-1)*10+(currentCol));
      floodFillParents[currentRow-1][currentCol][arrayLength(floodFillParents[currentRow-1][currentCol])] = currentRow*10+currentCol;
    }
    if (horizontalWalls[currentRow][currentCol] == 0 && !alreadyInArray(floodFillParents[currentRow-1][currentCol], currentRow*10+currentCol) && floodFillMap[currentRow][currentCol] == floodFillMap[currentRow-1][currentCol] - 1) {
      floodFillParents[currentRow-1][currentCol][arrayLength(floodFillParents[currentRow-1][currentCol])] = currentRow*10+currentCol;
    }
    if (horizontalWalls[currentRow+1][currentCol] == 0 && !checkQueue((currentRow+1)*10+(currentCol),q2)) {
      floodFillMap[currentRow+1][currentCol] = floodFillMap[currentRow][currentCol] + 1;
      visited.push((currentRow+1)*10+(currentCol));
      q.push((currentRow+1)*10+(currentCol));
      floodFillParents[currentRow+1][currentCol][arrayLength(floodFillParents[currentRow+1][currentCol])] = currentRow*10+currentCol;
    }
    if (horizontalWalls[currentRow+1][currentCol] == 0 && !alreadyInArray(floodFillParents[currentRow+1][currentCol], currentRow*10+currentCol) && floodFillMap[currentRow][currentCol] == floodFillMap[currentRow+1][currentCol] - 1) {
      floodFillParents[currentRow+1][currentCol][arrayLength(floodFillParents[currentRow+1][currentCol])] = currentRow*10+currentCol;
    }
    if (verticalWalls[currentRow][currentCol] == 0 && !checkQueue((currentRow)*10+(currentCol-1),q3)) {
      floodFillMap[currentRow][currentCol-1] = floodFillMap[currentRow][currentCol] + 1;
      visited.push((currentRow)*10+(currentCol-1));
      q.push((currentRow)*10+(currentCol-1));
      floodFillParents[currentRow][currentCol-1][arrayLength(floodFillParents[currentRow][currentCol-1])] = currentRow*10+currentCol;
    }
    if (verticalWalls[currentRow][currentCol] == 0 && !alreadyInArray(floodFillParents[currentRow][currentCol-1], currentRow*10+currentCol) && floodFillMap[currentRow][currentCol] == floodFillMap[currentRow][currentCol-1] - 1) {
      floodFillParents[currentRow][currentCol-1][arrayLength(floodFillParents[currentRow][currentCol-1])] = currentRow*10+currentCol;
    }
    if (verticalWalls[currentRow][currentCol+1] == 0 && !checkQueue((currentRow)*10+(currentCol+1),q4)) {
      floodFillMap[currentRow][currentCol+1] = floodFillMap[currentRow][currentCol] + 1;
      visited.push((currentRow)*10+(currentCol+1));
      q.push((currentRow)*10+(currentCol+1));
      floodFillParents[currentRow][currentCol+1][arrayLength(floodFillParents[currentRow][currentCol+1])] = currentRow*10+currentCol;
    }
    if (verticalWalls[currentRow][currentCol+1] == 0 && !alreadyInArray(floodFillParents[currentRow][currentCol+1], currentRow*10+currentCol) && floodFillMap[currentRow][currentCol] == floodFillMap[currentRow][currentCol+1] - 1) {
      floodFillParents[currentRow][currentCol+1][arrayLength(floodFillParents[currentRow][currentCol+1])] = currentRow*10+currentCol;
    }
    
  }

  std::vector<std::vector<int>> paths;
  int temp = (initialRow)*10+(initialCol);
  std::vector<int> thisPath;
  thisPath.push_back(temp);
  
  paths.push_back(generatePath(paths, thisPath, (initialRow)*10+(initialCol), finalRow*10+finalCol, floodFillParents));
  
  int mapPossibilities[4096][5][9];
  int mI = 0;
  while (mI < paths.size()) {
    int mJ = 0;
    while (mJ < 5) {
      int mK = 0;
      while (mK < 9) {
        mapPossibilities[mI][mJ][mK] = 1000;
        mK++;
      }
      mJ++;
    }
    mI++;
  }

  int aI = 0;
  while (aI < paths.size()) {
    int aJ = 0;
    while (aJ < paths[aI].size()) {
      int currentPos = paths[aI][aJ];
      mapPossibilities[aI][currentPos/10][currentPos%10] = floodFillMap[currentPos/10][currentPos%10];
      aJ++;
    }
    aI++; 
  }
  
  printMaps(mapPossibilities, horizontalWalls, verticalWalls, paths.size(), (initialRow), (initialCol), initialHeading, robotName);

  printAndPush("[z5308946_MTRN4110_PhaseB] Finding shortest path with least turns...", robotName);
  
  std::vector<std::vector<std::string>> pathHeadings;
  
  int hI = 0;
  while (hI < paths.size()) {
    int hJ = 0;
    std::vector<std::string> thisPathsHeadings;
    thisPathsHeadings.push_back(initialHeading);
    while (hJ < paths[hI].size()-1) {
      if (paths[hI][hJ+1]-paths[hI][hJ] == -10) {
        thisPathsHeadings.push_back("N");
      } else if (paths[hI][hJ+1]-paths[hI][hJ] == 1) {
        thisPathsHeadings.push_back("E");
      } else if (paths[hI][hJ+1]-paths[hI][hJ] == 10) {
        thisPathsHeadings.push_back("S");
      } else if (paths[hI][hJ+1]-paths[hI][hJ] == -1) {
        thisPathsHeadings.push_back("W");
      }
      hJ++;
    }
    pathHeadings.push_back(thisPathsHeadings);
    hI++;
  }

  int shortestPath = 0;
  int lowestChanges = 50;
  int pI = 0;
  while (pI < pathHeadings.size()) {
    int pJ = 0;
    int changes = 0;
    while (pJ < pathHeadings[pI].size()-1) {
      if (!(pathHeadings[pI][pJ] == pathHeadings[pI][pJ+1])) {
        changes++;
        if ((pathHeadings[pI][pJ] == "N" && pathHeadings[pI][pJ+1] == "S") || (pathHeadings[pI][pJ] == "S" && pathHeadings[pI][pJ+1] == "N") || (pathHeadings[pI][pJ] == "E" && pathHeadings[pI][pJ+1] == "W") || (pathHeadings[pI][pJ] == "W" && pathHeadings[pI][pJ+1] == "E")) {
          changes++;
        }
        
      }
      pJ++;
    }
    if (changes < lowestChanges) {
      lowestChanges = changes;
      shortestPath = pI;
    }
    
    pI++;
  }

  printMap(mapPossibilities[shortestPath], horizontalWalls, verticalWalls, (initialRow), (initialCol), initialHeading, robotName);

  printAndPush("[z5308946_MTRN4110_PhaseB] Shortest path with least turns found!", robotName);
  
  std::string pathInstructions = std::to_string(initialRow) + std::to_string(initialCol) + initialHeading;

  int numInstructions = 0;
  int iI = 0;
  while (iI < pathHeadings[shortestPath].size()-1) {

     if ((headingToBearing(pathHeadings[shortestPath][iI+1]) - headingToBearing(pathHeadings[shortestPath][iI]) + 360)%360 == 0) {
      pathInstructions = pathInstructions + "F";
       numInstructions = numInstructions + 1;
    } else if ((headingToBearing(pathHeadings[shortestPath][iI+1]) - headingToBearing(pathHeadings[shortestPath][iI]) + 360)%360 == 270) {
      pathInstructions = pathInstructions + "LF";
       numInstructions = numInstructions + 2;
    } else if ((headingToBearing(pathHeadings[shortestPath][iI+1]) - headingToBearing(pathHeadings[shortestPath][iI]) + 360)%360 == 90) {
      pathInstructions = pathInstructions + "RF";
       numInstructions = numInstructions + 2;
    } else if ((headingToBearing(pathHeadings[shortestPath][iI+1]) - headingToBearing(pathHeadings[shortestPath][iI]) + 360)%360 == 180) {
      pathInstructions = pathInstructions + "LLF";
      numInstructions = numInstructions + 3;
    }
    
    iI++;
  }

  printAndPush("[z5308946_MTRN4110_PhaseB] Path Plan (" + std::to_string(numInstructions) + " steps): " + pathInstructions, robotName);

  try {
      printAndPush("[z5308946_MTRN4110_PhaseB] Writing path plan to " + PATH_PLAN_FILE_NAME + "...", robotName);
      CsvProcessor csvWriter{ robotName, PATH_PLAN_FILE_NAME };
      std::vector<std::string> dataLine;
      dataLine.push_back(pathInstructions);
      csvWriter.writeLineToCsv(dataLine);
      printAndPush("[z5308946_MTRN4110_PhaseB] Path plan written to " + PATH_PLAN_FILE_NAME + "!", robotName); 
  }
  catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
  }
  
  
  
  return 0;
}
