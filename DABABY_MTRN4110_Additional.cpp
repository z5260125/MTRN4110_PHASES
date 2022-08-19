/*
 * File:          DABABY_MTRN4110_PhaseD.cpp
 * Date:          20/06/2022
 * Description:   Controller of E-puck for Phase D - Additional Features
 * Author:        DA DABABY Team
 * Modifications:
 * Platform:      Windows 10
 * Notes: 
*/

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>

#include <iostream>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>
#include <queue>

#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270
#define ROWS 5
#define COLS 9

constexpr double maxWheelSpeed = 6.28;
constexpr double wheelRadius = 0.0205;
constexpr double axleLength = 0.062825;
constexpr double pi = 3.14;

constexpr double oneSquarePos = 0.165;
constexpr double turnPos = M_PI/2*axleLength/2;

const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";

// void phaseC() {
  // if (PyImport_AppendInittab("MTRN4110_PhaseC", PyInit_MTRN4110_PhaseC) == -1) {

        // throw std::runtime_error("Could not extend built-in modules table.");

  // }
  // Py_Initialize();
  // PyRun_SimpleString("print('Hello World from Embedded Python!!!')");
  // FILE* fp = _Py_wfopen(L"MTRN4110_PhaseC.py", L"r");
  // if (fp != NULL) {
    // PyRun_SimpleFile(fp, "MTRN4110_PhaseC.py");
  // }
  // auto module = PyImport_ImportModule("example");

  // if (module == nullptr) {

      // PyErr_Print();

      // throw std::runtime_error("Could not import example.");

  // }
  
  // main();
  
  // Py_Finalize();
// }


std::string wallDetected(double sensorValue) {
  
  std::string returnValue = "N";
  
  if (sensorValue < 700.0) {
    returnValue = "Y";
  }
  
  return returnValue;
  
}

std::vector<int> wallsDetected(double dist[3]) {
  std::vector<int> walls(3);
  for (int i = 0; i < 3; i++) {
    if (wallDetected(dist[i]) == "Y") {
      walls[i] = 1;
    }
    else {
      walls[i] = 0;
    }
  }

  return walls;
}

std::string bearToHead(int bearing) {
  
  std::string heading;
  
  if (bearing == 0) {
    heading = "N";
  } else if (bearing == 90) {
    heading = "E";
  } else if (bearing == 180) {
    heading = "S";
  } else if (bearing == 270) {
    heading = "W";
  }
  
  return heading;
}

void updateWalls(std::vector<std::vector<int>> &hWallArr, std::vector<std::vector<int>> &vWallArr, int currentRow, int currentCol, std::vector<int> walls, int bearing) {
  if (bearing == NORTH) {
    if (currentRow != 0) {
      hWallArr[currentRow-1][currentCol] = walls[1]; //front wall
    }
    if (currentCol != 0) {
      vWallArr[currentRow][currentCol-1] = walls[0]; //left wall
    }
    if (currentCol != 8) {
      vWallArr[currentRow][currentCol] = walls[2]; //right wall
    }
  } else if (bearing == EAST) {
    if (currentCol != 8) {
      vWallArr[currentRow][currentCol] = walls[1]; //front wall
    }
    if (currentRow != 0) {
      hWallArr[currentRow-1][currentCol] = walls[0]; //left wall
    }
    if (currentRow != 4) {
      hWallArr[currentRow][currentCol] = walls[2]; //right wall
    }
  } else if (bearing == SOUTH) {
    if (currentRow != 4) {
      hWallArr[currentRow][currentCol] = walls[1]; //front wall
    }
    if (currentCol != 8) {
      vWallArr[currentRow][currentCol] = walls[0]; //left wall
    }
    if (currentCol != 0) {
      vWallArr[currentRow][currentCol-1] = walls[2]; //right wall
    }
  } else {
    if (currentCol != 0) {
      vWallArr[currentRow][currentCol-1] = walls[1]; //front wall
    }
    if (currentRow != 4) {
      hWallArr[currentRow][currentCol] = walls[0]; //left wall
    }
    if (currentRow != 0) {
      hWallArr[currentRow-1][currentCol] = walls[2]; //right wall
    }
  }
}

int checkUnvisitedNeighbours(std::vector<std::vector<int>> map, int row, int col) {
  int unvisited = 1;
  if (row != 0 && !map[row-1][col]) {
    unvisited++;
  }
  if (row != 4 && !map[row+1][col]) {
    unvisited++;
  }
  if (col != 0 && !map[row][col-1]) {
    unvisited++;
  }
  if (col != 8 && !map[row][col+1]) {
    unvisited++;
  }
  return unvisited;
}

int checkLowestReachableNeighbour(std::vector<std::vector<int>> map, int row, int col, int bearing, std::vector<int> walls) {
  int min = 1000;
  int neighbour = 0;
  if (bearing == NORTH) {
    if (row != 0 && map[row-1][col] < min && !walls[1]) {
      min = map[row-1][col];
      neighbour = 1;
      //std::cout << "up" << std::endl;
    }
    if (col != 0 && map[row][col-1] < min && !walls[0]) {
      min = map[row][col-1];
      neighbour = 4;
      //std::cout << "left" << std::endl;
    }
    if (col != 8 && map[row][col+1] < min && !walls[2]) {
      neighbour = 2;
      //std::cout << "right" << std::endl;
    }
  } else if (bearing == EAST) {
    if (row != 0 && map[row-1][col] < min && !walls[0]) {
      min = map[row-1][col];
      neighbour = 1;
      //std::cout << "up" << std::endl;
    }
    if (row != 4 && map[row+1][col] < min && !walls[2]) {
      min = map[row+1][col];
      neighbour = 3;
      //std::cout << "down" << std::endl;
    }
    if (col != 8 && map[row][col+1] < min && !walls[1]) {
      neighbour = 2;
      //std::cout << "right" << std::endl;
    }
  } else if (bearing == SOUTH) {
    if (row != 4 && map[row+1][col] < min && !walls[1]) {
      min = map[row+1][col];
      neighbour = 3;
      //std::cout << "down" << std::endl;
    }
    if (col != 0 && map[row][col-1] < min && !walls[2]) {
      min = map[row][col-1];
      neighbour = 4;
      //std::cout << "left" << std::endl;
    }
    if (col != 8 && map[row][col+1] < min && !walls[0]) {
      neighbour = 2;
      //std::cout << "right" << std::endl;
    }
  } else {
    if (row != 0 && map[row-1][col] < min && !walls[2]) {
      min = map[row-1][col];
      neighbour = 1;
      //std::cout << "up" << std::endl;
    }
    if (row != 4 && map[row+1][col] < min && !walls[0]) {
      min = map[row+1][col];
      neighbour = 3;
      //std::cout << "down" << std::endl;
    }
    if (col != 0 && map[row][col-1] < min && !walls[1]) {
      neighbour = 4;
      //std::cout << "left" << std::endl;
    }
  }
  return neighbour;
}

void printExplorationModuleMap(std::vector<std::vector<int>> vWallArray, 
std::vector<std::vector<int>> hWallArray, int targetRow, int targetCol) {
  std::string prefix = "[DABABY_MTRN4110_PhaseD] ";
  std::string hBorder = " --- --- --- --- --- --- --- --- --- ";
  std::cout << prefix << hBorder << std::endl;
  for (int i = 0; i < ROWS; i++) {
    std::cout << prefix << "|";
    for (int j = 0; j < COLS-1; j++) {
      if (i == targetRow && j == targetCol) {
        std::cout << " x ";
      } else {
        std::cout << "   ";
      }
      if (vWallArray[i][j]) {
        std::cout << "|";
      } else {
        std::cout << " ";
      }
    }
    std::cout << "   " << "|" << std::endl;
    // print horizontal walls
    if (i == 4) {
      break;
    }
    std::cout << prefix;
    for (int k = 0; k < COLS; k++) {
      if (hWallArray[i][k]) {
        std::cout << " ---";
      } else {
        std::cout << "    ";
      }
    }
    std::cout << " " << std::endl;
  }
  std::cout << prefix << hBorder << std::endl;
}

// Class repurposed from my own MTRN2500 CSV Reader code
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
                result.push_back(subReadLine);
            }

            data.push_back(result);

        }

        std::cout << "[DABABY_MTRN4110_PhaseD] Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
        std::cout << "[DABABY_MTRN4110_PhaseD] Motion Plan: " << data[0][0] << std::endl;
        std::cout << "[DABABY_MTRN4110_PhaseD] Motion plan read in!" << std::endl;
    }
    else {

        throw std::runtime_error(mRobotName + ": Reading data from " + mFileName + " failed!");

    }

    return data;
}

void checkWalls (int& timeStep, double& leftDist, double& forwardDist, double& rightDist, webots::Robot& robot, std::shared_ptr<webots::DistanceSensor> leftDS, std::shared_ptr<webots::DistanceSensor> frontDS, std::shared_ptr<webots::DistanceSensor> rightDS) {
  double dsLSum = 0;
  double dsFSum = 0;
  double dsRSum = 0;
  double dsIter = 0;
  
  double tempTime = robot.getTime();
  while (robot.step(timeStep) != -1) {
    dsIter = dsIter + 1;
    dsLSum = dsLSum + leftDS->getValue();
    dsFSum = dsFSum + frontDS->getValue();
    dsRSum = dsRSum + rightDS->getValue();
    
    if (robot.getTime() - tempTime >= 10) {
      break;
    }
  }

  leftDist = dsLSum/dsIter;
  forwardDist = dsFSum/dsIter;
  rightDist = dsRSum/dsIter;

}

void goForward(int i, int& row, int& col, int bearing, int timeStep, webots::Robot& robot, std::shared_ptr<webots::Motor> leftMotor, std::shared_ptr<webots::Motor> rightMotor, std::shared_ptr<webots::PositionSensor> leftWheelPS, std::shared_ptr<webots::PositionSensor> rightWheelPS) {
  
  double timeStart = robot.getTime();
  while (robot.step(timeStep) != -1) {
    double t = robot.getTime() - timeStart;
    double v = 1.9785*t-0.3957*t*t;
    if (v <= 0) {
      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0);
      break;
    }
    //std::cout << t << ", " << v << std::endl;
    leftMotor->setVelocity(v);
    rightMotor->setVelocity(v);
  }
  
  if (bearing == 0) {
    row--;
  } else if (bearing == 180) {
    row++;
  } else if (bearing == 90) {
    col++;
  } else if (bearing == 270) {
    col--;
  }
  
}

void turnNinety(int i, int direction, int& bearing, int& timeStep, webots::Robot& robot, std::shared_ptr<webots::Motor> leftMotor, std::shared_ptr<webots::Motor> rightMotor, std::shared_ptr<webots::InertialUnit> imu) {
  
  double timeStart = robot.getTime();
  while (robot.step(timeStep) != -1) {
    double t = robot.getTime() - timeStart;
    double v = 0.5374*t-0.1075*t*t;
    if (v <= 0) {
      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0);
      break;
    }
    //std::cout << t << ", " << v << std::endl;
    leftMotor->setVelocity(-1*direction*v);
    rightMotor->setVelocity(direction*v);

  }
  
  bearing = (bearing + (180 + direction*90)) % 360;
  
}

void printAndWrite(int row, int col, int step, int bearing, double leftDist, double forwardDist, double rightDist, std::string robotName) {
  
  std::string heading = bearToHead(bearing);
  std::string leftWall = wallDetected(leftDist);
  std::string frontWall = wallDetected(forwardDist);
  std::string rightWall = wallDetected(rightDist);
  std::cout << std::endl;
  std::cout << "[DABABY_MTRN4110_PhaseD] Step: " << std::setw(3) << std::setfill('0') << step << ", Row: " << row << ", Column: " << col << ", Heading: " << heading << ", Left Wall: " << leftWall << ", Front Wall: " << frontWall << ", Right Wall: " << rightWall << std::endl;
  std::vector<std::string> dataLine;
  dataLine.push_back(std::to_string(step));
  dataLine.push_back(std::to_string(row));
  dataLine.push_back(std::to_string(col));
  dataLine.push_back(heading);
  dataLine.push_back(leftWall);
  dataLine.push_back(frontWall);
  dataLine.push_back(rightWall);
  
  try {
    CsvProcessor csvWriter{ robotName, MOTION_EXECUTION_FILE_NAME };
    csvWriter.writeLineToCsv(dataLine);
  }
  catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
  }
  
}

void phaseA(int& timeStep, webots::Robot& robot, std::shared_ptr<webots::Motor> leftMotor, std::shared_ptr<webots::Motor> rightMotor, std::shared_ptr<webots::DistanceSensor> leftDS, std::shared_ptr<webots::DistanceSensor> frontDS, std::shared_ptr<webots::DistanceSensor> rightDS, std::shared_ptr<webots::PositionSensor> leftWheelPS, std::shared_ptr<webots::PositionSensor> rightWheelPS, std::shared_ptr<webots::InertialUnit> imu) {
  std::string robotName{ robot.getName() };
  
  std::vector<std::vector<std::string>> data;  
  
  try {
    CsvProcessor csvReader{ robotName, MOTION_PLAN_FILE_NAME };
    data = csvReader.readDataFromCsv();
 
  }
  catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
  }
  
  std::vector<std::string> headLine;
  headLine.push_back("Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall");
  
  try {
      CsvProcessor csvWriter{ robotName, MOTION_EXECUTION_FILE_NAME };
      csvWriter.writeLineToCsv(headLine);
  }
  catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
  }
  
  std::string dataString = data[0][0];
  
  int dataSize = dataString.length();
  
  int step = 0;
  
  int row = dataString[0] - '0';
  int col = dataString[1] - '0';
  
  int bearing = 0;
  if (dataString[2] == 'N') {
    bearing = 0;
  } else if (dataString[2] == 'E') {
    bearing = 90;
  } else if (dataString[2] == 'S') {
    bearing = 180;
  } else if (dataString[2] == 'W') {
    bearing = 270;
  }
  
  double leftDist = 0.0;
  double forwardDist = 0.0;
  double rightDist = 0.0;
  
  checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
  printAndWrite(row, col, step, bearing, leftDist, forwardDist, rightDist, robotName);
  
  
  for (int i = 3; i < dataSize; i++) {
    
    if (dataString[i] == 'F') {
      
      goForward(i, row, col, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
      checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
      step++;
      printAndWrite(row, col, step, bearing, leftDist, forwardDist, rightDist, robotName);
      
    } else if (dataString[i] == 'L') {
    
      turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
      checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
      step++;
      printAndWrite(row, col, step, bearing, leftDist, forwardDist, rightDist, robotName);
      
    } else if (dataString[i] == 'R') {
      
      turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
      checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
      step++;
      printAndWrite(row, col, step, bearing, leftDist, forwardDist, rightDist, robotName);

    }
    
  }
  
  std::cout << "[DABABY_MTRN4110_PhaseD] Motion plan executed!" << std::endl;
}

void communicateWalls(int horizontalWalls[6][9], int verticalWalls[5][10], int row, int col, int bearing, double& leftDist, double& forwardDist, double& rightDist) {
  if (bearing == 0) {
    if (wallDetected(leftDist) == "Y") {
      verticalWalls[row][col] = 1;  
    }
    if (wallDetected(forwardDist) == "Y") {
      horizontalWalls[row][col] = 1;  
    }
    if (wallDetected(rightDist) == "Y") {
      verticalWalls[row][col+1] = 1;  
    }
  } else if (bearing == 180) {
    if (wallDetected(leftDist) == "Y") {
      verticalWalls[row][col+1] = 1;  
    }
    if (wallDetected(forwardDist) == "Y") {
      horizontalWalls[row+1][col] = 1;
    }
    if (wallDetected(rightDist) == "Y") {
      verticalWalls[row][col] = 1;  
    }
  } else if (bearing == 90) {
    if (wallDetected(leftDist) == "Y") {
      horizontalWalls[row][col] = 1;  
    }
    if (wallDetected(forwardDist) == "Y") {
      verticalWalls[row][col+1] = 1;
    }
    if (wallDetected(rightDist) == "Y") {
      horizontalWalls[row+1][col] = 1;  
    }
  } else if (bearing == 270) {
    if (wallDetected(leftDist) == "Y") {
      horizontalWalls[row+1][col] = 1;  
    }
    if (wallDetected(forwardDist) == "Y") {
      verticalWalls[row][col] = 1;
    }
    if (wallDetected(rightDist) == "Y") {
      horizontalWalls[row][col] = 1;  
    }
  } 
}

void printMap(int horizontalWalls[6][9], int verticalWalls[5][10], int initialRow, int initialCol, std::string heading, int finalRow, int finalCol) {
  int alternator = 0;
  int numRows = 0;
  while (numRows < 11) {
    std::string printMe = "[DABABY_MTRN4110_PhaseD] ";
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
          /*
          } else if (mapToPrint[(numRows-1)/2][h/2] < 10) {
            printMe = printMe + " " + std::to_string(mapToPrint[(numRows-1)/2][h/2]) + " ";
          } else if (mapToPrint[(numRows-1)/2][h/2] < 100) {
            printMe = printMe + " " + std::to_string(mapToPrint[(numRows-1)/2][h/2]);
          */
          } else if ((numRows-1)/2 == finalRow && h/2 == finalCol) {
            printMe = printMe + " x ";
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
    std::cout << printMe << std::endl;
  }
}

void printMapSlam(int mapToPrint[5][9], int horizontalWalls[6][9], int verticalWalls[5][10], int initialRow, int initialCol, std::string heading, int finalRow, int finalCol) {
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
    std::cout << printMe << std::endl;
  }
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

void slam(int floodFillMap[5][9], int horizontalWalls[6][9], int verticalWalls[5][10], int finalRow, int finalCol) {
  
  int e = 0;
  while (e < 5) {
    int f = 0;
    while (f < 9) {
      floodFillMap[e][f] = 45;
      f++;
    }
    e++;
  }
  
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
    }
    if (horizontalWalls[currentRow+1][currentCol] == 0 && !checkQueue((currentRow+1)*10+(currentCol),q2)) {
      floodFillMap[currentRow+1][currentCol] = floodFillMap[currentRow][currentCol] + 1;
      visited.push((currentRow+1)*10+(currentCol));
      q.push((currentRow+1)*10+(currentCol));
    }
    if (verticalWalls[currentRow][currentCol] == 0 && !checkQueue((currentRow)*10+(currentCol-1),q3)) {
      floodFillMap[currentRow][currentCol-1] = floodFillMap[currentRow][currentCol] + 1;
      visited.push((currentRow)*10+(currentCol-1));
      q.push((currentRow)*10+(currentCol-1));
    }
    if (verticalWalls[currentRow][currentCol+1] == 0 && !checkQueue((currentRow)*10+(currentCol+1),q4)) {
      floodFillMap[currentRow][currentCol+1] = floodFillMap[currentRow][currentCol] + 1;
      visited.push((currentRow)*10+(currentCol+1));
      q.push((currentRow)*10+(currentCol+1));
    }
    
  }
  
}

char getHeading(std::shared_ptr<webots::InertialUnit> imu) {
  double yaw = 0;
  double yawAverage = 0;
  for (int j = 0; j < 10; j++) {
    if (j == 0) {
      yawAverage = 0;
    }
    yaw = imu->getRollPitchYaw()[2];
    if (yaw > 0) {
      yaw = -1 * yaw;
    }
    yawAverage = yawAverage + yaw;
  }
  yaw = yawAverage / 10;

  if ((yaw > -0.03) && (yaw < 0.03)) {
    return 'N';
  } else if ((yaw > (-pi / 2) - 0.02) && (yaw < (-pi / 2) + 0.02)) {
    return 'E';
  } else if ((yaw > (pi / 2) - 0.02) && (yaw < (pi / 2) + 0.02)) {
    return 'W';
  } else {
    return 'S';
  }
}

int headingToBearing(char heading) {
  if (heading == 'S') {
    return SOUTH;
  }
  else if (heading == 'E') {
    return EAST;
  }
  else if (heading == 'W') {
    return WEST;
  }
  else {
    return NORTH;
  }
}

int main(int argc, char **argv) {
  
  webots::Robot robot;
  // int timeStep = robot.getBasicTimeStep();
  int timeStep = 64;
  
  std::shared_ptr<webots::Motor> leftMotor{robot.getMotor("left wheel motor")};
  std::shared_ptr<webots::Motor> rightMotor{robot.getMotor("right wheel motor")};
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  
  
  std::shared_ptr<webots::DistanceSensor> leftDS{robot.getDistanceSensor("dsL")};
  std::shared_ptr<webots::DistanceSensor> frontDS{robot.getDistanceSensor("dsF")};
  std::shared_ptr<webots::DistanceSensor> rightDS{robot.getDistanceSensor("dsR")};
  leftDS->enable(timeStep);
  frontDS->enable(timeStep);
  rightDS->enable(timeStep);
  
  
  
  std::shared_ptr<webots::PositionSensor> leftWheelPS{robot.getPositionSensor("left wheel sensor")};
  std::shared_ptr<webots::PositionSensor> rightWheelPS{robot.getPositionSensor("right wheel sensor")};
  leftWheelPS->enable(timeStep);
  rightWheelPS->enable(timeStep);
  
  
  
  std::shared_ptr<webots::InertialUnit> imu{robot.getInertialUnit("IMU")};
  imu->enable(timeStep);
  
  std::shared_ptr<webots::Keyboard> keyboard{robot.getKeyboard()};
  keyboard->enable(timeStep);
  int command{ -1 };
  
  std::shared_ptr<webots::Camera> camera{robot.getCamera("camera")};
  camera->enable(timeStep);
  int width = camera->getWidth();
  int height = camera->getHeight();
  
  robot.step(timeStep);
  
  
  //README for more detail on functions
  std::cout << "[DABABY_MTRN4110_PhaseD] Press 1 for Keyboard Mode" << std::endl;
  std::cout << "[DABABY_MTRN4110_PhaseD] Press 2 for Keyboard Sensor Mode" << std::endl;
  std::cout << "[DABABY_MTRN4110_PhaseD] Press 3 for Exploration Module" << std::endl;
  std::cout << "[DABABY_MTRN4110_PhaseD] Press 4 for Keyboard Mapping and Pathfinding" << std::endl;
  std::cout << "[DABABY_MTRN4110_PhaseD] Press 5 for Seek Green Square Mode" << std::endl;
  
  while (robot.step(timeStep) != -1) {
    const int prevCommand = command;
    command = keyboard->getKey();
    if (command != prevCommand) {
      if (command == '1') {
        std::cout << "[DABABY_MTRN4110_PhaseD] Keyboard Mode";
        while (robot.step(timeStep) != -1) {
        
          int direction{ -1 };
          leftMotor->setVelocity(0.0);
          rightMotor->setVelocity(0.0);
          if (direction == -1) {
              direction = keyboard->getKey();
          }
    
          if (direction == webots::Keyboard::UP) {
              leftMotor->setVelocity(3.14);
              rightMotor->setVelocity(3.14);
          } else if (direction == webots::Keyboard::DOWN) {
              leftMotor->setVelocity(-3.14);
              rightMotor->setVelocity(-3.14);
          } else if (direction == webots::Keyboard::LEFT) {
              leftMotor->setVelocity(-3.14);
              rightMotor->setVelocity(3.14);
          } else if (direction == webots::Keyboard::RIGHT) {
              leftMotor->setVelocity(3.14);
              rightMotor->setVelocity(-3.14);
          } else if (direction == ' ') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
          } else if (direction == 'Q' || direction == 'q') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
              std::cout << "[DABABY_MTRN4110_PhaseD] Quitted Keyboard Mode";
              break;
          }
        
        }
      
      } else if (command == '2') {
        std::cout << "[DABABY_MTRN4110_PhaseD] Keyboard Sensor Mode" << std::endl;
        
        int i = 3;
        int bearing = 180;
        int row = 0;
        int col = 0;
        
        int targetRow = 0, targetCol = 0;
        std::cout << "[DABABY_MTRN4110_PhaseD] Input Target Position Row: " << std::endl;
        while (robot.step(timeStep) != -1) {
          const int prevCommand = command;
          command = keyboard->getKey();
          if (command != prevCommand) {
            if (command >= '0' && command < '5') {
              targetRow = command - '0';
              std::cout << targetRow << std::endl;
              break;
            }
          }
        }
         std::cout << "[DABABY_MTRN4110_PhaseD] Input Target Position Col: " << std::endl;
        while (robot.step(timeStep) != -1) {
          const int prevCommand = command;
          command = keyboard->getKey();
          if (command != prevCommand) {
            if (command >= '0' && command < '9') {
              targetCol = command - '0';
              std::cout << targetCol << std::endl;
              break;
            }
          }
        }
        
        int horizontalWalls[6][9] = {{1,1,1,1,1,1,1,1,1},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{1,1,1,1,1,1,1,1,1}};
        int verticalWalls[5][10] = {{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1}};
        
        printMap(horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
        
        double leftDist = 0.0;
        double forwardDist = 0.0;
        double rightDist = 0.0;
        
        std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." << std::endl;
        checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
        communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
        printMap(horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
        std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
        
        while (robot.step(timeStep) != -1) {
        
          int direction{ -1 };
          if (direction == -1) {
              direction = keyboard->getKey();
          }
          
          if (direction == webots::Keyboard::UP || direction == 'W' || direction == 'w') {
              std::cout << "[DABABY_MTRN4110_PhaseD] Driving..." << std::endl;
              goForward(i, row, col, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
              i++;
              std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." << std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              printMap(horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
              std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::LEFT || direction == 'A' || direction == 'a') {
              std::cout << "[DABABY_MTRN4110_PhaseD] Turning..." << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              printMap(horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
              std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::RIGHT || direction == 'D' || direction == 'd') {
              std::cout << "[DABABY_MTRN4110_PhaseD] Turning..." << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              printMap(horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
              std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
          } else if (direction == 'Q' || direction == 'q') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
              std::cout << "[DABABY_MTRN4110_PhaseD] Quitted Keyboard Sensor Mode";
              break;
          }
        }
      } else if (command == '3') {
        // Exploration Module
        std::cout << "[DABABY_MTRN4110_PhaseD] Exploration Module" << std::endl;
        
        int currentRow = 0, currentCol = 0; //assumes starting pos
        int targetRow = 0, targetCol = 0;
        std::cout << "[DABABY_MTRN4110_PhaseD] Input Target Position Row: " << std::endl;
        while (robot.step(timeStep) != -1) {
          const int prevCommand = command;
          command = keyboard->getKey();
          if (command != prevCommand) {
            if (command >= '0' && command < '5') {
              targetRow = command - '0';
              std::cout << targetRow << std::endl;
              break;
            }
          }
        }
         std::cout << "[DABABY_MTRN4110_PhaseD] Input Target Position Col: " << std::endl;
        while (robot.step(timeStep) != -1) {
          const int prevCommand = command;
          command = keyboard->getKey();
          if (command != prevCommand) {
            if (command >= '0' && command < '9') {
              targetCol = command - '0';
              std::cout << targetCol << std::endl;
              break;
            }
          }
        }
        std::cout << "[DABABY_MTRN4110_PhaseD] Exploring the maze..." << std::endl;
        std::vector<std::vector<int>> map(ROWS, std::vector<int>(COLS));
        std::vector<std::vector<int>> hWallArr(ROWS-1, std::vector<int>(COLS));
        std::vector<std::vector<int>> vWallArr(ROWS, std::vector<int>(COLS-1));
        std::vector<int> walls;
        int i = 0, dir = 0;
        
        int visitedNodes = 0;
        char heading = getHeading(imu);
        int bearing = headingToBearing(heading);
        double dist[3]; // 0: left, 1: north, 2: right
        
        while (visitedNodes < 45) {
          if (!map[currentRow][currentCol]) { // unvisited
            map[currentRow][currentCol] = checkUnvisitedNeighbours(map, currentRow, currentCol);
            visitedNodes++;
          } else {
            map[currentRow][currentCol] += 3;
          }
          // std::cout << "cell[" << currentRow << "][" << currentCol << "] bearing: " << bearing << std::endl;
          checkWalls(timeStep, dist[0], dist[1], dist[2], robot, leftDS, frontDS, rightDS);
          walls = wallsDetected(dist);
          updateWalls(hWallArr, vWallArr, currentRow, currentCol, walls, bearing);

          dir = checkLowestReachableNeighbour(map, currentRow, currentCol, bearing, walls);
          if (bearing == NORTH) {
            if (dir == 1) {
              // std::cout << "N: GOING FORWARD" << std::endl;
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 2) {
              // std::cout << "N: TURNING RIGHT" << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 4) {
              // std::cout << "N: TURNING LEFT" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else {
              // std::cout << "N: ROTATING" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            }
          } else if (bearing == EAST) {
            if (dir == 2) {
              // std::cout << "E: GOING FORWARD" << std::endl;
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 3) {
              // std::cout << "E: TURNING RIGHT" << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 1) {
              // std::cout << "E: TURNING LEFT" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else {
              // std::cout << "E: ROTATING" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            }
          } else if (bearing == SOUTH) {
            if (dir == 3) {
              // std::cout << "S: GOING FORWARD" << std::endl;
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 4) {
              // std::cout << "S: TURNING RIGHT" << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 2) {
              // std::cout << "S: TURNING LEFT" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else {
              // std::cout << "S: ROTATING" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            }
          } else {
            if (dir == 4) {
              // std::cout << "W: GOING FORWARD" << std::endl;
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 1) {
              // std::cout << "W: TURNING RIGHT" << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else if (dir == 3) {
              // std::cout << "W: TURNING LEFT" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            } else {
              // std::cout << "W: ROTATING" << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              goForward(i, currentRow, currentCol, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
            }
          }
        }
        std::cout << "[DABABY_MTRN4110_PhaseD] MAZE ANALYSED" << std::endl;
          
        std::cout << "[DABABY_MTRN4110_PhaseD] PRINTING MAP..." << std::endl;
        printExplorationModuleMap(vWallArr, hWallArr, targetRow, targetCol);

      } else if (command == '4') {
        std::cout << "[DABABY_MTRN4110_PhaseD] Keyboard Mapping and Pathfinding" << std::endl;
        
        int targetRow = 0, targetCol = 0;
        std::cout << "[DABABY_MTRN4110_PhaseD] Input Target Position Row: " << std::endl;
        while (robot.step(timeStep) != -1) {
          const int prevCommand = command;
          command = keyboard->getKey();
          if (command != prevCommand) {
            if (command >= '0' && command < '5') {
              targetRow = command - '0';
              std::cout << targetRow << std::endl;
              break;
            }
          }
        }
        std::cout << "[DABABY_MTRN4110_PhaseD] Input Target Position Col: " << std::endl;
        while (robot.step(timeStep) != -1) {
          const int prevCommand = command;
          command = keyboard->getKey();
          if (command != prevCommand) {
            if (command >= '0' && command < '9') {
              targetCol = command - '0';
              std::cout << targetCol << std::endl;
              break;
            }
          }
        }
        
        int i = 3;
        int bearing = 180;
        int row = 0;
        int col = 0;
        
        int horizontalWalls[6][9] = {{1,1,1,1,1,1,1,1,1},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{1,1,1,1,1,1,1,1,1}};
        int verticalWalls[5][10] = {{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1}};
        int floodFillMap[5][9];
        
        slam(floodFillMap, horizontalWalls, verticalWalls, targetRow, targetCol);
        printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
        
        double leftDist = 0.0;
        double forwardDist = 0.0;
        double rightDist = 0.0;
        
        std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." << std::endl;
        checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
        communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
        slam(floodFillMap, horizontalWalls, verticalWalls, targetRow, targetCol);
        printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
        std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
        
        while (robot.step(timeStep) != -1) {
        
          int direction{ -1 };
          if (direction == -1) {
              direction = keyboard->getKey();
          }
          
          if (direction == webots::Keyboard::UP || direction == 'W' || direction == 'w') {
              std::cout << "[DABABY_MTRN4110_PhaseD] Driving..." << std::endl;
              goForward(i, row, col, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
              i++;
              std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." << std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              slam(floodFillMap, horizontalWalls, verticalWalls, targetRow, targetCol);
              printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
              std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::LEFT || direction == 'A' || direction == 'a') {
              std::cout << "[DABABY_MTRN4110_PhaseD] Turning..." << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              slam(floodFillMap, horizontalWalls, verticalWalls, targetRow, targetCol);
              printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
              std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::RIGHT || direction == 'D' || direction == 'd') {
              std::cout << "[DABABY_MTRN4110_PhaseD] Turning..." << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "[DABABY_MTRN4110_PhaseD] Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              slam(floodFillMap, horizontalWalls, verticalWalls, targetRow, targetCol);
              printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", targetRow, targetCol);
              std::cout << "[DABABY_MTRN4110_PhaseD] Ready for next input!" << std::endl;
          } else if (direction == 'Q' || direction == 'q') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
              std::cout << "[DABABY_MTRN4110_PhaseD] Quitted Keyboard Mapping and Pathfinding Mode";
              break;
          }
        }
      } else if (command == '5') {
        std::cout << "[DABABY_MTRN4110_PhaseD] Seek Green Square Mode" << std::endl;      
        
        double startRun = robot.getTime();
        
        while (robot.step(timeStep) != -1) {
          
          int direction{ -1 };
          leftMotor->setVelocity(0.0);
          rightMotor->setVelocity(0.0);
          if (direction == -1) {
              direction = keyboard->getKey();
          }
    
          if (direction == webots::Keyboard::UP) {
              leftMotor->setVelocity(3.14);
              rightMotor->setVelocity(3.14);
          } else if (direction == webots::Keyboard::DOWN) {
              leftMotor->setVelocity(-3.14);
              rightMotor->setVelocity(-3.14);
          } else if (direction == webots::Keyboard::LEFT) {
              leftMotor->setVelocity(-3.14);
              rightMotor->setVelocity(3.14);
          } else if (direction == webots::Keyboard::RIGHT) {
              leftMotor->setVelocity(3.14);
              rightMotor->setVelocity(-3.14);
          } else if (direction == ' ') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
          } else if (direction == 'Q' || direction == 'q') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
              std::cout << "[DABABY_MTRN4110_PhaseD] Quitted Seek Green Square Mode";
              break;
          }
        
          const unsigned char* image = camera->getImage();
          if (image) {
              int seenGreen = 0;
              
              for (int x = 0; x < width; ++x) {
                  for (int y = 0; y < height; ++y) {
                      seenGreen += camera->imageGetGreen(image, width, x, y);
                  }
              }
              
              if (seenGreen >= 14000000) {
                  leftMotor->setVelocity(0.0);
                  rightMotor->setVelocity(0.0);
                  std::cout << "[DABABY_MTRN4110_PhaseD] Found Target in " << robot.getTime() - startRun << " seconds" << std::endl;
                  break;
              }
          }
        }
      
      } else if (command == webots::Keyboard::END) { // end key
        std::cout << "[DABABY_MTRN4110_PhaseD] End key pressed. Exiting!" << std::endl;
        robot.step(-1);
      }
    }
  }
  return 0;
}
