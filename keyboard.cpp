/*
 * File:          z5308946_MTRN4110_PhaseA.cpp
 * Date:          20/06/2022
 * Description:   Controller of E-puck for Phase A - Driving and Perception
 * Author:        Iniyan Vigneswaran
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
#include <cmath>
#include <queue>

constexpr double maxWheelSpeed = 6.28;
constexpr double wheelRadius = 0.0205;
constexpr double axleLength = 0.062825;

constexpr double oneSquarePos = 0.165;
constexpr double turnPos = M_PI/2*axleLength/2;

const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";

std::string wallDetected(double sensorValue) {
  
  std::string returnValue = "N";
  
  if (sensorValue < 700.0) {
    returnValue = "Y";
  }
  
  return returnValue;
  
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

        std::cout << "[z5308946_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
        std::cout << "[z5308946_MTRN4110_PhaseA] Motion Plan: " << data[0][0] << std::endl;
        std::cout << "[z5308946_MTRN4110_PhaseA] Motion plan read in!" << std::endl;
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
  /*
  double leftStartPos = leftWheelPS->getValue() * wheelRadius;
  double rightStartPos = rightWheelPS->getValue() * wheelRadius;
  
  if (i == 3) {
    leftStartPos = 0;
    rightStartPos = 0;
  }
  
  leftMotor->setVelocity(0.5*maxWheelSpeed);
  rightMotor->setVelocity(0.5*maxWheelSpeed);
  
  while (robot.step(timeStep) != -1) {
  
    if (leftWheelPS->getValue() * wheelRadius - leftStartPos >= oneSquarePos && rightWheelPS->getValue() * wheelRadius - rightStartPos >= oneSquarePos) {
      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0);
      break;
    }
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
  */
  
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
  /*
  int turnAngle = 180 + direction*90;
  
  leftMotor->setVelocity(direction*-0.5*maxWheelSpeed);
  rightMotor->setVelocity(direction*0.5*maxWheelSpeed);
  
  while (robot.step(timeStep) != -1) {
    
    double myAngle = ((bearing + turnAngle) % 360)/1.0 - std::fmod(((-imu->getRollPitchYaw()[2] + 2.0 * M_PI ) * 180.0/M_PI),360);
    if ((myAngle >= -20.0 && myAngle <= 20.0) || (myAngle >= 340.0 || myAngle <= -340.0)) {
      break;
    }
  }
  
  leftMotor->setVelocity(direction*-0.01*maxWheelSpeed);
  rightMotor->setVelocity(direction*0.01*maxWheelSpeed);
  
  while (robot.step(timeStep) != -1) {
    
    double iter = 0;
    double sum = 0;
    
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    
    double tempTime = robot.getTime();
    while (robot.step(timeStep) != -1) {
      
      double angle = std::fmod(((-imu->getRollPitchYaw()[2] + 2.0 * M_PI ) * 180.0/M_PI),360);
      if ((bearing + turnAngle) % 360 == 0 && angle >= 180.0) {
        sum = sum - 360 + angle;
      } else if ((bearing + turnAngle) % 360 == 90 && angle >= 270.0) {
        sum = sum - 360 + angle;
      } else if ((bearing + turnAngle) % 360 == 270 && angle <= 90.0) {
        sum = sum + 360 + angle;
      } else {
        sum = sum + angle;
      }
      
      iter = iter + 1;
      if (robot.getTime() - tempTime >= 10) {
        break;
      }
    }
    
    leftMotor->setVelocity(direction*-0.01*maxWheelSpeed);
    rightMotor->setVelocity(direction*0.01*maxWheelSpeed);
    
    if (sum/iter >= -0.35+1.0*((bearing + turnAngle)%360) && sum/iter <= 0.35+1.0*((bearing + turnAngle)%360)) {
      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0);
      break;
    }
    
  }
  
  bearing = (bearing + turnAngle) % 360;
  */
  
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
  std::cout << "[z5308946_MTRN4110_PhaseA] Step: " << std::setw(3) << std::setfill('0') << step << ", Row: " << row << ", Column: " << col << ", Heading: " << heading << ", Left Wall: " << leftWall << ", Front Wall: " << frontWall << ", Right Wall: " << rightWall << std::endl;
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
  //std::string fileName = robotName + ".csv";
  
  std::vector<std::vector<std::string>> data;  
  
  try {
    
    //CsvProcessor csvProcessor{ robotName, fileName };
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
  
  std::cout << "[z5308946_MTRN4110_PhaseA] Motion plan executed!" << std::endl;
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
  
  
  
  
  
  
  
  
  while (robot.step(timeStep) != -1) {
    const int prevCommand = command;
    command = keyboard->getKey();
    if (command != prevCommand) {
      
      if (command == '1') {
      
        phaseA(timeStep, robot, leftMotor, rightMotor, leftDS, frontDS, rightDS, leftWheelPS, rightWheelPS, imu);
      
      } else if (command == '2') {
        std::cout << "Keyboard Mode";
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
              std::cout << "End";
              break;
          }
        
        }
      
      } else if (command == '3') {
        std::cout << "Keyboard Sensor Mode" << std::endl;
        
        int i = 3;
        int bearing = 180;
        int row = 0;
        int col = 0;
        
        int horizontalWalls[6][9] = {{1,1,1,1,1,1,1,1,1},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{1,1,1,1,1,1,1,1,1}};
        int verticalWalls[5][10] = {{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1}};
        
        printMap(horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
        
        double leftDist = 0.0;
        double forwardDist = 0.0;
        double rightDist = 0.0;
        
        std::cout << "Checking Walls..." << std::endl;
        checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
        communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
        printMap(horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
        std::cout << "Ready for next input!" << std::endl;
        
        while (robot.step(timeStep) != -1) {
        
          int direction{ -1 };
          if (direction == -1) {
              direction = keyboard->getKey();
          }
          
          if (direction == webots::Keyboard::UP || direction == 'W' || direction == 'w') {
              goForward(i, row, col, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
              i++;
              std::cout << "Checking Walls..." << std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              printMap(horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
              std::cout << "Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::LEFT || direction == 'A' || direction == 'a') {
              std::cout << "Turning..." << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              printMap(horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
              std::cout << "Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::RIGHT || direction == 'D' || direction == 'd') {
              std::cout << "Turning..." << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              printMap(horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
              std::cout << "Ready for next input!" << std::endl;
          } else if (direction == 'Q' || direction == 'q') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
              std::cout << "End";
              break;
          }
        }
      } else if (command == '4') {
        // Exploration Module
        
        
        
        
        
        
        
        
        
      } else if (command == '5') {
        std::cout << "Keyboard SLAM" << std::endl;
        
        int i = 3;
        int bearing = 180;
        int row = 0;
        int col = 0;
        
        int horizontalWalls[6][9] = {{1,1,1,1,1,1,1,1,1},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{1,1,1,1,1,1,1,1,1}};
        int verticalWalls[5][10] = {{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,0,0,1}};
        int floodFillMap[5][9];
        
        slam(floodFillMap, horizontalWalls, verticalWalls, 2, 4);
        printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
        
        double leftDist = 0.0;
        double forwardDist = 0.0;
        double rightDist = 0.0;
        
        std::cout << "Checking Walls..." << std::endl;
        checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
        communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
        slam(floodFillMap, horizontalWalls, verticalWalls, 2, 4);
        printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
        std::cout << "Ready for next input!" << std::endl;
        
        while (robot.step(timeStep) != -1) {
        
          int direction{ -1 };
          if (direction == -1) {
              direction = keyboard->getKey();
          }
          
          if (direction == webots::Keyboard::UP || direction == 'W' || direction == 'w') {
              goForward(i, row, col, bearing, timeStep, robot, leftMotor, rightMotor, leftWheelPS, rightWheelPS);
              i++;
              std::cout << "Checking Walls..." << std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              slam(floodFillMap, horizontalWalls, verticalWalls, 2, 4);
              printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
              std::cout << "Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::LEFT || direction == 'A' || direction == 'a') {
              std::cout << "Turning..." << std::endl;
              turnNinety(i, 1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              slam(floodFillMap, horizontalWalls, verticalWalls, 2, 4);
              printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
              std::cout << "Ready for next input!" << std::endl;
          } else if (direction == webots::Keyboard::RIGHT || direction == 'D' || direction == 'd') {
              std::cout << "Turning..." << std::endl;
              turnNinety(i, -1, bearing, timeStep, robot, leftMotor, rightMotor, imu);
              i++;
              std::cout << "Checking Walls..." <<  std::endl;
              checkWalls(timeStep, leftDist, forwardDist, rightDist, robot, leftDS, frontDS, rightDS);
              communicateWalls(horizontalWalls, verticalWalls, row, col, bearing, leftDist, forwardDist, rightDist);
              slam(floodFillMap, horizontalWalls, verticalWalls, 2, 4);
              printMapSlam(floodFillMap, horizontalWalls, verticalWalls, 0, 0, "S", 2, 4);
              std::cout << "Ready for next input!" << std::endl;
          } else if (direction == 'Q' || direction == 'q') {
              leftMotor->setVelocity(0.0);
              rightMotor->setVelocity(0.0);
              std::cout << "End";
              break;
          }
        }
      } else if (command == '6') {
        std::cout << "Seek Green square mode" << std::endl;      
        
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
              std::cout << "End";
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
                  std::cout << "Found Target in " << robot.getTime() - startRun << " seconds" << std::endl;
                  break;
              }

          }
        
        }
      
      }
    }
  }
  
  return 0;
}
