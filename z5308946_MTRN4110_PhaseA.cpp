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

#include <iostream>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <fstream>
#include <cmath>

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
                //double doubReadLine = std::stod(subReadLine);
                result.push_back(subReadLine);
                //result.push_back(doubReadLine);
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

int main(int argc, char **argv) {
  
  webots::Robot robot;
  // int timeStep = robot.getBasicTimeStep();
  int timeStep = 64;
  
  std::unique_ptr<webots::Motor> leftMotor{robot.getMotor("left wheel motor")};
  std::unique_ptr<webots::Motor> rightMotor{robot.getMotor("right wheel motor")};
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  
  
  std::unique_ptr<webots::DistanceSensor> leftDS{robot.getDistanceSensor("dsL")};
  std::unique_ptr<webots::DistanceSensor> frontDS{robot.getDistanceSensor("dsF")};
  std::unique_ptr<webots::DistanceSensor> rightDS{robot.getDistanceSensor("dsR")};
  leftDS->enable(timeStep);
  frontDS->enable(timeStep);
  rightDS->enable(timeStep);
  
  
  
  std::unique_ptr<webots::PositionSensor> leftWheelPS{robot.getPositionSensor("left wheel sensor")};
  std::unique_ptr<webots::PositionSensor> rightWheelPS{robot.getPositionSensor("right wheel sensor")};
  leftWheelPS->enable(timeStep);
  rightWheelPS->enable(timeStep);
  
  
  
  std::unique_ptr<webots::InertialUnit> imu{robot.getInertialUnit("IMU")};
  imu->enable(timeStep);
  
  robot.step(timeStep);
  
  
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
  
  double dsLsum = 0;
  double dsFsum = 0;
  double dsRsum = 0;
  double dsiter = 0;
  
  double temptime = robot.getTime();
  while (robot.step(timeStep) != -1) {
    
    dsiter = dsiter + 1;
    dsLsum = dsLsum + leftDS->getValue();
    dsFsum = dsFsum + frontDS->getValue();
    dsRsum = dsRsum + rightDS->getValue();
    
    if (robot.getTime() - temptime >= 10) {
      break;
    }
  }
  
  
  std::string zeroHeading = bearToHead(bearing);
  std::string zeroLeftWall = wallDetected(dsLsum/dsiter);
  std::string zeroFrontWall = wallDetected(dsFsum/dsiter);
  std::string zeroRightWall = wallDetected(dsRsum/dsiter);
  std::cout << std::endl;
  std::cout << "[z5308946_MTRN4110_PhaseA] Step: " << std::setw(3) << std::setfill('0') << step << ", Row: " << row << ", Column: " << col << ", Heading: " << zeroHeading << ", Left Wall: " << zeroLeftWall << ", Front Wall: " << zeroFrontWall << ", Right Wall: " << zeroRightWall << std::endl;
  std::vector<std::string> zeroLine;
  zeroLine.push_back(std::to_string(step));
  zeroLine.push_back(std::to_string(row));
  zeroLine.push_back(std::to_string(col));
  zeroLine.push_back(zeroHeading);
  zeroLine.push_back(zeroLeftWall);
  zeroLine.push_back(zeroFrontWall);
  zeroLine.push_back(zeroRightWall);
  
  try {
      CsvProcessor csvWriter{ robotName, MOTION_EXECUTION_FILE_NAME };
      csvWriter.writeLineToCsv(zeroLine);
  }
  catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
  }
  
  for (int i = 3; i < dataSize; i++) {
    
    if (dataString[i] == 'F') {
      
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
      
      step++;
      
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
      
      
      std::string heading = bearToHead(bearing);
      std::string leftWall = wallDetected(dsLSum/dsIter);
      std::string frontWall = wallDetected(dsFSum/dsIter);
      std::string rightWall = wallDetected(dsRSum/dsIter);
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
      
    } else if (dataString[i] == 'L') {
      
      leftMotor->setVelocity(-0.01*maxWheelSpeed);
      rightMotor->setVelocity(0.01*maxWheelSpeed);
      
      while (robot.step(timeStep) != -1) {
        
        double iter = 0;
        double sum = 0;
        
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        
        double tempTime = robot.getTime();
        while (robot.step(timeStep) != -1) {
          
          double angle = std::fmod(((-imu->getRollPitchYaw()[2] + 2.0 * M_PI ) * 180.0/M_PI),360);
          if ((bearing + 270) % 360 == 0 && angle >= 180.0) {
            sum = sum - 360 + angle;
          } else if ((bearing + 270) % 360 == 90 && angle >= 270.0) {
            sum = sum - 360 + angle;
          } else if ((bearing + 270) % 360 == 270 && angle <= 90.0) {
            sum = sum + 360 + angle;
          } else {
            sum = sum + angle;
          }
          
          iter = iter + 1;
          if (robot.getTime() - tempTime >= 10) {
            break;
          }
        }
        
        leftMotor->setVelocity(-0.01*maxWheelSpeed);
        rightMotor->setVelocity(0.01*maxWheelSpeed);
        
        if (sum/iter >= -0.35+1.0*((bearing + 270)%360) && sum/iter <= 0.35+1.0*((bearing + 270)%360)) {
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(0);
          break;
        }
        
      }
      
      bearing = (bearing + 270) % 360;
      
      step++;
      
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
      
      
      std::string heading = bearToHead(bearing);
      std::string leftWall = wallDetected(dsLSum/dsIter);
      std::string frontWall = wallDetected(dsFSum/dsIter);
      std::string rightWall = wallDetected(dsRSum/dsIter);
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

    } else if (dataString[i] == 'R') {
      
      leftMotor->setVelocity(0.01*maxWheelSpeed);
      rightMotor->setVelocity(-0.01*maxWheelSpeed);
      
      while (robot.step(timeStep) != -1) {
        
        double iter = 0;
        double sum = 0;
        
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        
        double tempTime = robot.getTime();
        while (robot.step(timeStep) != -1) {
          
          double angle = std::fmod(((-imu->getRollPitchYaw()[2] + 2.0 * M_PI ) * 180.0/M_PI),360);
          if ((bearing + 90) % 360 == 0 && angle >= 180.0) {
            sum = sum - 360 + angle;
          } else if ((bearing + 90) % 360 == 90 && angle >= 270.0) {
            sum = sum - 360 + angle;
          } else if ((bearing + 90) % 360 == 270 && angle <= 90.0) {
            sum = sum + 360 + angle;
          } else {
            sum = sum + angle;
          }
          
          iter = iter + 1;
          if (robot.getTime() - tempTime >= 10) {
            break;
          }
        }
        
        leftMotor->setVelocity(0.01*maxWheelSpeed);
        rightMotor->setVelocity(-0.01*maxWheelSpeed);
        
        if (sum/iter >= -0.35+1.0*((bearing + 90)%360) && sum/iter <= 0.35+1.0*((bearing + 90)%360)) {
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(0);
          break;
        }
        
      }
      
      bearing = (bearing + 90) % 360;
      
      step++;
      
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
      
      
      std::string heading = bearToHead(bearing);
      std::string leftWall = wallDetected(dsLSum/dsIter);
      std::string frontWall = wallDetected(dsFSum/dsIter);
      std::string rightWall = wallDetected(dsRSum/dsIter);
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
    
  }
  
  std::cout << "[z5308946_MTRN4110_PhaseA] Motion plan executed!" << std::endl;
   
  return 0;
}