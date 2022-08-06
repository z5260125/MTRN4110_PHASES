// File:          z5260176_MTRN4110_PhaseA.cpp
// Description:   Controller for e-puck for Phase A
// Platform:      Windows

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>

#include <memory>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <array>
#include <cstdlib>

constexpr double maxWheelSpeed = 6.28; // rad/s
const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXEC_FILE_NAME = "../../MotionExecution.csv";
const std::string prefix = "[z5260176_MTRN4110_PhaseA] ";

// read line from MotionPlan.txt and returns a string
std::string readMotionPlan() {
    // create a std::ifstream object and initialise with txt file
    std::ifstream inFile(MOTION_PLAN_FILE_NAME);
    std::string data;
    // check whether the file is open
    if (inFile.is_open()) {
        std::cout << prefix << "Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
        // read all lines and add to data string
        std::string str;
        while (std::getline(inFile, str)) {
          data.append(str);
        }
        std::cout << prefix << "Motion Plan: " << str << std::endl;
        std::cout << prefix << "Motion plan read in!" << std::endl;
    }
    else {
        // throw an error message
        throw std::runtime_error(prefix + " Reading data failed!");
    }
    inFile.close();
    return data;
}

// write to MotionExecution.csv
void writeToFile(int *p, int step, std::string isLeftWall, std::string isFrontWall, std::string isRightWall) {
  std::ofstream inFile;
  if (step == 0) {
    inFile.open(MOTION_EXEC_FILE_NAME);
    inFile << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
    inFile.close();
  }
  inFile.open(MOTION_EXEC_FILE_NAME, std::ios::in | std::ios::app);
  inFile << step << "," << p[0] - 48 << "," << p[1] - 48 << "," << (char)p[2] 
  << "," << isLeftWall << "," << isFrontWall << "," << isRightWall << "\n";
  inFile.close();
}

std::string checkWall(double dist) {
  if (dist <= 660.0) {
    return "Y";
  }
  return "N";
}

void printInConsole(int *p, int step, std::string isLeftWall, std::string isFrontWall,
std::string isRightWall) {
  std::cout << prefix << "Step: " << std::setfill('0') << std::setw(3) << step
  << ", Row: " << p[0] - 48 << ", Column: " << p[1] - 48
  << ", Heading: " << (char)p[2] << ", Left Wall: " << isLeftWall << ", Front Wall: "
  << isFrontWall << ", Right Wall: " << isRightWall << std::endl;

  writeToFile(p, step, isLeftWall, isFrontWall, isRightWall);
}

bool reachedTarget(double currYaw, bool left, char heading, double range) {
  // If the robot is turning left
  if (left) {
    if (heading == 'N') { // 0 yaw -> W
      return currYaw <= 1.5708 + range && currYaw >= 1.5708 - range;
    } else if (heading == 'E') { // -1.5708 yaw -> N
      return currYaw <= 0 + range && currYaw >= 0 - range;
    } else if (heading == 'S') { // < 3.14159 or > -3.14159 -> E
      return currYaw <= -1.5708 + range && currYaw >= -1.5708 - range;
    } else if (heading == 'W') { // 1.5708 yaw -> S
      return (currYaw >= 3.14159 - range - 0.04 && currYaw <= 3.14159) //positive pi
      || (currYaw >= -3.14159 && currYaw <= -3.14159 + range + 0.04); //negative pi
    }
  } else {
      if (heading == 'N') { // -> E
      return currYaw <= -1.5708 + range && currYaw >= -1.5708 - range;
    } else if (heading == 'E') { // -> S
      return (currYaw >= 3.14159 - range - 0.03 && currYaw <= 3.14159) //positive pi
      || (currYaw >= -3.14159 && currYaw <= -3.14159 + range + 0.03); //negative pi
    } else if (heading == 'S') { // -> W
      return currYaw <= 1.5708 + range && currYaw >= 1.5708 - range;
    } else if (heading == 'W') { // -> N
      return currYaw <= 0.6285 + range && currYaw >= 0.6285 - range;
    }
  }
  throw std::runtime_error(prefix + "Invalid Heading");
}

// update coordinates after moving forward
int *updatePosition(int *p) {
  char h = p[2];
  // change new coordinates based on heading
  if (h == 'N') {
    p[0]--;
  } else if (h == 'E') {
    p[1]++;
  } else if (h == 'S') {
    p[0]++;
  } else if (h == 'W') {
    p[1]--;
  } else {
    throw std::runtime_error(prefix + " Invalid Heading!");
  }
  return p;
}

// update heading after turning left
int *updateHeadingLeft(int *p) {
  if (p[2] == 'N') {
    p[2] = 'W';
  } else if (p[2] == 'E') {
    p[2] = 'N';
  } else if (p[2] == 'S') {
    p[2] = 'E';
  } else if (p[2] == 'W') {
    p[2] = 'S';
  } else {
    throw std::runtime_error(prefix + " Invalid Direction!");
  }
  return p;
}

// update heading after turning right
int *updateHeadingRight(int *p) {
  if (p[2] == 'N') {
    p[2] = 'E';
  } else if (p[2] == 'E') {
    p[2] = 'S';
  } else if (p[2] == 'S') {
    p[2] = 'W';
  } else if (p[2] == 'W') {
    p[2] = 'N';
  } else {
    throw std::runtime_error(prefix + " Invalid Direction!");
  }
  return p;
}

int *updatePositionAndHeading(char c, int *p) {
  if (c == 'F') {
    return updatePosition(p);
  } else if (c == 'L') {
    return updateHeadingLeft(p);
  } else if (c == 'R') {
    return updateHeadingRight(p);
  } else {
    throw std::runtime_error(prefix + " Invalid Direction");
  }
  return p;
}

// reads the first 3 characters of motion plan string
// and then returns an array with the position and heading
// of the e-puck

int main(int argc, char **argv) {
  // create the Robot instance
  webots::Robot robot;

  // set the time step of the current world.
  const int timeStep = 64;
  
  // Get the motors
  std::unique_ptr<webots::Motor> leftMotor{robot.getMotor("left wheel motor")};
  std::unique_ptr<webots::Motor> rightMotor{robot.getMotor("right wheel motor")};
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // Get the sensors
  std::unique_ptr<webots::DistanceSensor> dsL{robot.getDistanceSensor("dsL")};
  std::unique_ptr<webots::DistanceSensor> dsF{robot.getDistanceSensor("dsF")};
  std::unique_ptr<webots::DistanceSensor> dsR{robot.getDistanceSensor("dsR")};
  dsL->enable(timeStep);
  dsF->enable(timeStep);
  dsR->enable(timeStep);
  
  // Get the IMU
  std::unique_ptr<webots::InertialUnit> IMU {robot.getInertialUnit("IMU")};
  IMU->enable(timeStep);
  
  std::string str = readMotionPlan();
  // take first 3 characters in string for the initial position and heading
  int currPos[3] = {str[0], str[1], str[2]};
  str.erase(0,3);
  
  int *p = currPos; // pointer to current position array
  int i = 0; // counter for characters in str
  int once = 0;
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot.step(timeStep) != -1) {
    // double yaw = 0;
    // for (int j = 0; j < 10; j++) {
      // rightMotor->setVelocity(0);
      // leftMotor->setVelocity(0);
      // robot.step(timeStep);
      // yaw += abs(IMU->getRollPitchYaw()[2]);
    // }
    // std::cout << "yaw: " << yaw/10.0 << std::endl;
    // continue;
    if (i == (int)str.size()) {
      leftMotor->setVelocity(0.0);
      rightMotor->setVelocity(0.0);
      std::cout << "Motion plan executed!" << std::endl;
      robot.step(-1);
    }
    if (once == 0) {
      double dsLSum = 0, dsFSum = 0, dsRSum = 0;
      for (int j = 0; j < 10; j++) {
        rightMotor->setVelocity(0);
        leftMotor->setVelocity(0);
        robot.step(timeStep);
        dsLSum += dsL->getValue();
        dsFSum += dsF->getValue();
        dsRSum += dsR->getValue();
      }
      std::string isLeftWall = checkWall(dsLSum/10.0);
      std::string isFrontWall = checkWall(dsFSum/10.0);
      std::string isRightWall = checkWall(dsRSum/10.0);
      printInConsole(p, i, isLeftWall, isFrontWall, isRightWall);
      once = 1;
      continue;
    }
    
    double rightSpeed = 0.5 * maxWheelSpeed;
    double leftSpeed = 0.5 * maxWheelSpeed;
    double currYaw = IMU->getRollPitchYaw()[2];
    if (str[i] == 'F' && once) {
      // Go forward
      leftMotor->setVelocity(leftSpeed);
      rightMotor->setVelocity(rightSpeed);
      robot.step(2565);
      p = updatePositionAndHeading(str[i], p);
      i++;
      double dsLSum = 0, dsFSum = 0, dsRSum = 0;
      for (int j = 0; j < 10; j++) {
        // stop motor, add yaw for each time step
        rightMotor->setVelocity(0);
        leftMotor->setVelocity(0);
        robot.step(timeStep);
        dsLSum += dsL->getValue();
        dsFSum += dsF->getValue();
        dsRSum += dsR->getValue();
      }
      std::string isLeftWall = checkWall(dsLSum/10.0);
      std::string isFrontWall = checkWall(dsFSum/10.0);
      std::string isRightWall = checkWall(dsRSum/10.0);
      printInConsole(p, i, isLeftWall, isFrontWall, isRightWall);
    } else if (str[i] == 'L' && once) {
      // Turn left
      if (reachedTarget(currYaw, true, p[2], 0.12)) { //if within 0.1 radians
        // check 10 times
        double sum = 0, dsLSum = 0, dsFSum = 0, dsRSum = 0;
        for (int j = 0; j < 10; j++) {
          // stop motor, add yaw for each time step
          rightMotor->setVelocity(0);
          leftMotor->setVelocity(0);
          robot.step(timeStep);
          if (p[2] == 'W') {
            sum += abs(IMU->getRollPitchYaw()[2]);
          } else {
            sum += IMU->getRollPitchYaw()[2];
          }
          dsLSum += dsL->getValue();
          dsFSum += dsF->getValue();
          dsRSum += dsR->getValue();
        }
        if (reachedTarget(sum/10.0, true, p[2], 0.01)) {
          p = updatePositionAndHeading(str[i], p);
          i++;
          std::string isLeftWall = checkWall(dsLSum/10.0);
          std::string isFrontWall = checkWall(dsFSum/10.0);
          std::string isRightWall = checkWall(dsRSum/10.0);
          printInConsole(p, i, isLeftWall, isFrontWall, isRightWall);
          continue;
        }
        leftMotor->setVelocity(-leftSpeed/20);
        rightMotor->setVelocity(rightSpeed/20);
      } else {
        leftMotor->setVelocity(-leftSpeed/5);
        rightMotor->setVelocity(rightSpeed/5);
      }
    } else if (str[i] == 'R' && once) {
      // Turn right
      if (reachedTarget(currYaw, false, p[2], 0.12)) { //if within 0.1 radians
        // check 10 times
        double sum = 0, dsLSum = 0, dsFSum = 0, dsRSum = 0;
        for (int j = 0; j < 10; j++) {
          // stop motor, add yaw for each time step
          rightMotor->setVelocity(0);
          leftMotor->setVelocity(0);
          robot.step(timeStep);
          if (p[2] == 'E') {
            sum += abs(IMU->getRollPitchYaw()[2]);
          } else {
            sum += IMU->getRollPitchYaw()[2];
          }
          dsLSum += dsL->getValue();
          dsFSum += dsF->getValue();
          dsRSum += dsR->getValue();
        }
        if (reachedTarget(sum/10.0, false, p[2], 0.01)) {
          rightMotor->setVelocity(0);
          leftMotor->setVelocity(0);
          p = updatePositionAndHeading(str[i], p);
          i++;
          std::string isLeftWall = checkWall(dsLSum/10.0);
          std::string isFrontWall = checkWall(dsFSum/10.0);
          std::string isRightWall = checkWall(dsRSum/10.0);
          printInConsole(p, i, isLeftWall, isFrontWall, isRightWall);
          continue;
        }
        rightMotor->setVelocity(-rightSpeed/11);
        leftMotor->setVelocity(leftSpeed/11);
      } else {
        rightMotor->setVelocity(-rightSpeed/5);
        leftMotor->setVelocity(leftSpeed/5);
      }
    } else {
      throw std::runtime_error(prefix + " Invalid Direction");
    }
  };

  // Enter here exit cleanup code.

  // delete robot;
  return 0;
}
