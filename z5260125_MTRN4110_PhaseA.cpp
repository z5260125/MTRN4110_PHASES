// File: z5260125_MTRN4110_PhaseA.cpp
// Date: 18/06/2022
// Description: MTRN4110 PhaseA
// Author: Kaila Vergara
// Modifications:
// Platform: Windows

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>

#include <memory>
#include <fstream>
#include <iomanip>

#define TIME_STEP 64

constexpr double maxWheelSpeed = 6.28;
constexpr double pi = 3.14;
constexpr double obstacleThreshold = 650;

const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
const std::string prefix = "[z5260125_MTRN4110_PhaseA] ";

int main(int argc, char **argv) {
  // Robot initialisation
  webots::Robot robot;
  
  // Motor initialisation
  std::unique_ptr<webots::Motor> leftMotor{ robot.getMotor("left wheel motor") };
  std::unique_ptr<webots::Motor> rightMotor{ robot.getMotor("right wheel motor") };
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // Distance sensor initialisation
  std::unique_ptr<webots::DistanceSensor> dsF{ robot.getDistanceSensor("dsF") };
  std::unique_ptr<webots::DistanceSensor> dsL{ robot.getDistanceSensor("dsL") };
  std::unique_ptr<webots::DistanceSensor> dsR{ robot.getDistanceSensor("dsR") };
  dsF->enable(TIME_STEP);
  dsL->enable(TIME_STEP);
  dsR->enable(TIME_STEP);
  
  // IMU initialisation
  std::unique_ptr<webots::InertialUnit> iu{ robot.getInertialUnit("IMU") };
  iu->enable(TIME_STEP);
  
  // Read path from file
  std::ifstream motionPlan;
  motionPlan.open(MOTION_PLAN_FILE_NAME);
  std::string path;
  motionPlan >> path;
  int pathLength = path.length();
  int row = path[0] - 48;
  int column = path[1] - 48;
  char heading = path[2];
  std::cout << prefix << "Reading in motion plan from " << MOTION_PLAN_FILE_NAME << "..." << std::endl;
  std::cout << prefix << "Motion plan: " << path << std::endl;
  std::cout << prefix << "Motion plan read in!" << std::endl;
  std::cout << prefix << "Executing motion plan..." << std::endl;
  
  // Write path to file
  std::ofstream motionExec(MOTION_EXECUTION_FILE_NAME);
  motionExec << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall" << std::endl;
  
  int i = 3;
  double ds[3];
  char wall[3];
  int stepCount = 0;
  double yaw = 0.0;
  double yawAverage = 0.0;
  double dsFAverage = 0.0;
  double dsLAverage = 0.0;
  double dsRAverage = 0.0;
  
  while (robot.step(TIME_STEP) != -1) {    
    // Detect walls around robot
    for (int j = 0; j < 10; j++) {
      if (j == 0) {
        dsLAverage = 0;
        dsFAverage = 0;
        dsRAverage = 0;
      }
      ds[0] = dsL->getValue();
      ds[1] = dsF->getValue();
      ds[2] = dsR->getValue();
      dsLAverage = dsLAverage + ds[0];
      dsFAverage = dsFAverage + ds[1];
      dsRAverage = dsRAverage + ds[2];
      robot.step(TIME_STEP);
    }
    ds[0] = dsLAverage / 10;
    ds[1] = dsFAverage / 10;
    ds[2] = dsRAverage / 10;
    
    for (int j = 0; j < 3; j++) {
      if (ds[j] < obstacleThreshold) {
        wall[j] = 'Y';
      } else {
        wall[j] = 'N';
      }
    }
    
    // Write to file and print to console
    motionExec << stepCount << ','
               << row << ','
               << column << ','
               << heading << ','
               << wall[0] << ','
               << wall[1] << ','
               << wall[2] << ','
               << std::endl;
    std::cout << prefix 
              << "Step: " << std::setw(3) << std::setfill('0') << stepCount
              << ", Row: " << row
              << ", Column: " << column
              << ", Heading:  " << heading
              << ", Left Wall: " << wall[0]
              << ", Front Wall: " << wall[1]
              << ", Right Wall: " << wall[2]
              << std::endl;
    
    // Follow path
    if (i < pathLength) {
      // Correct position using iu
      if (path[i] == 'F') {
        leftMotor->setVelocity(0.5 * maxWheelSpeed);
        rightMotor->setVelocity(0.5 * maxWheelSpeed);
        robot.step(2600);
        
        // Update heading and position
        if (heading == 'N') {
          row--;
        } else if (heading == 'E') {
          column++;
        } else if (heading == 'W') {
          column--;
        } else if (heading == 'S') {
          row++;
        }
      } else if (path[i] == 'L') {
        leftMotor->setVelocity(-0.5 * maxWheelSpeed);
        rightMotor->setVelocity(0.5 * maxWheelSpeed);
        robot.step(750);
      
        // Update heading and position
        if (heading == 'N') {
          heading = 'W';
        } else if (heading == 'E') {
          heading = 'N';
        } else if (heading == 'W') {
          heading = 'S';
        } else if (heading == 'S') {
          heading = 'E';
        }
      } else if (path[i] == 'R') {
        leftMotor->setVelocity(0.5 * maxWheelSpeed);
        rightMotor->setVelocity(-0.5 * maxWheelSpeed);
        robot.step(750);
      
        // Update heading and position
        if (heading == 'N') {
          heading = 'E';
        } else if (heading == 'E') {
          heading = 'S';
        } else if (heading == 'W') {
          heading = 'N';
        } else if (heading == 'S') {
          heading = 'W';
        }
      }
    } else {
      leftMotor->setVelocity(0.0);
      rightMotor->setVelocity(0.0);
      motionExec.close();
      std::cout << prefix << "Motion plan executed!" << std::endl;
      robot.step(-1);
    }
    
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);    
    
    // Fix rotation
    for (int j = 0; j < 10; j++) {
      if (j == 0) {
        yawAverage = 0;
      }
      yaw = iu->getRollPitchYaw()[2];
      if (heading == 'S' && yaw > 0) {
        yaw = -1 * yaw;
      }
      yawAverage = yawAverage + yaw;
      robot.step(TIME_STEP); 
    }
    yaw = yawAverage / 10;
    
    if (heading == 'N') {
      while (!((yaw > -0.03) && (yaw < 0.03))) {
        if (yaw > 0) {
          leftMotor->setVelocity(0.05 * maxWheelSpeed);
          rightMotor->setVelocity(-0.05 * maxWheelSpeed);
        } else {
          leftMotor->setVelocity(-0.05 * maxWheelSpeed);
          rightMotor->setVelocity(0.05 * maxWheelSpeed);
        }
        robot.step(TIME_STEP);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        for (int j = 0; j < 10; j++) {
          if (j == 0) {
            yawAverage = 0;
          }
          yaw = iu->getRollPitchYaw()[2];
          yawAverage = yawAverage + yaw;
          robot.step(TIME_STEP); 
        }
        yaw = yawAverage / 10;
      }
    } else if (heading == 'E') {
      while (!((yaw > (-pi/2) - 0.02) && (yaw < (-pi/2) + 0.02))) {
        if (yaw > -pi/2) {
          leftMotor->setVelocity(0.05 * maxWheelSpeed);
          rightMotor->setVelocity(-0.05 * maxWheelSpeed);
        } else {
          leftMotor->setVelocity(-0.05 * maxWheelSpeed);
          rightMotor->setVelocity(0.05 * maxWheelSpeed);
        }
        robot.step(TIME_STEP);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        for (int j = 0; j < 10; j++) {
          if (j == 0) {
            yawAverage = 0;
          }
          yaw = iu->getRollPitchYaw()[2];
          yawAverage = yawAverage + yaw;
          robot.step(TIME_STEP); 
        }
        yaw = yawAverage / 10;
      }
    } else if (heading == 'W') {
      while (!((yaw > (pi/2) - 0.02) && (yaw < (pi/2) + 0.02))) {
        if (yaw > pi/2) {
          leftMotor->setVelocity(0.05 * maxWheelSpeed);
          rightMotor->setVelocity(-0.05 * maxWheelSpeed);
        } else {
          leftMotor->setVelocity(-0.05 * maxWheelSpeed);
          rightMotor->setVelocity(0.05 * maxWheelSpeed);
        }
        robot.step(TIME_STEP);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        for (int j = 0; j < 10; j++) {
          if (j == 0) {
            yawAverage = 0;
          }
          yaw = iu->getRollPitchYaw()[2];
          yawAverage = yawAverage + yaw;
          robot.step(TIME_STEP); 
        }
        yaw = yawAverage / 10;
      }
    } else {
      while (!((yaw > -pi - 0.05) && (yaw < -pi + 0.05))) {
        if (yaw > -pi) {
          leftMotor->setVelocity(0.05 * maxWheelSpeed);
          rightMotor->setVelocity(-0.05 * maxWheelSpeed);
        } else {
          leftMotor->setVelocity(-0.05 * maxWheelSpeed);
          rightMotor->setVelocity(0.05 * maxWheelSpeed);
        }
        robot.step(TIME_STEP);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        for (int j = 0; j < 10; j++) {
          if (j == 0) {
            yawAverage = 0;
          }
          yaw = iu->getRollPitchYaw()[2];
          if (yaw > 0) {
            yaw = -1 * yaw;
          }
          yawAverage = yawAverage + yaw;
          robot.step(TIME_STEP); 
        }
        yaw = yawAverage / 10;
      }
    }
    
    // Fix cell spacing
    for (int j = 0; j < 10; j++) {
      if (j == 0) {
        dsFAverage = 0;
      }
      ds[1] = dsF->getValue();
      dsFAverage = dsFAverage + ds[1];
      robot.step(TIME_STEP);
    }
    ds[1] = dsFAverage / 10;
    // 540 < X < 560
    if (ds[1] < 900) {
      while (!(ds[1] < 560 && ds[1] > 540)) {
        if (ds[1] < 560) {
          leftMotor->setVelocity(-0.5 * maxWheelSpeed);
          rightMotor->setVelocity(-0.5 * maxWheelSpeed);
        } else if (ds[1] > 540){
          leftMotor->setVelocity(0.5 * maxWheelSpeed);
          rightMotor->setVelocity(0.5 * maxWheelSpeed);
        }
        robot.step(TIME_STEP);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        for (int j = 0; j < 10; j++) {
          if (j == 0) {
            dsFAverage = 0;
          }
          ds[1] = dsF->getValue();
          dsFAverage = dsFAverage + ds[1];
          robot.step(TIME_STEP);
        }
        ds[1] = dsFAverage / 10;
      }
    }
    
    i++;
    stepCount++;
  }
  return 0;
}