// File:          MXO_main.cpp
// Date: 18/1/25
// Description: simulation robot for a minisumo competition
// Author: DINO
// Modifications:
// init code
// add atuators

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#define TIME_STEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();


  Motor *leftMotor = robot->getMotor("motor_left");
  Motor *rightMotor = robot->getMotor("motor_direita");

  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  
  PositionSensor *encoderL = robot->getPositionSensor("encoder_left") ;
  PositionSensor *encoderR = robot->getPositionSensor("encoder_right") ;
  encoderL->enable(TIME_STEP);
  encoderR->enable(TIME_STEP);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1)
  {
  
    double encoderLeft = encoderL->getValue();
    double encoderRight = encoderR->getValue();
    
    std::cout << "encoder left: " <<encoderLeft;
    std::cout << "| encoder right: " <<encoderRight<<std::endl;
    
    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
